#!/usr/bin/env bash
set -euo pipefail

# Start the PX4 Classic inspection world and drive one complete simulated
# vertical tower scan through MAVLink.  The first non-option argument is the
# PX4 checkout; controller options are passed through unchanged.

script_dir="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
super_root="$(cd -- "${script_dir}/.." && pwd)"
px4_root="${PX4_ROOT:-${HOME}/PX4-Autopilot}"

if [[ $# -gt 0 && "$1" != -* ]]; then
  px4_root="$1"
  shift
fi

world="${PX4_SITL_WORLD:-${super_root}/inspection_gazebo/worlds/px4_inspection.world}"
model="${PX4_SIM_MODEL:-iris_inspection}"
duration="${INSPECTION_DURATION:-45}"
radius="${INSPECTION_RADIUS:-4.0}"
z_min="${INSPECTION_Z_MIN:-1.5}"
z_max="${INSPECTION_Z_MAX:-9.5}"
orbits="${INSPECTION_ORBITS:-2.0}"
rate_hz="${INSPECTION_RATE_HZ:-20.0}"
log_dir="${PX4_INSPECTION_LOG_DIR:-/tmp/super_px4_inspection}"

if [[ "${REGENERATE_WORLD:-0}" == "1" || ! -f "$world" ]]; then
  python3 "${super_root}/inspection_gazebo/scripts/generate_px4_inspection_world.py"
fi

if [[ ! -d "$px4_root" ]]; then
  echo "PX4 checkout not found: $px4_root" >&2
  exit 2
fi
if [[ ! -x "$px4_root/build/px4_sitl_default/bin/px4" ]]; then
  echo "PX4 SITL binary not found: $px4_root/build/px4_sitl_default/bin/px4" >&2
  echo "Build it first with: make -C \"$px4_root\" px4_sitl_default" >&2
  exit 2
fi
if [[ ! -f "$px4_root/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/${model}/${model}.sdf" ]]; then
  echo "PX4 Gazebo model not found: ${model}" >&2
  echo "Install it first with: ${script_dir}/install_px4_inspection_model.sh \"$px4_root\"" >&2
  exit 2
fi
if ! command -v gazebo >/dev/null 2>&1 || ! command -v gzserver >/dev/null 2>&1; then
  echo "Gazebo Classic (gazebo/gzserver) is required" >&2
  exit 2
fi

# PX4 keeps a per-instance lock and sitl_run.sh stops an existing Gazebo
# frontend before starting. Refuse a concurrent run before either can disturb
# a simulator that the operator already has open.
for process_name in px4 gazebo; do
  if pgrep -x "$process_name" >/dev/null 2>&1; then
    echo "An existing ${process_name} process is running; close the other PX4/Gazebo simulation before retrying." >&2
    pgrep -a -x "$process_name" >&2 || true
    exit 2
  fi
done

# PX4 only accepts one coherent stream of Offboard setpoints. Refuse to start
# if a controller from an earlier run is still publishing to the local SITL.
for controller_pattern in '[p]x4_offboard_bridge.py' '[p]x4_mavlink_inspection.py'; do
  stale_controller="$(pgrep -af "$controller_pattern" || true)"
  if [[ -n "$stale_controller" ]]; then
    echo "An inspection controller is already running; stop it before starting this SITL run:" >&2
    echo "$stale_controller" >&2
    exit 2
  fi
done

build_dir="$px4_root/build/px4_sitl_default"
has_gazebo_target() {
  local target_name="gazebo-classic_${model}"
  ninja -C "$build_dir" -t targets all 2>/dev/null \
    | awk -F: -v target="$target_name" '
      {
        name = $1
        sub(/^.*\//, "", name)
        if (name == target) found = 1
      }
      END { exit !found }
    '
}
if ! has_gazebo_target; then
  printf 'PX4 target gazebo-classic_%s is missing in %s. Regenerate it first with:\n  env -u LD_LIBRARY_PATH make -C "%s" px4_sitl_default\n' \
    "$model" "$build_dir" "$px4_root" >&2
  exit 2
fi

mkdir -p "$log_dir"
run_log="${log_dir}/run_$(date +%Y%m%d_%H%M%S)_$$.log"
controller_log="${run_log%.log}.controller.log"
session_pid_file="${run_log%.log}.pgid"
echo "PX4 root: $px4_root"
echo "World: $world"
echo "Model: $model"
echo "Run log: $run_log"
echo "Controller log: $controller_log"
ros_plugin_dir="/opt/ros/humble/lib"
use_ros_plugins="${PX4_INSPECTION_USE_ROS:-0}"
uxrce_agent_bin="${UXRCE_AGENT_BIN:-$(command -v MicroXRCEAgent 2>/dev/null || true)}"
if [[ -z "$uxrce_agent_bin" && -x /tmp/microxrce_install/bin/MicroXRCEAgent ]]; then
  uxrce_agent_bin=/tmp/microxrce_install/bin/MicroXRCEAgent
fi
if [[ -z "$uxrce_agent_bin" && -x "${super_root}/tools/microxrce_install/bin/MicroXRCEAgent" ]]; then
  uxrce_agent_bin="${super_root}/tools/microxrce_install/bin/MicroXRCEAgent"
fi
use_ros_offboard="${PX4_INSPECTION_USE_ROS_OFFBOARD:-0}"
use_fast_lio="${PX4_INSPECTION_USE_FAST_LIO:-0}"
use_super_planner="${PX4_INSPECTION_USE_SUPER_PLANNER:-0}"
fast_lio_workspace="${FAST_LIO_WORKSPACE:-/home/venom/venom_ws}"
if [[ "$use_ros_offboard" == "1" && -z "$uxrce_agent_bin" ]]; then
  echo "PX4 ROS Offboard requested but MicroXRCEAgent is missing" >&2
  echo "Install/build eProsima/Micro-XRCE-DDS-Agent first, then retry" >&2
  exit 2
fi
agent_pid=""
ros_bridge_pid=""
ros_bridge_pgid_file=""
if [[ "$use_ros_offboard" == "1" ]]; then
  agent_lib_dir="$(dirname "$uxrce_agent_bin")/../lib"
  LD_LIBRARY_PATH="${agent_lib_dir}:${LD_LIBRARY_PATH:-}" "$uxrce_agent_bin" udp4 -p 8888 >"${run_log%.log}.uxrce.log" 2>&1 &
  agent_pid=$!
  echo "MicroXRCEAgent: $uxrce_agent_bin udp4 -p 8888 (pid $agent_pid)"
  sleep 1
  if ! kill -0 "$agent_pid" 2>/dev/null; then
    echo "MicroXRCEAgent failed to start; see ${run_log%.log}.uxrce.log" >&2
    exit 2
  fi
  super_ws="${SUPER_WORKSPACE:-${super_root}/../super_ws}"
  if [[ ! -f /opt/ros/humble/setup.bash || ! -f "${super_ws}/install/setup.bash" ]]; then
    echo "ROS 2 workspace is required for automatic Offboard bridge startup" >&2
    exit 2
  fi
  if [[ "$use_fast_lio" == "1" ]]; then
    if [[ ! -f "${fast_lio_workspace}/install/setup.bash" ]]; then
      echo "FAST-LIO2 workspace not found: ${fast_lio_workspace}/install/setup.bash" >&2
      echo "Set FAST_LIO_WORKSPACE to a workspace containing fast_lio and livox_ros_driver2." >&2
      exit 2
    fi
    fast_lio_config="${PX4_FAST_LIO_CONFIG:-${fast_lio_workspace}/install/fast_lio/share/fast_lio/config/mid360.yaml}"
    if [[ ! -f "$fast_lio_config" ]]; then
      echo "FAST-LIO2 MID-360 config not found: $fast_lio_config" >&2
      exit 2
    fi
  else
    fast_lio_config=""
  fi
  set +u
  unset AMENT_CURRENT_PREFIX AMENT_SHELL
  source /opt/ros/humble/setup.bash
  if [[ "$use_fast_lio" == "1" ]]; then
    # The sensor/FAST-LIO overlay provides livox_ros_driver2 and fast_lio;
    # source the project overlay last so its PX4 message contract wins.
    source "${fast_lio_workspace}/install/setup.bash"
  fi
  source "${super_ws}/install/setup.bash"
  set -u
  if ! ros2 interface show px4_msgs/msg/VehicleStatus >/dev/null 2>&1; then
    echo "px4_msgs/msg/VehicleStatus is missing from the current ROS install." >&2
    echo "Rebuild with: ${script_dir}/build_ros2_humble.sh" >&2
    exit 2
  fi
  pointcloud_topic="${PX4_ROS_POINTCLOUD_TOPIC:-}"
  planning_frame="${PX4_ROS_PLANNING_FRAME:-}"
  if [[ -z "$pointcloud_topic" ]]; then
    if [[ "$use_fast_lio" == "1" ]]; then
      pointcloud_topic="/cloud_registered"
    else
      pointcloud_topic="/inspection/tower_points"
    fi
  fi
  if [[ -z "$planning_frame" ]]; then
    if [[ "$use_fast_lio" == "1" ]]; then
      planning_frame="odom"
    else
      planning_frame="map"
    fi
  fi
  fast_lio_launch_arg="false"
  if [[ "$use_fast_lio" == "1" ]]; then
    fast_lio_launch_arg="true"
  fi
  super_planner_launch_arg="false"
  if [[ "$use_super_planner" == "1" ]]; then
    super_planner_launch_arg="true"
    if [[ "$use_fast_lio" != "1" ]]; then
      echo "SUPER planner mode requires PX4_INSPECTION_USE_FAST_LIO=1 so /cloud_registered and /odom feed ROG-Map." >&2
      exit 2
    fi
  fi
  fast_lio_rviz="${PX4_ROS_FAST_LIO_RVIZ:-false}"
  case "${HEADLESS:-}" in
    1|true|TRUE|yes|YES) fast_lio_rviz="false" ;;
  esac
  bridge_log="${run_log%.log}.ros_offboard.log"
  ros_bridge_pgid_file="${run_log%.log}.ros_offboard.pgid"
  # Keep the ROS launch tree in its own process group and record the actual
  # session leader.  ``setsid --wait ros2 launch`` can leave launch-created
  # nodes orphaned when the wrapper exits, which would block the next SITL run.
  setsid --fork --wait bash -c '
    printf "%s\\n" "$$" > "$1"
    shift
    exec "$@"
  ' _ "$ros_bridge_pgid_file" ros2 launch inspection_manager tower_pointcloud_demo.launch.py \
    enable_px4_setpoints:=true \
    request_offboard:="${PX4_ROS_REQUEST_OFFBOARD:-true}" \
    request_arm:="${PX4_ROS_REQUEST_ARM:-true}" \
    health_sequence:="${PX4_ROS_HEALTH_SEQUENCE:-READY:120}" \
    obstacle_amplitude:="${PX4_ROS_OBSTACLE_AMPLITUDE:-0.0}" \
    obstacle_radius:="${PX4_ROS_OBSTACLE_RADIUS:-0.0}" \
    tower_x:="${PX4_ROS_TOWER_X:-0.0}" \
    tower_y:="${PX4_ROS_TOWER_Y:-0.0}" \
    tower_height_m:="${PX4_ROS_TOWER_HEIGHT_M:-10.0}" \
    pointcloud_topic:="$pointcloud_topic" \
    planning_frame:="$planning_frame" \
    use_fast_lio:="$fast_lio_launch_arg" \
    use_super_planner:="$super_planner_launch_arg" \
    super_config_name:="${PX4_SUPER_CONFIG_NAME:-inspection_super_px4.yaml}" \
    super_goal_stride:="${PX4_SUPER_GOAL_STRIDE:-8}" \
    fast_lio_config_path:="${fast_lio_config}" \
    fast_lio_rviz:="$fast_lio_rviz" \
    path_duration_s:="${PX4_ROS_PATH_DURATION_S:-75.0}" >"${bridge_log}" 2>&1 &
  ros_bridge_pid=$!
  echo "ROS Offboard bridge: inspection_manager/tower_pointcloud_demo.launch.py (pid $ros_bridge_pid)"
fi
if [[ "$use_ros_plugins" == "1" && -f /opt/ros/humble/setup.bash \
  && -f "$ros_plugin_dir/libgazebo_ros_init.so" \
  && -f "$ros_plugin_dir/libgazebo_ros_factory.so" \
  && -f "$ros_plugin_dir/libgazebo_ros_camera.so" \
  && -f "$ros_plugin_dir/libgazebo_ros_ray_sensor.so" ]]; then
  set +u
  unset AMENT_CURRENT_PREFIX AMENT_SHELL
  source /opt/ros/humble/setup.bash
  set -u
  echo "ROS Gazebo plugins: available; ROS 2 sensor topics enabled"
else
  unset ROS_VERSION
  unset AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX AMENT_SHELL COLCON_PREFIX_PATH
  echo "ROS Gazebo plugins: disabled; using stable native PX4/MAVLink simulation"
fi

# Put the PX4/Gazebo tree in a dedicated session. The child writes its own
# process-group id after setsid, so cleanup also works from an interactive
# shell where setsid has to fork before it can create the new session.
headless=""
case "${HEADLESS:-}" in
  1|true|TRUE|yes|YES) headless=1 ;;
esac
verbose_sim=""
case "${PX4_SIM_VERBOSE:-}" in
  1|true|TRUE|yes|YES) verbose_sim=1 ;;
esac
setsid --fork --wait bash -c '
  printf "%s\\n" "$$" > "$1"
  shift
  exec "$@"
' _ "$session_pid_file" env \
  ROS_VERSION= \
  AMENT_PREFIX_PATH= \
  PX4_INSPECTION_NATIVE=1 \
  HEADLESS="$headless" \
  GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}" \
  GAZEBO_IP="${GAZEBO_IP:-127.0.0.1}" \
  GAZEBO_HOST="${GAZEBO_HOST:-127.0.0.1}" \
  PX4_SITL_WORLD="$world" \
  PX4_SIM_MODEL="$model" \
  VERBOSE_SIM="$verbose_sim" \
  NO_PXH=1 \
  ninja -C "$build_dir" "gazebo-classic_${model}" >"$run_log" 2>&1 &
launcher_pid=$!

stop_ros_bridge() {
  if [[ -r "${ros_bridge_pgid_file:-}" ]]; then
    local bridge_pgid=""
    read -r bridge_pgid < "$ros_bridge_pgid_file" || true
    if [[ "$bridge_pgid" =~ ^[0-9]+$ ]] && [[ "$bridge_pgid" != "$$" ]]; then
      kill -TERM -- "-$bridge_pgid" 2>/dev/null || true
    fi
  fi
  if [[ -n "${ros_bridge_pid:-}" ]] && kill -0 "$ros_bridge_pid" 2>/dev/null; then
    kill -TERM "$ros_bridge_pid" 2>/dev/null || true
    wait "$ros_bridge_pid" 2>/dev/null || true
  fi
}

cleanup() {
  status=$?
  trap - EXIT INT TERM
  if [[ -n "${agent_pid:-}" ]] && kill -0 "$agent_pid" 2>/dev/null; then
    kill "$agent_pid" 2>/dev/null || true
    wait "$agent_pid" 2>/dev/null || true
  fi
  if [[ -n "${ros_bridge_pid:-}" || -r "${ros_bridge_pgid_file:-}" ]]; then
    # ros2 launch and all of its sensor/planner/controller nodes are isolated
    # in one session so cleanup cannot leave an orphaned Offboard publisher.
    stop_ros_bridge
  fi
  if [[ -n "${controller_pid:-}" ]] && kill -0 "$controller_pid" 2>/dev/null; then
    kill "$controller_pid" 2>/dev/null || true
    wait "$controller_pid" 2>/dev/null || true
  fi
  if [[ -r "$session_pid_file" ]]; then
    read -r session_pgid < "$session_pid_file"
    kill -- "-$session_pgid" 2>/dev/null || true
  fi
  if kill -0 "$launcher_pid" 2>/dev/null; then
    kill "$launcher_pid" 2>/dev/null || true
    wait "$launcher_pid" 2>/dev/null || true
  fi
  # Simulator children share the dedicated session process group above.
  # Avoid broad name/path kills that could terminate another live SITL run.
  exit "$status"
}
trap cleanup EXIT INT TERM

controller_args=(
  --connection "${PX4_MAVLINK_CONNECTION:-udpin:0.0.0.0:14540}"
  --duration "$duration"
  --radius "$radius"
  --z-min "$z_min"
  --z-max "$z_max"
  --orbits "$orbits"
  --rate-hz "$rate_hz"
)
if [[ $# -gt 0 ]]; then
  controller_args+=("$@")
fi

if [[ "$use_ros_offboard" == "1" ]]; then
  echo "ROS Offboard mode active; MAVLink controller is disabled to avoid competing control sources."
  echo "Waiting for the ROS bridge to finish both inspection orbits and confirm PX4 disarm..."
  controller_pid=""
  # The bridge owns the reference path and NAV_LAND request. Keep the runner
  # alive long enough for the configured scan and landing, then use its log
  # markers as the completion contract.
  # Completion requires VehicleStatus to confirm AUTO_LAND and then disarmed.
  # A NAV_LAND command alone is not proof that landing finished.
  deadline=$(( $(date +%s) + ${PX4_ROS_OFFBOARD_TIMEOUT_S:-300} ))
  while ! grep -q "PX4_DISARMED_AFTER_LAND" "$bridge_log"; do
    if ! kill -0 "$ros_bridge_pid" 2>/dev/null; then
      echo "ROS Offboard launch exited before PX4 confirmed disarm; see $bridge_log" >&2
      exit 2
    fi
    if grep -Eq "PX4_UNEXPECTED_AUTO_LAND|PX4_MODE_EXITED" "$bridge_log"; then
      echo "PX4 left the planned Offboard mission before completion; see $bridge_log" >&2
      exit 2
    fi
    if (( $(date +%s) >= deadline )); then
      echo "ROS Offboard mission/landing timeout; PX4 did not confirm disarm; see $bridge_log" >&2
      exit 2
    fi
    sleep 2
  done
  # The ROS bridge can keep spinning after PX4 exits.  Treat a completed
  # landing/disarm marker as the contract and stop the bridge before leaving
  # the runner, so it cannot continue reporting stale PX4 position forever.
  stop_ros_bridge
  if ! grep -q "OUTPUTTING_SETPOINT" "$bridge_log"; then
    echo "ROS Offboard bridge produced no setpoint output; see $bridge_log" >&2
    exit 2
  fi
  if ! grep -q "PX4_DISARMED_AFTER_LAND" "$bridge_log"; then
    echo "ROS Offboard bridge did not confirm landing and disarm; see $bridge_log" >&2
    exit 2
  fi
  echo "ROS Offboard inspection SITL completed successfully"
  exit 0
fi

echo "Waiting for PX4 heartbeat; the controller will start after SITL accepts UDP 14540..."
python3 "$script_dir/px4_mavlink_inspection.py" "${controller_args[@]}" >"$controller_log" 2>&1 &
controller_pid=$!
if wait "$controller_pid"; then
  controller_status=0
else
  controller_status=$?
fi
if [[ "$controller_status" -ne 0 ]]; then
  echo "PX4 inspection controller failed; see $controller_log and $run_log" >&2
  exit "$controller_status"
fi

echo "PX4 inspection SITL completed successfully"
