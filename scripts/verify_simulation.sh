#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${project_root}/../super_ws"
export PYTHONNOUSERSITE=1
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="${project_root}/inspection_core:${project_root}/inspection_manager:${project_root}/px4_offboard_bridge/scripts:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
export LD_LIBRARY_PATH="/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

set +u
source /opt/ros/humble/setup.bash
set -u

echo "[1/4] Python regression tests"
python3 -m pytest -q \
  "${project_root}/inspection_core/test" \
  "${project_root}/inspection_manager/test" \
  "${project_root}/px4_offboard_bridge/test"

echo "[2/4] Python compilation"
python3 -m compileall -q \
  "${project_root}/inspection_core" \
  "${project_root}/inspection_manager" \
  "${project_root}/inspection_gazebo" \
  "${project_root}/px4_offboard_bridge"

echo "[3/4] Scenario and artifact validation"
PYTHONPATH="${project_root}/inspection_core" \
  python3 "${project_root}/inspection_core/scripts/run_scenario_suite.py" >/tmp/super_scenario_suite.json
python3 "${project_root}/inspection_core/scripts/validate_artifacts.py" \
  "${project_root}/artifacts/ablation_batch_150.json"

echo "[4/4] Installed ROS packages"
if [[ ! -f "${workspace}/install/setup.bash" ]]; then
  echo "missing install setup: ${workspace}/install/setup.bash" >&2
  exit 1
fi
set +u
source "${workspace}/install/setup.bash"
set -u
for package in inspection_manager inspection_gazebo px4_offboard_bridge px4_msgs; do
  ros2 pkg prefix "${package}"
done

native_world="${project_root}/inspection_gazebo/worlds/inspection_line_native.world"
if grep -Eq 'libgazebo_ros_(camera|ray_sensor)|inspection_depth_camera|inspection_lidar' "${native_world}"; then
  echo "native Gazebo world unexpectedly contains ROS sensor plugins" >&2
  exit 1
fi
echo "NATIVE_GAZEBO_WORLD_STATIC_CHECK_OK"

echo "SIMULATION_VERIFICATION_OK"
