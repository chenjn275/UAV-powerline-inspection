#!/usr/bin/env bash
set -euo pipefail

px4_root="${1:-$HOME/PX4-Autopilot}"
script_dir="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sensor_templates="${script_dir%/scripts}/inspection_gazebo/models"
model_dir="$px4_root/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_inspection"
airframe="$px4_root/ROMFS/px4fmu_common/init.d-posix/airframes/1018_gazebo-classic_iris_inspection"
models_root="$px4_root/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"

mkdir -p "$model_dir"
cat > "$model_dir/model.config" <<'EOF'
<?xml version="1.0"?>
<model>
  <name>PX4 Iris inspection sensors</name>
  <version>1.0</version>
  <sdf version="1.5">iris_inspection.sdf</sdf>
  <description>PX4 Iris with depth camera and 360 degree inspection lidar.</description>
</model>
EOF
cat > "$model_dir/inspection_sensor_links.xml" <<'EOF'
<?xml version="1.0" ?>
    <link name="d435i_mount">
      <pose>0.10 0 -0.03 0 0 0</pose>
      <inertial><mass>0.01</mass><inertia><ixx>0.00001</ixx><iyy>0.00001</iyy><izz>0.00001</izz></inertia></inertial>
      <visual name="d435i_body"><geometry><box><size>0.12 0.035 0.035</size></box></geometry>
        <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.12 0.12 0.12 1</diffuse></material></visual>
    </link>
    <joint name="d435i_mount_joint" type="fixed"><parent>base_link</parent><child>d435i_mount</child></joint>
    <link name="mid360_mount"><pose>0 0 0.12 0 0 0</pose>
      <inertial><mass>0.02</mass><inertia><ixx>0.00002</ixx><iyy>0.00002</iyy><izz>0.00002</izz></inertia></inertial>
      <visual name="mid360_body"><geometry><cylinder><radius>0.045</radius><length>0.06</length></cylinder></geometry>
        <material><ambient>0.08 0.08 0.08 1</ambient><diffuse>0.18 0.18 0.18 1</diffuse></material></visual>
    </link>
    <joint name="mid360_mount_joint" type="fixed"><parent>base_link</parent><child>mid360_mount</child></joint>
EOF
cp "$models_root/iris/iris.sdf" "$model_dir/iris_inspection.sdf"
python3 - "$model_dir/iris_inspection.sdf" "$model_dir/inspection_sensor_links.xml" <<'PY'
import pathlib, sys
model, sensors = map(pathlib.Path, sys.argv[1:])
text = model.read_text()
extra = sensors.read_text().split('<?xml version="1.0" ?>', 1)[-1]
text = text.replace('</model>', extra + '\n</model>')
text = text.replace('<model name="iris">', '<model name="iris_inspection">', 1)
model.write_text(text)
PY
rm -f "$model_dir/inspection_sensor_links.xml"
 : <<'OLD_WRAPPER'
    <link name="d435i_mount">
      <pose>0.10 0 -0.03 0 0 0</pose>
      <visual name="d435i_body"><geometry><box><size>0.12 0.035 0.035</size></box></geometry>
        <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.12 0.12 0.12 1</diffuse></material></visual>
      <visual name="d435i_lens"><pose>0.061 0 0 0 0 0</pose><geometry><cylinder><radius>0.012</radius><length>0.006</length></cylinder></geometry>
        <material><ambient>0.02 0.08 0.16 1</ambient><diffuse>0.03 0.20 0.45 1</diffuse></material></visual>
    </link>
    <joint name="d435i_mount_joint" type="fixed"><parent>iris::base_link</parent><child>iris_inspection::d435i_mount</child></joint>
    <link name="mid360_mount">
      <pose>0 0 0.12 0 0 0</pose>
      <visual name="mid360_body"><geometry><cylinder><radius>0.045</radius><length>0.06</length></cylinder></geometry>
        <material><ambient>0.08 0.08 0.08 1</ambient><diffuse>0.18 0.18 0.18 1</diffuse></material></visual>
      <visual name="mid360_ring"><pose>0 0 0.031 0 0 0</pose><geometry><cylinder><radius>0.035</radius><length>0.004</length></cylinder></geometry>
        <material><ambient>0.12 0.35 0.12 1</ambient><diffuse>0.20 0.70 0.20 1</diffuse></material></visual>
    </link>
    <joint name="mid360_mount_joint" type="fixed"><parent>iris::base_link</parent><child>iris_inspection::mid360_mount</child></joint>
  </model>
</sdf>
EOF
OLD_WRAPPER

# Keep the ROS-facing sensor models separate from PX4's rangefinder model.  The
# stock ``model://lidar`` is a one-beam PX4 plugin and cannot publish a ROS
# PointCloud2; these two templates provide D435i/MID-360-shaped ROS topics.
for sensor_model in inspection_depth_camera inspection_lidar; do
  source_model="$sensor_templates/$sensor_model"
  if [ ! -d "$source_model" ]; then
    echo "missing sensor template: $sensor_model (looked under $sensor_templates)" >&2
    exit 1
  fi
  mkdir -p "$models_root/$sensor_model"
  cp "$source_model/model.config" "$models_root/$sensor_model/model.config"
  cp "$source_model/$sensor_model.sdf" "$models_root/$sensor_model/$sensor_model.sdf"
done

cat > "$airframe" <<'EOF'
#!/bin/sh
# @name PX4 Iris inspection vehicle: depth camera plus 360 lidar
# @type Quadrotor Wide
. ${R}etc/init.d-posix/airframes/10015_gazebo-classic_iris
EOF
chmod +x "$airframe"

# PX4 v1.16 keeps the Classic model list explicit; an SDF alone does not
# create a Ninja target. Add our model once and keep the airframe list in sync.
classic_cmake="$px4_root/src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake"
if ! grep -q '^\s*iris_inspection\s*$' "$classic_cmake"; then
  sed -i '/^[[:space:]]*iris_depth_camera[[:space:]]*$/a\		iris_inspection' "$classic_cmake"
fi
airframes_cmake="$px4_root/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"
sed -i '/10018_gazebo-classic_iris_inspection/d' "$airframes_cmake"
if ! grep -q '1018_gazebo-classic_iris_inspection' "$airframes_cmake"; then
  sed -i '/10018_gazebo-classic_iris_foggy_lidar/a\	1018_gazebo-classic_iris_inspection' "$airframes_cmake"
fi
rm -f "$px4_root/ROMFS/px4fmu_common/init.d-posix/airframes/10018_gazebo-classic_iris_inspection"
printf 'installed model: %s\ninstalled airframe: %s\n' "$model_dir" "$airframe"
printf '%s\n' 'Reconfigure PX4 SITL after this script so CMake discovers the new airframe.'
