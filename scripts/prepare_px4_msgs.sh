#!/usr/bin/env bash
set -euo pipefail

script_dir="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
super_root="$(cd -- "${script_dir}/.." && pwd)"
workspace="${1:-${SUPER_WORKSPACE:-${super_root}/../super_ws}}"
px4_msgs_source="${workspace}/src/px4_msgs"
status_source="${super_root}/px4_offboard_bridge/px4_msgs/VehicleStatus.msg"
status_target="${px4_msgs_source}/msg/VehicleStatus.msg"
cmake_file="${px4_msgs_source}/CMakeLists.txt"

if [[ ! -d "${px4_msgs_source}/msg" || ! -f "$cmake_file" ]]; then
  echo "px4_msgs source package not found in ${workspace}/src" >&2
  exit 2
fi

if [[ -f "$status_target" ]]; then
  if ! cmp -s "$status_source" "$status_target"; then
    echo "${status_target} exists but differs from the PX4 VehicleStatus version bundled with SUPER." >&2
    echo "Align the px4_msgs and PX4 source revisions before building; refusing to overwrite it." >&2
    exit 2
  fi
else
  install -m 0644 "$status_source" "$status_target"
fi

if rg -q 'VehicleStatus\.msg' "$cmake_file"; then
  if ! rg -q 'VehicleStatus\.msg"\)' "$cmake_file"; then
    echo "VehicleStatus.msg is listed outside the PX4_MSGS interface list in ${cmake_file}." >&2
    echo "Repair that CMakeLists.txt before rebuilding; refusing to guess at a local modification." >&2
    exit 2
  fi
else
  if ! rg -q 'VehicleLocalPosition\.msg' "$cmake_file"; then
    echo "Cannot add VehicleStatus.msg to unrecognized px4_msgs CMakeLists.txt: ${cmake_file}" >&2
    exit 2
  fi
  sed -i '/VehicleLocalPosition\.msg")/c\  "${CMAKE_CURRENT_SOURCE_DIR}/msg:VehicleLocalPosition.msg"\n  "${CMAKE_CURRENT_SOURCE_DIR}/msg:VehicleStatus.msg")' "$cmake_file"
fi

echo "px4_msgs VehicleStatus interface is ready in ${px4_msgs_source}"
