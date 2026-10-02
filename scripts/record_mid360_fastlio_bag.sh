#!/usr/bin/env bash
set -euo pipefail

# Record the verified MID-360/Livox -> FAST-LIO2 evidence topics. This script
# is intended for the user's desktop ROS 2 environment where DDS sockets are
# available; it deliberately fails early when ros2 bag is unavailable.
duration="${BAG_DURATION:-60}"
output_root="${BAG_OUTPUT_ROOT:-$PWD/artifacts/rosbags}"
domain="${ROS_DOMAIN_ID:-42}"
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 is required; source /opt/ros/humble/setup.bash first" >&2
  exit 2
fi
mkdir -p "$output_root"
stamp="$(date +%Y%m%d_%H%M%S)"
bag="$output_root/mid360_fastlio_${stamp}"
export ROS_DOMAIN_ID="$domain"
topics=(
  /livox/lidar
  /livox/imu
  /cloud_registered
  /cloud_registered_body
  /odom
  /path
  /tf
  /tf_static
)
echo "Recording ${duration}s to ${bag} (ROS_DOMAIN_ID=${ROS_DOMAIN_ID})"
set +e
timeout --signal=INT --kill-after=5s "$duration" \
  ros2 bag record --storage sqlite3 --output "$bag" "${topics[@]}"
record_status=$?
set -e
# GNU timeout returns 124 when the requested duration expires.  The recorder
# has already received SIGINT and flushed its cache in that case, so it is a
# successful bounded capture.  Preserve real recorder failures.
if [[ "$record_status" -ne 0 && "$record_status" -ne 124 && "$record_status" -ne 130 ]]; then
  echo "ros2 bag record failed with status ${record_status}" >&2
  exit "$record_status"
fi
ros2 bag info "$bag" | tee "${bag}.info.txt"
echo "ROS_BAG_RECORD_OK $bag"
