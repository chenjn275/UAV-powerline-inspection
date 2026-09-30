#!/usr/bin/env bash
set -euo pipefail

# ROS 2 Humble's rosidl_adapter can corrupt UTF-8 source paths while CMake
# reads the generated IDL tuple list.  Keep build and log paths ASCII even
# when the checkout itself lives under a non-ASCII directory.
super_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# The checkout normally keeps its colcon workspace beside the source packages
# (``<project>/super_ws``).  Older instructions used a sibling workspace
# (``<project>/../super_ws``), so keep that location as a fallback for existing
# users while preferring the workspace that is actually part of this checkout.
if [[ -n "${SUPER_WORKSPACE:-}" ]]; then
  workspace="${SUPER_WORKSPACE}"
elif [[ -d "${super_root}/super_ws/src" ]]; then
  workspace="${super_root}/super_ws"
elif [[ -d "${super_root}/../super_ws/src" ]]; then
  workspace="${super_root}/../super_ws"
else
  workspace="${super_root}/super_ws"
fi
build_base="${SUPER_BUILD_BASE:-/tmp/super_build}"
log_base="${SUPER_LOG_BASE:-/tmp/super_log}"
install_base="${SUPER_INSTALL_BASE:-"${workspace}/install"}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble" >&2
  exit 1
fi
if [[ ! -d "${workspace}/src" ]]; then
  echo "ROS workspace source directory not found: ${workspace}/src" >&2
  exit 1
fi

# ROS setup templates intentionally read a few optional variables before
# assigning defaults, so disable nounset only while sourcing them.
set +u
# A previously sourced overlay can leave AMENT_CURRENT_PREFIX pointing at a
# removed checkout.  Reset the setup state before sourcing the underlay so a
# fresh build does not try to read ``<old-prefix>/setup.sh``.
unset AMENT_CURRENT_PREFIX AMENT_SHELL
source /opt/ros/humble/setup.bash
set -u

if [[ "${CLEAN_BUILD:-0}" == "1" ]]; then
  rm -rf -- "${build_base}" "${log_base}" "${install_base}"
fi
mkdir -p -- "${build_base}" "${log_base}"

# Keep the install tree self-contained. Symlink-install points setup hooks
# into the temporary build directory, which may be removed after a build.
colcon --log-base "${log_base}" build \
  --build-base "${build_base}" \
  --install-base "${install_base}" \
  "$@"

echo "Build completed. Source: ${install_base}/setup.bash"
