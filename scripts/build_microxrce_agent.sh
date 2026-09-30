#!/usr/bin/env bash
set -euo pipefail

install_prefix="${UXRCE_AGENT_PREFIX:-/tmp/microxrce_install}"
source_dir="${MICROXRCE_AGENT_SOURCE:-/tmp/Micro-XRCE-DDS-Agent}"

if [[ ! -d "${source_dir}" ]]; then
  cat >&2 <<EOF
Micro-XRCE-DDS-Agent source was not found at ${source_dir}.
Fetch it first, for example:
  git clone --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git ${source_dir}
Then rerun this script.
EOF
  exit 2
fi

cmake -S "${source_dir}" -B "${source_dir}/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${install_prefix}"
cmake --build "${source_dir}/build" --parallel "${BUILD_JOBS:-2}"
cmake --install "${source_dir}/build"

agent="${install_prefix}/bin/MicroXRCEAgent"
if [[ ! -x "${agent}" ]]; then
  echo "Agent build completed but executable is missing: ${agent}" >&2
  exit 1
fi
echo "UXRCE_AGENT_BIN=${agent}"
echo "Run: ${agent} udp4 -p 8888"
