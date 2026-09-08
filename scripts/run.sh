#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
source "${ORBBEC_WS:-$HOME/orbbec_ws}/install/setup.bash"
source install/setup.bash
exec python3 -m tracking.launch "$@"
