#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$project_root/ros2_ws"
colcon build --symlink-install --event-handlers console_direct+
