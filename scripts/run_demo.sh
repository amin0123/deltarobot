#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
isaac_root="${ISAAC_SIM_PATH:-/home/amin24/isaacsim}"
output_dir="${DELTA_OUTPUT:-$project_root/artifacts/latest}"
attempts="${DELTA_ATTEMPTS:-24}"
config_file="${DELTA_CONFIG:-$project_root/ros2_ws/src/delta_sorting/config/cell.yaml}"
mkdir -p "$output_dir"
set +u
source /opt/ros/jazzy/setup.bash
source "$project_root/ros2_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
cd "$project_root"
setsid ros2 launch delta_sorting sorting.launch.py \
  "config_file:=$config_file" \
  "max_attempts:=$attempts" "report_dir:=$output_dir" > "$output_dir/ros.log" 2>&1 &
ros_pid=$!
cleanup() {
  kill -INT -- "-$ros_pid" 2>/dev/null || true
  wait "$ros_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
"$isaac_root/python.sh" sim/run_cell.py --config "$config_file" --output "$output_dir" "$@" 2>&1 | tee "$output_dir/isaac.log"
