#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
set +u
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID="${DELTA_TEST_DOMAIN:-43}"
python3 - <<'PY'
from pathlib import Path
import yaml
root=Path('artifacts/edge_tests')
root.mkdir(parents=True,exist_ok=True)
cfg=yaml.safe_load(Path('ros2_ws/src/delta_sorting/config/cell.yaml').read_text())
cfg['caps']['count']=2
cfg['task']['empty_feed_timeout']=7.0
cfg['task']['autostart']=True
(root/'automatic.yaml').write_text(yaml.safe_dump(cfg))
cfg['task']['autostart']=False
(root/'manual.yaml').write_text(yaml.safe_dump(cfg))
PY
export DELTA_CONFIG="$project_root/artifacts/edge_tests/manual.yaml"
export DELTA_OUTPUT="$project_root/artifacts/edge_tests/interlocks"
# Do not exit on the evaluator's intentional stop fault: this test must reset
# and command another motion after testing the latched emergency stop.
scripts/run_demo.sh --headless --scenario empty --max-time 15 > artifacts/edge_tests/interlocks_console.log 2>&1 &
edge_pid=$!
python3 scripts/check_interlocks.py --output "$DELTA_OUTPUT/checks.json"
wait "$edge_pid"
export DELTA_CONFIG="$project_root/artifacts/edge_tests/automatic.yaml"
export DELTA_ATTEMPTS=1
for scenario in empty uncertain failed_grasp; do
  export DELTA_OUTPUT="$project_root/artifacts/edge_tests/$scenario"
  scripts/run_demo.sh --headless --scenario "$scenario" --max-time 65 --exit-on-complete \
    > "artifacts/edge_tests/${scenario}_console.log" 2>&1
done
python3 - <<'PY'
import json
from pathlib import Path
p=Path('artifacts/edge_tests')
assert json.loads((p/'interlocks/checks.json').read_text())['passed']
for scenario in ['empty','uncertain']:
 r=json.loads((p/scenario/'results.json').read_text())
 assert r['finished'] and r['attempted']==0 and r['actual_gripper_requests']==0, (scenario,r)
r=json.loads((p/'failed_grasp/results.json').read_text())
assert r['finished'] and r['attempted']==1 and r['successful_picks']==0 and r['failures']==1, r
assert r['actual_gripper_requests']==2, r
print('All live negative/recovery checks passed.')
PY
