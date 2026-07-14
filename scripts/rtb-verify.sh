#!/usr/bin/env bash
# Integration check for the Robotics Toolbox bridge: spin up the Python
# rtb-kinematics service (real robot models) and run the C++ rtb_dev against
# it — a real UR10 Cartesian moveL, planned by the mature library, landing on
# target. Mirrors the URSim integration pattern (needs the sidecar running).
#
#   scripts/rtb-verify.sh          # builds rtb_dev, sets up a venv, runs both
set -euo pipefail
cd "$(dirname "$0")/.."

VENV=${VENV:-/tmp/robonode-rtb-venv}
if [ ! -x "$VENV/bin/python" ]; then
  python3 -m venv "$VENV"
  "$VENV/bin/pip" install -q -r services/rtb-kinematics/requirements.txt
fi

echo "== service self-test =="
( cd services/rtb-kinematics && "$VENV/bin/python" test_service.py )

echo "== build rtb_dev =="
cmake -B build >/dev/null
cmake --build build -j --target rtb_dev >/dev/null

echo "== start service + run rtb_dev =="
"$VENV/bin/python" services/rtb-kinematics/service.py --port 8091 >/tmp/rtb-verify-svc.log 2>&1 &
SVC=$!
trap 'kill $SVC 2>/dev/null || true' EXIT
for _ in $(seq 1 20); do
  curl -sf -X POST http://localhost:8091/health -d '{}' >/dev/null 2>&1 && break
  sleep 1
done
./build/apps/rtb_dev/rtb_dev ur10
