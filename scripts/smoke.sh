#!/usr/bin/env bash
# Deploy smoke: drive the built CLI (in-process Platform — the same facade the
# web server binds to) through every subsystem end to end. Exit 0 means the
# deployable binary actually works: cell boot, RT motion + physics, capability
# swaps, sandbox authoring, Cartesian IK, the program engine, stations.
#
#   scripts/smoke.sh            # uses ./build
#   BUILD_DIR=out scripts/smoke.sh
set -uo pipefail
cd "$(dirname "$0")/.."
CLI="${BUILD_DIR:-build}/apps/robonode_cli/robonode_cli"
[ -x "$CLI" ] || { echo "FAIL: $CLI not built (cmake --build ${BUILD_DIR:-build})"; exit 1; }

fail=0
check() { if eval "$2"; then echo "  ok  $1"; else echo "  FAIL $1"; fail=1; fi; }
j() { python3 -c "import sys,json;$1"; }

check "cell boots (descriptor-driven, 7 nodes)" \
  "$CLI nodes 2>/dev/null | grep -q rail-x"
check "RT motion + physics + worker queue + telemetry" \
  "$CLI run 2>/dev/null | j 'sys.exit(0 if abs(json.load(sys.stdin)[\"pos\"][0]-400)<30 else 1)'"
check "trajectory capability swap (moveJ)" \
  "$CLI version planner robonode.moveJ 2>/dev/null | grep -q moveJ"
check "control capability swap (smooth)" \
  "$CLI version control robonode.smooth 2>/dev/null | grep -q smooth"
check "vision authoring (sandbox compile + register)" \
  "$CLI define vision smoke 'x; y; z + 0.07' 2>/dev/null | grep -q user.smoke"
check "user trajectory code plans the move (sandboxed via+end reaches the goal)" \
  "$CLI define planner smokepath 'x; y; z + 0.2; x; y; z' 2>/dev/null | grep -q user.smokepath && \
   $CLI version planner user.smokepath 2>/dev/null | grep -q user.smokepath && \
   $CLI movel 0.9 0.25 0.35 2>/dev/null | j 'j=json.load(sys.stdin);t=j.get(\"tcp\",[0,0,0]);sys.exit(0 if sum((t[i]-[0.9,0.25,0.35][i])**2 for i in range(3))**0.5<0.03 else 1)'"
check "a user module outlives the process that authored it" \
  "$CLI define vision persisted 'x; y; z + 0.01' >/dev/null 2>&1; $CLI modules 2>/dev/null | grep -q persisted"
check "the bring-your-own driver can be selected and drives the axis" \
  "$CLI swap rail-x robonode.byo-example 2>/dev/null | grep -q byo-example && \
   $CLI run 2>/dev/null | j 'sys.exit(0 if abs(json.load(sys.stdin)[\"pos\"][0]-400)<40 else 1)'"
check "Cartesian moveL (real IK reaches target)" \
  "$CLI movel 0.9 0.25 0.35 2>/dev/null | j 'j=json.load(sys.stdin);t=j.get(\"tcp\",[0,0,0]);sys.exit(0 if sum((t[i]-[0.9,0.25,0.35][i])**2 for i in range(3))**0.5<0.03 else 1)'"
check "program engine (bin-picking app runs to completion)" \
  "$CLI app bin-picking.app.json 2>/dev/null | j 'sys.exit(0 if not json.load(sys.stdin).get(\"running\",True) else 1)'"
check "stations (data-driven)" \
  "$CLI stations 2>/dev/null | grep -q conveyor"
check "6-DoF: an app selects the pose planner and reaches point + angle" \
  "$CLI app pose-demo.app.json 2>/dev/null | j 'j=json.load(sys.stdin);t=j.get(\"tcp\",[0,0,0]);sys.exit(0 if sum((t[i]-[0.85,0.25,0.55][i])**2 for i in range(3))**0.5<0.03 else 1)'"
check "e-stop latches the cell, and a stop that worked is not a failure" \
  "$CLI estop 2>/dev/null | j 'j=json.load(sys.stdin);sys.exit(0 if j[\"latched\"] and j[\"state\"]==\"held\" and not j[\"running\"] and not j.get(\"last_error\") else 1)'"
check "jog drives one axis through the OTG, in that axis's own unit" \
  "$CLI jog rail-x 700 2>/dev/null | j 'p=json.load(sys.stdin)[\"pos\"];sys.exit(0 if abs(p[0]-700)<40 else 1)'"
check "grasping thin air is refused, with the distance it measured" \
  "out=\$($CLI grasp 2>&1); echo \"\$out\" | grep -q 'grasp reach'"
check "the log ring is readable without holding a stream open" \
  "$CLI logs 2>/dev/null | grep -q 'cell built'"
check "swapping the scenario on a running platform changes the model" \
  "$CLI use-scene cluttered-line.scene.json 2>/dev/null | j 'm=json.load(sys.stdin);sys.exit(0 if any(l[\"name\"]==\"crate\" for l in m[\"links\"]) else 1)'"
check "a fork is yours: the shipped document is untouched underneath it" \
  "$CLI fork scenes clean-line.scene.json smoke-fork.scene.json --session smoke >/dev/null 2>&1 && \
   $CLI scenes --session smoke 2>/dev/null | grep -q smoke-fork"
check "a refusal says what the solver knew, not just that it failed" \
  "out=\$($CLI movel 9 9 9 2>&1); echo \"\$out\" | grep -qE 'mm short|singular'"
check "naming a robot that is not there fails instead of answering about another" \
  "! $CLI --cell ghost nodes >/dev/null 2>&1"
check "a second robot starts from the catalogue with its own chain" \
  "$CLI add-cell fixed --robot ur10e-fixed.cell.json 2>/dev/null | j 'c=json.load(sys.stdin);n={x[\"id\"]:x[\"nodes\"] for x in c};sys.exit(0 if n.get(\"main\")==7 and n.get(\"fixed\")==6 else 1)'"
check "one document out of a collection reads the same way whatever the collection" \
  "$CLI capability vision 2>/dev/null | j 'sys.exit(0 if json.load(sys.stdin)[\"id\"]==\"vision\" else 1)' && \
   $CLI scene conveyor-line.scene.json 2>/dev/null | j 'sys.exit(0 if json.load(sys.stdin)[\"base\"] else 1)'"
check "every document kind the platform declares is reachable" \
  "$CLI robots 2>/dev/null | grep -q cell.json && $CLI apps 2>/dev/null | grep -q app.json && $CLI scenes 2>/dev/null | grep -q scene.json"
check "a command verb prints the view its own row names, not a generic one" \
  "$CLI version control robonode.smooth 2>/dev/null | j 'c=json.load(sys.stdin);sys.exit(0 if c.get(\"id\")==\"control\" and c.get(\"version\")==\"robonode.smooth\" else 1)'"
check "verbs are published (an editor authors what the runner runs)" \
  "$CLI verbs 2>/dev/null | grep -q intercept"
check "the 1 kHz loop kept up: overruns and jitter inside the settings' budget" \
  "$CLI run --quality 2>/dev/null | j 'q=json.load(sys.stdin);sys.exit(0 if q.get(\"cycles\",0) > 100 and q.get(\"within_budget\") else 1)'"
check "compare reports outcome AND how well each version ran — each from its OWN trial" \
  "$CLI compare control robonode.direct robonode.smooth 2>/dev/null | j 'r=json.load(sys.stdin);res=r[\"results\"];q=[x.get(\"quality\",{}) for x in res];runs=[x.get(\"run\") for x in q];sys.exit(0 if len(res)==2 and r[\"trial\"] and q[0].get(\"worst_axis\",{}).get(\"id\") and all(runs) and runs[0]!=runs[1] else 1)'"
check "watch --json --quiet pipes one machine line per change" \
  "$CLI watch --for 1 --json --quiet state 2>/dev/null | j 'sys.exit(0 if all(\"state\" in json.loads(l) for l in sys.stdin if l.strip()) else 1)'"

# --server is the mode an agent debugging a LIVE cell needs: without it
# `robonode telemetry` answers about a different robot than the one on screen.
# It had no test anywhere, which for the mode that exists to avoid answering
# about the wrong robot is the wrong thing to take on trust.
SERVER="${BUILD_DIR:-build}/apps/cell_server/cell_server"
if [ -x "$SERVER" ]; then
  ROBONODE_PORT=8231 "$SERVER" >/dev/null 2>&1 &
  server_pid=$!
  for _ in $(seq 1 60); do curl -sf http://localhost:8231/ >/dev/null 2>&1 && break; sleep 1; done
  # The whole journey an agent runs against a LIVE platform: use a shipped app,
  # make one of your own, run it, start a second robot, judge two algorithms.
  # Four of these verbs used to be refused with "local-only" against --server,
  # for no reason the server agreed with.
  check "the whole journey works against a running server, not just in process" \
    "$CLI --server http://localhost:8231 --session smoke fork apps pick-demo.app.json mine.app.json >/dev/null 2>&1 && \
     $CLI --server http://localhost:8231 --session smoke apps 2>/dev/null | grep -q mine.app.json && \
     $CLI --server http://localhost:8231 --session smoke app mine.app.json 2>/dev/null | grep -q tcp && \
     $CLI --server http://localhost:8231 add-cell second --robot ur10e-fixed.cell.json >/dev/null 2>&1 && \
     $CLI --server http://localhost:8231 --cell second nodes 2>/dev/null | grep -q j6 && \
     $CLI --server http://localhost:8231 drop-cell second >/dev/null 2>&1 && \
     $CLI --server http://localhost:8231 compare control robonode.direct robonode.smooth 2>/dev/null | \
       j 'r=json.load(sys.stdin);sys.exit(0 if len(r[\"results\"])==2 else 1)'"
  check "--server drives the RUNNING cell, not a private copy of it" \
    "$CLI --server http://localhost:8231 nodes 2>/dev/null | j 'sys.exit(0 if len(json.load(sys.stdin)[\"nodes\"])==7 else 1)' && \
     $CLI --server http://localhost:8231 movel 0.85 0.25 0.55 >/dev/null 2>&1 && \
     curl -sf http://localhost:8231/telemetry | j 'j=json.load(sys.stdin);t=j[\"tcp\"];sys.exit(0 if sum((t[i]-[0.85,0.25,0.55][i])**2 for i in range(3))**0.5<0.05 else 1)'"
  kill "$server_pid" 2>/dev/null
  wait "$server_pid" 2>/dev/null
fi

rm -rf apps/cell_server/modules
[ "$fail" -eq 0 ] && echo "smoke: all subsystems OK" || echo "smoke: FAILURES above"
exit "$fail"
