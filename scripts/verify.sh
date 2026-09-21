#!/usr/bin/env bash
# Definition of Done — one command an agent runs before closing any issue.
# Exit 0 means: design invariants hold, everything builds, every C++ suite
# (unit + integration) is green. This is what makes "done" executable instead
# of a judgement call — no slice merges without it.
#
#   scripts/verify.sh          # design + build + ctest (incl. gateway integration)
#   scripts/verify.sh --e2e    # also the browser e2e (needs docker + node)
#
# Reuses ./build if configured (BUILD_DIR overrides). Run from repo root.
set -uo pipefail
cd "$(dirname "$0")/.."

BUILD="${BUILD_DIR:-build}"
step() { printf '\n=== %s ===\n' "$1"; }

# Two of these suites measure REAL TIME — a 1 kHz loop's jitter, and an arm
# reaching for a part on a moving belt. Another cell_server on the same machine
# is a second physics engine competing for the same cores, and the number they
# then report is about the machine, not the code. ctest itself is serial (no -j
# anywhere), so this is the only way that load gets in.
if pgrep -f 'cell_server' >/dev/null 2>&1; then
  echo "FAIL: a cell_server is already running. Two of these suites measure real time,"
  echo "      and a second physics engine on the same cores makes their numbers meaningless."
  echo "      Stop it first:  pkill -f cell_server"
  exit 1
fi

step "design invariants"
bash scripts/check-design.sh || { echo "FAIL: design drift"; exit 1; }

step "configure + build"
if [ ! -f "$BUILD/CMakeCache.txt" ]; then
  cmake -B "$BUILD" -DCMAKE_BUILD_TYPE=Release || { echo "FAIL: configure"; exit 1; }
fi
cmake --build "$BUILD" -j || { echo "FAIL: build"; exit 1; }

step "tests (ctest — unit + integration)"
ctest --test-dir "$BUILD" --output-on-failure || { echo "FAIL: ctest"; exit 1; }

step "deploy smoke (the built binary drives every subsystem)"
BUILD_DIR="$BUILD" bash scripts/smoke.sh || { echo "FAIL: smoke"; exit 1; }

if [ "${1:-}" = "--e2e" ]; then
  step "browser e2e (Playwright)"
  docker compose up --build -d || { echo "FAIL: docker up"; exit 1; }
  for i in $(seq 1 60); do curl -sf http://localhost:8080/ >/dev/null && break; sleep 5; done
  npm install >/dev/null 2>&1 && npx playwright install --with-deps chromium >/dev/null 2>&1
  E2E_BASE_URL=http://localhost:8080 npm run test:e2e; e2e_rc=$?
  docker compose down
  [ "$e2e_rc" -eq 0 ] || { echo "FAIL: e2e"; exit 1; }
fi

step "scenario matrix"
green=$(grep -c '✅ live\|| \*\*✅\*\*' docs/TEST-STRATEGY.md 2>/dev/null || echo "?")
echo "journeys marked green in docs/TEST-STRATEGY.md: ~$green (update the matrix if you added one)"

printf '\n✅ VERIFY PASSED — design clean, build green, all suites pass.\n'
