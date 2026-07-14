#!/usr/bin/env bash
# Module-boundary lint: layering is structural, not aspirational.
# Rules (SPEC seams / MIL discipline):
#   core      includes only robonode/core (and stdlib) — the bottom
#   motion    may include core; trajlib is its private implementation detail
#   recorder  may include core (+ mcap); never motion/adapters
#   adapters  may include core+motion (+ vendor libs); never recorder
#   trajlib   may be included ONLY from motion/ (apps/tests/adapters go
#             through motion's public surface)
# Run from repo root. Exits nonzero listing every violation.
set -uo pipefail
cd "$(dirname "$0")/.."

fail=0
violation() { echo "BOUNDARY VIOLATION: $1"; fail=1; }

check() { # check <dir> <forbidden-include-regex> <label>
  local hits
  hits=$(grep -rnE "^#include [\"<]($2)" "$1" 2>/dev/null || true)
  if [ -n "$hits" ]; then
    violation "$3"
    echo "$hits" | sed 's/^/    /'
  fi
}

# core: nothing but core
check core 'robonode/(motion|recorder|adapter|sim_mujoco)|trajlib/|ruckig/|mcap/|mujoco/|ur_client_library/' \
  "core must depend on nothing"

# motion: no recorder, no adapters, no sim, no vendor comms, no mcap
check motion 'robonode/(recorder|adapter|sim_mujoco)|mcap/|mujoco/|ur_client_library/' \
  "motion may include core (+ trajlib/ruckig internally) only"

# recorder: core + mcap only
check recorder 'robonode/(motion|adapter|sim_mujoco)|trajlib/|ruckig/|mujoco/|ur_client_library/' \
  "recorder may include core + mcap only"

# adapters: no recorder in headers/impl of the adapter itself; demo apps may
# use recorder, so restrict the rule to include/ (the reusable surface)
check adapters/ur/include 'robonode/recorder|trajlib/|ruckig/|mcap/' \
  "adapter headers may include core + motion + vendor only"

# celld: vendor-blind coordination plane — core + motion + json only
check celld 'robonode/(recorder|adapter|sim_mujoco|gateway)|trajlib/|ruckig/|mcap/|mujoco/|httplib|ur_client_library/' \
  "celld may include core + motion + json only"

# gateway: the web-facing bridge — celld + sim_mujoco + httplib (its own dep).
# httplib is gateway/app-private.
hits=$(grep -rnE '^#include [<"]httplib' core motion recorder celld sim-mujoco adapters trajectory-lab 2>/dev/null || true)
if [ -n "$hits" ]; then
  violation "httplib is private to gateway/ + its server app"
  echo "$hits" | sed 's/^/    /'
fi

# trajlib and ruckig are motion-private
hits=$(grep -rnE '^#include ["<](trajlib|ruckig)/' apps tests adapters core recorder celld sim-mujoco 2>/dev/null || true)
if [ -n "$hits" ]; then
  violation "trajlib/ruckig are private to motion/"
  echo "$hits" | sed 's/^/    /'
fi

# mujoco is sim-mujoco-private: no direct include anywhere else
hits=$(grep -rnE '^#include [<"]mujoco/' apps tests adapters core motion recorder celld 2>/dev/null || true)
if [ -n "$hits" ]; then
  violation "mujoco is private to sim-mujoco/"
  echo "$hits" | sed 's/^/    /'
fi

# rtb bridge headers are bridge/app/test-private — the core modules never
# depend on the kinematics bridge (it depends on them).
hits=$(grep -rnE '^#include [<"]robonode/rtb/' core motion recorder celld sim-mujoco adapters gateway 2>/dev/null || true)
if [ -n "$hits" ]; then
  violation "robonode/rtb is private to bridges/rtb + apps/tests"
  echo "$hits" | sed 's/^/    /'
fi

if [ "$fail" -eq 0 ]; then
  echo "module boundaries: OK"
fi
exit "$fail"
