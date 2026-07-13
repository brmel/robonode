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
check core 'robonode/(motion|recorder|adapter)|trajlib/|ruckig/|mcap/|ur_client_library/' \
  "core must depend on nothing"

# motion: no recorder, no adapters, no vendor comms, no mcap
check motion 'robonode/(recorder|adapter)|mcap/|ur_client_library/' \
  "motion may include core (+ trajlib/ruckig internally) only"

# recorder: core + mcap only
check recorder 'robonode/(motion|adapter)|trajlib/|ruckig/|ur_client_library/' \
  "recorder may include core + mcap only"

# adapters: no recorder in headers/impl of the adapter itself; demo apps may
# use recorder, so restrict the rule to include/ (the reusable surface)
check adapters/ur/include 'robonode/recorder|trajlib/|ruckig/|mcap/' \
  "adapter headers may include core + motion + vendor only"

# celld: vendor-blind coordination plane — core + motion + json only
check celld 'robonode/(recorder|adapter)|trajlib/|ruckig/|mcap/|ur_client_library/' \
  "celld may include core + motion + json only"

# trajlib and ruckig are motion-private
hits=$(grep -rnE '^#include ["<](trajlib|ruckig)/' apps tests adapters core recorder celld 2>/dev/null || true)
if [ -n "$hits" ]; then
  violation "trajlib/ruckig are private to motion/"
  echo "$hits" | sed 's/^/    /'
fi

if [ "$fail" -eq 0 ]; then
  echo "module boundaries: OK"
fi
exit "$fail"
