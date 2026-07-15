#!/usr/bin/env bash
# Design-invariant lint — the teeth that keep an autonomous agent from
# drifting. These are the ADR decisions expressed as greppable rules; CI and
# scripts/verify.sh run this. A violation fails the build, so the design
# cannot rot silently. New invariants get added here as issues land (see the
# "pending gates" list at the bottom) — the guard grows with the system, the
# same way the scenario matrix does.
#
# Run from repo root. Exit nonzero listing every violation.
set -uo pipefail
cd "$(dirname "$0")/.."

fail=0
bad() { echo "DESIGN VIOLATION: $1"; fail=1; }

# 1) Module boundaries (the structural seams). Delegate to the boundary lint.
if ! bash scripts/check-boundaries.sh >/tmp/robonode-boundaries.log 2>&1; then
  bad "module boundaries broken:"; sed 's/^/    /' /tmp/robonode-boundaries.log
fi

# 2) The seams we own must exist — deleting one is a design regression, not a
#    refactor. Every swappable engine (Part 2 of ARCHITECTURE) hides behind one.
SEAMS=(
  motion/include/robonode/motion/axis_adapter.hpp
  motion/include/robonode/motion/kinematics.hpp
  motion/include/robonode/motion/planner.hpp
  motion/include/robonode/motion/setpoint_source.hpp
  motion/include/robonode/motion/driver_registry.hpp
  gateway/include/robonode/gateway/cell_gateway.hpp
  gateway/include/robonode/platform.hpp
  celld/include/robonode/celld/cell.hpp
)
for s in "${SEAMS[@]}"; do
  [ -f "$s" ] || bad "seam missing: $s (a seam is a contract, not disposable)"
done

# 3) RT-loop purity + anti-hardcoding (ADR-5, #32): the pure-logic modules
#    carry no Python, and no baked-in config — model/config file paths, IPs, or
#    URLs. Everything comes THROUGH the seam as descriptor data. core/motion/
#    celld are the audited pure modules (apps/services may hold their defaults).
hits=$(grep -rnE --include='*.hpp' --include='*.cpp' \
  '<Python\.h>|^import |\.(xml|json)"|https?://|([0-9]{1,3}\.){3}[0-9]{1,3}' \
  core motion celld 2>/dev/null | grep -v _deps || true)
[ -n "$hits" ] && { bad "pure modules must stay Python-free + hardcoding-free (paths/IPs/URLs are descriptor data — ADR-5/#32):"; echo "$hits" | sed 's/^/    /'; }

# 4) One error model at the seams (ADR-7): no exceptions thrown across a seam.
#    Guard the obvious regression — throwing out of a public seam header.
hits=$(grep -rnE '\bthrow\b' motion/include/robonode/motion/axis_adapter.hpp \
  motion/include/robonode/motion/kinematics.hpp \
  motion/include/robonode/motion/planner.hpp 2>/dev/null || true)
[ -n "$hits" ] && { bad "seam headers must not throw across the boundary (ADR-7 → std::expected):"; echo "$hits" | sed 's/^/    /'; }

if [ "$fail" -eq 0 ]; then
  echo "design invariants: OK"
fi

# Pending gates — enabled (moved above) as their issue closes, so the guard
# hardens over time instead of failing prematurely. Still open:
#   #34 → forbid rtb-service calls from motion/celld (RT path Python-free at call level)
#   #35 → forbid std::mutex / new / malloc on the executive step path
#   #36 → forbid `throw` across any seam; require std::expected returns
# Done: #28/#32 (descriptor-driven + anti-hardcoding, enabled above) · #42/#44
# (spdlog logging seam — the no-printf rule stays OFF: apps print CLI output by
# design; the seam is for events, not stdout).
echo "pending gates (enable as #34/#35/#36 close): see script footer"
exit "$fail"
