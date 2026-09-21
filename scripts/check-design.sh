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
  motion/include/robonode/motion/controller.hpp
  motion/include/robonode/motion/setpoint_source.hpp
  motion/include/robonode/motion/driver_registry.hpp
  sandbox/include/robonode/sandbox/program.hpp
  gateway/include/robonode/gateway/cell_gateway.hpp
  gateway/include/robonode/platform.hpp
  celld/include/robonode/celld/cell.hpp
)
for s in "${SEAMS[@]}"; do
  [ -f "$s" ] || bad "seam missing: $s (a seam is a contract, not disposable)"
done

# 3) RT-loop purity + anti-hardcoding (ADR-5): the pure-logic modules
#    carry no Python, and no baked-in config — model/config file paths, IPs, or
#    URLs. Everything comes THROUGH the seam as descriptor data. core/motion/
#    celld are the audited pure modules (apps/services may hold their defaults).
# A config path has a NAME before the extension (world.xml, cells/x.json); a
# bare ".json" is a file-extension check (persistence code), not hardcoding.
hits=$(grep -rnE --include='*.hpp' --include='*.cpp' \
  '<Python\.h>|^import |[[:alnum:]_/]\.(xml|json)"|https?://|([0-9]{1,3}\.){3}[0-9]{1,3}' \
  core motion celld 2>/dev/null | grep -v _deps || true)
[ -n "$hits" ] && { bad "pure modules must stay Python-free + hardcoding-free (paths/IPs/URLs are descriptor data — ADR-5):"; echo "$hits" | sed 's/^/    /'; }

# 4) One error model at the seams (ADR-7): no exceptions thrown across a seam.
#    Guard the obvious regression — throwing out of a public seam header.
hits=$(grep -rnE '\bthrow\b' motion/include/robonode/motion/axis_adapter.hpp \
  motion/include/robonode/motion/kinematics.hpp \
  motion/include/robonode/motion/controller.hpp \
  motion/include/robonode/motion/planner.hpp 2>/dev/null || true)
[ -n "$hits" ] && { bad "seam headers must not throw across the boundary (ADR-7 → std::expected):"; echo "$hits" | sed 's/^/    /'; }

# 5) The node model (#node-model): the gateway addresses robots and axes by ID.
#    Positional indexing into the flat node list is how the 7-node assumption
#    got baked in the first time.
hits=$(grep -rnE 'nodes\(\)\[[0-9]|nodes\[[0-9]' gateway/include apps 2>/dev/null || true)
[ -n "$hits" ] && { bad "gateway must address nodes by id, never by position:"; echo "$hits" | sed 's/^/    /'; }

# 6) One capability contract (ADR-8): capability ids are data served by
#    /capabilities. A surface that hardcodes the list drifts from the platform.
hits=$(grep -rnE '"(vision|planner|control)"' apps/cell_server/web/*.js 2>/dev/null | grep -vE ':[0-9]+: *(//|\*)' || true)
[ -n "$hits" ] && { bad "web surfaces must read capabilities from /capabilities, not hardcode ids:"; echo "$hits" | sed 's/^/    /'; }

# 6z) No half-features on the web surface: a control in the markup that no code
#     reaches is a button that does nothing, and it looks exactly like one that
#     works. Styling-only hooks count as reached (the CSS names them).
for id in $(grep -ohE 'id="[a-zA-Z_]+"' apps/cell_server/web/index.html | sed 's/id="//;s/"//' | sort -u); do
  grep -rqlE "'$id'|\"$id\"|#$id\b" apps/cell_server/web/*.js apps/cell_server/web/*.css 2>/dev/null \
    || bad "#$id is in the markup and nothing reaches it — wire it or delete it"
done

# 6y) The same rule for the CLI: a subcommand CLI11 knows about but nothing
#     dispatches parses your arguments, prints nothing and exits 0. The verb
#     tables (reads, kinds, commands) are generated, so what this catches is a
#     hand-written declaration whose dispatch line was never added.
declared=$(grep -oE 'add_subcommand\("[a-z-]+"' apps/robonode_cli/main.cpp | sed -E 's/.*"([a-z-]+)"/\1/' | sort -u)
for verb in $declared; do
  grep -qE "c_[a-z]+ = app.add_subcommand\(\"$verb\"" apps/robonode_cli/main.cpp || continue
  handle=$(grep -oE "c_[a-z]+ = app.add_subcommand\(\"$verb\"" apps/robonode_cli/main.cpp | sed -E 's/ =.*//')
  grep -q "\*$handle" apps/robonode_cli/main.cpp \
    || bad "the CLI declares '$verb' and never dispatches it — wire it or delete it"
done

# 6a) The live views are a table in the facade (Platform::kViews). A surface that
#     declares one by name is a surface that can be missing one the others have —
#     which is exactly the parity ADR-8 exists to guarantee. Routes and CLI verbs
#     are GENERATED from the table; naming a view when declaring one is the drift.
views=$(grep -oE '\{"[a-z-]+", &CellGateway::' gateway/include/robonode/platform.hpp \
  | sed -E 's/\{"([a-z-]+)".*/\1/' || true)
for v in $views; do
  hits=$(grep -rn "svr.Get(\"/$v\"\|add_subcommand(\"$v\"" apps/ 2>/dev/null || true)
  [ -n "$hits" ] && { bad "a live view must come from Platform::kViews, not be declared per surface:"; echo "$hits" | sed 's/^/    /'; }
done

# 4b) ADR-12 extended to evidence: a run RECORD carries the identity of the run
#     that made it. Without it "the last run" is whatever ran most recently, and
#     a comparison whose trial never moved reports the previous version's
#     numbers under this one's name — which reads exactly like evidence.
grep -q 'j\["run"\] = quality_run_' gateway/include/robonode/gateway/cell_gateway.hpp \
  || bad "the last-run record must carry which run it is from (ADR-12)"
grep -q 'value("run"' gateway/include/robonode/platform.hpp \
  || bad "compare must check that a quality record belongs to the trial it reports"

# 5c) ADR-17: a cell-scoped view answers about THAT cell. The log is the one
#     process-wide store left, so reading it without naming the cell inside a
#     gateway is how `/logs?cell=x` starts answering about every robot again.
hits=$(grep -n 'recent_json()' gateway/include/robonode/gateway/*.hpp 2>/dev/null || true)
[ -n "$hits" ] && { bad "a cell-scoped view must name its cell when reading the shared log (ADR-17):"; echo "$hits" | sed 's/^/    /'; }

# 3b) The scenario matrix is a ledger, and a ledger with two entries under one id
#     is not one. Two rows sat on ids already in use, describing features that
#     had SHIPPED while marked planned — the row said the work was pending and
#     the code said it was done, and nothing was checking.
dupes=$(awk -F'|' '/^\| J/ {gsub(/ /,"",$2); print $2}' docs/TEST-STRATEGY.md | sort | uniq -d)
[ -n "$dupes" ] && { bad "the scenario matrix has more than one row under one journey id:"; echo "$dupes" | sed 's/^/    /'; }

# 3c) A matrix row claims a layer; the Evidence column says WHERE. Two rules, so
#     the claim is checkable instead of taken on trust:
#       - what a row names has to exist,
#       - and no row may ship without naming it. That rule started at J130 (the
#         frontier the first audit reached) and now covers all of them: the
#         backlog is empty, so the exemption has nothing left to excuse.
python3 - <<'PYEOF' || exit 1
import re, subprocess, sys
rows = [l for l in open('docs/TEST-STRATEGY.md') if l.startswith('| J')]
bad = []
for line in rows:
    cells = [c.strip() for c in line.strip().strip('|').split('|')]
    jid, evidence = cells[0], cells[-1]
    number = int(jid[1:])
    if evidence == '—':
        bad.append(f"{jid} ships without naming its evidence")
        continue
    for token in re.findall(r'`([^`]+)`', evidence) + re.findall(r'"([^"]+)"', evidence):
        token = token.strip()
        if not token or token.startswith('nodes['):
            continue
        if token.endswith(('.ts', '.sh', '.cpp', '.hpp')):
            found = subprocess.run(['sh', '-c',
                                    f"find tests e2e scripts .github -name '{token}' | grep -q ."],
                                   capture_output=True).returncode == 0
        else:
            found = subprocess.run(['grep', '-rqlF', '--', token, 'tests', 'e2e', 'scripts', '.github'],
                                   capture_output=True).returncode == 0
        if not found:
            bad.append(f"{jid} names '{token}', which is nowhere in tests/ e2e/ scripts/ .github/")
for b in bad:
    print(b)
sys.exit(1 if bad else 0)
PYEOF

# 4c) ADR-12: one definition of "done". A surface that re-derives it from the
#      shape of a telemetry frame is a second answer to the only question a
#      caller is waiting on — they drifted once already.
hits=$(grep -rn 'applied_id.*accepted_id\|accepted_id.*applied_id' --include='*.hpp' --include='*.cpp' --include='*.js' \
  gateway apps 2>/dev/null | grep -v 'progress.hpp' | grep -vE ':[0-9]+: *(//|\*)' || true)
[ -n "$hits" ] && { bad "waiting on a cell goes through progress_of(), not a second predicate (ADR-12):"; echo "$hits" | sed 's/^/    /'; }

# 5a) ADR-17/ADR-14: a setting comes from the settings file, resolved once and
#     injected. A default argument that reads `Settings{}` is a second source of
#     truth, and the caller that forgot to pass one looks deliberate.
hits=$(grep -rn 'Settings{}\.' --include='*.hpp' --include='*.cpp' \
  core motion celld gateway engines apps vision sandbox recorder adapters 2>/dev/null || true)
[ -n "$hits" ] && { bad "a tunable must be injected from Settings, not defaulted in a signature (ADR-14/17):"; echo "$hits" | sed 's/^/    /'; }

# 5b) One owner for the live world. Two threads stepping the same physics
#     data is a segfault, not a race you get away with — so a raw Scene is
#     reachable through SceneView and nowhere else. Opening one (the engine
#     choice) stays with the cell; touching one does not.
hits=$(grep -rnE 'Scene\*|->advance\(|->contacts\(\)|->site_poses\(\)' \
  gateway/include apps examples 2>/dev/null | grep -v 'scene_view.hpp' || true)
[ -n "$hits" ] && { bad "the live world is SceneView's: reach it through with(), not a raw Scene:"; echo "$hits" | sed 's/^/    /'; }

# 6a2) A CLI command verb names a command the gateway actually routes. The verbs
#      are a table now, so this is checkable: a row whose wire name nothing
#      handles is a verb that parses, prints an ack and does nothing.
wire=$(grep -rhoE '\.on\("[a-z_]+"' gateway/include | sed -E 's/.*"([a-z_]+)"/\1/'; \
       grep -hoE '"[a-z_]+"' <(grep -A 1 'for (const char\* verb :' gateway/include/robonode/gateway/cell_gateway.hpp) \
       | tr -d '"')
for cmd in $(grep -oE 'command_verb\("[a-z-]+", "[a-z_]+"' apps/robonode_cli/main.cpp \
             | sed -E 's/.*, "([a-z_]+)"/\1/'); do
  echo "$wire" | grep -qx "$cmd" || { bad "CLI command verb '$cmd' names a command the gateway does not route"; }
done

# 6b) The same rule for every OTHER list the platform publishes: driver families
#     and cell states are declared by the cell and the supervisor, so a surface
#     that spells them out will show the wrong ones the day a descriptor changes.
hits=$(grep -rnE 'data-fam="[a-z]|"(physics|sim)"' apps/cell_server/web/*.js apps/cell_server/web/*.html 2>/dev/null \
  | grep -vE ':[0-9]+: *(//|\*|<!--)' || true)
[ -n "$hits" ] && { bad "web surfaces must build the family selector from GET /nodes, not from markup:"; echo "$hits" | sed 's/^/    /'; }

# 6c) A class assertion must name the whole class list. `toHaveClass(/on/)`
#     also matches "rreason", so the test passes whether the class is there or
#     not — an assertion that cannot fail is worse than no assertion.
hits=$(grep -rn 'toHaveClass(/' e2e/*.ts 2>/dev/null | grep -vE ':[0-9]+: *(//|\*)' || true)
[ -n "$hits" ] && { bad "browser assertions must name the exact class list, not a regex that can match another class:"; echo "$hits" | sed 's/^/    /'; }

# 6d) The agent manual lists the CLI's verbs. An agent reads that table as the
#     surface it has, so a verb missing from it is a capability nobody uses —
#     and a verb listed that does not exist is a plan that fails at run time.
if [ -x "${BUILD_DIR:-build}/apps/robonode_cli/robonode_cli" ]; then
  missing=""
  for verb in $("${BUILD_DIR:-build}/apps/robonode_cli/robonode_cli" --help 2>/dev/null \
                | grep -E '^  [a-z][a-z-]+' | awk '{print $1}'); do
    grep -qE -- "\`$verb([\`, ]|\$)" AGENTS.md || missing="$missing $verb"
  done
  [ -n "$missing" ] && { bad "AGENTS.md does not list every CLI verb:$missing"; }
fi

# 6e) As far as it has landed: the seam-facing motion headers return a
#     reason instead of throwing one. An exception here unwinds through the
#     executive on the thread that owns the physics, and the caller who passed a
#     ragged waypoint list deserves a sentence, not a terminate(). sync_executive
#     is not on this list yet — its checks are construction-time invariants
#     between vectors one caller builds together (see the seam rules).
hits=$(grep -rn 'throw ' motion/include/robonode/motion/sync_blend.hpp \
       motion/include/robonode/motion/cartesian.hpp motion/include/robonode/motion/planner.hpp \
       2>/dev/null | grep -vE ':[0-9]+: *(//|\*)' || true)
[ -n "$hits" ] && { bad "the motion seams must return a reason, not throw one:"; echo "$hits" | sed 's/^/    /'; }

# 7) Branding: colour lives in the token layer, so the 3D view and the chrome
#    cannot drift and a theme switch repaints both.
hits=$(grep -rnE '0x[0-9a-fA-F]{6}|#[0-9a-fA-F]{6}' apps/cell_server/web/*.js 2>/dev/null | grep -v three | grep -v 'theme.js' | grep -vE ':[0-9]+: *(//|\*)' || true)
[ -n "$hits" ] && { bad "web colour literals must come from tokens.css via theme.js:"; echo "$hits" | sed 's/^/    /'; }

# 8) No silent catch in the web app: a failure the user cannot see is a bug we
#    will be told about by the user instead of by the UI.
hits=$(grep -rnE 'catch \{ *\}|catch \{ */\*' apps/cell_server/web/*.js 2>/dev/null || true)
[ -n "$hits" ] && { bad "every catch must report through the error channel:"; echo "$hits" | sed 's/^/    /'; }

# 9) The wire contract is written down and executable: every surface binds to
#    it (ADR-8), so it may not live only in the code that happens to emit it.
for c in telemetry nodes capabilities command-ack model; do
  [ -f "contracts/$c.schema.json" ] || bad "contracts/$c.schema.json missing (the wire contract must be declared)"
done

# 10) Tunables are data (config/robonode.settings.json): a heuristic you cannot
#    change without a rebuild is one nobody will improve.
[ -f config/robonode.settings.json ] || bad "config/robonode.settings.json missing (tuning must be data)"


# Open-source hygiene: the licence, the notice for the engines we reuse, and the
# contributor entry point. A repo that quietly loses one of these is a repo
# nobody can safely contribute to.
for f in LICENSE NOTICE CONTRIBUTING.md CODE_OF_CONDUCT.md SECURITY.md; do
  [ -f "$f" ] || bad "$f is missing — the project is licensed and open to contributions"
done

[ "$fail" -eq 0 ] && echo "design invariants: OK"

# Pending gates — enabled (moved above) as their issue closes, so the guard
# hardens over time instead of failing prematurely. Still open:
#   - forbid rtb-service calls from motion/celld (RT path Python-free at call level)
#   - forbid std::mutex / new / malloc on the executive step path
#   - the rest of it: sync_executive's construction checks, and the 21
#         remaining `Status f(…, T& out)` signatures (rule in docs/ARCHITECTURE.md)
# Done: descriptor-driven + anti-hardcoding (enabled above)
# (spdlog logging seam — the no-printf rule stays OFF: apps print CLI output by
# design; the seam is for events, not stdout).
echo "pending gates: see script footer"
exit "$fail"
