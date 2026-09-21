# Changelog

All notable changes to RoboNode. Format: [Keep a Changelog]; the platform is a
modular monolith (RT core) + compute sidecars, so entries are grouped by surface.

## [0.9.0] — 2026-07-31 — an application you can make, and an algorithm you can judge

Two things a platform for testing robotics algorithms has to do that this could
not: let you *build* the application, and tell you whether your algorithm is any
good. Both now go through the facade, so the browser and the CLI cannot disagree
about what they mean.

### Changed
- **The driver-family selector is built from the cell, not from markup.** The
  node tree now publishes `families`, so a descriptor that declares a third one
  shows a third button with no HTML edit. `check-design.sh` fails the build if a
  web surface spells a family name out again — the same rule capability ids
  already had. The cell's state set is pinned in the telemetry contract and
  checked against the live server.

### Fixed
- **A comparison stopped printing one algorithm's evidence under another's
  name.** Each version's row carried `quality` read from "the last run" — so a
  version whose trial failed before anything moved was reported with the
  PREVIOUS version's cycles, jitter and worst-axis error. Two rows came back
  bit-identical to sixteen digits, which is what made it visible; nothing about
  the output looked wrong, and it is exactly the plausible-looking wrong answer
  a person judging an algorithm would believe. A run record now carries which
  run it is from (ADR-12 applied to evidence, not just to commands), and a
  comparison attaches quality only when that counter advanced during its own
  trial. `check-design.sh` holds both halves; the smoke check now fails if the
  two versions' records share an identity.

  Enforcing that immediately found what the misattribution had been hiding:
  the second version's trial was moving *nothing*. Each version started where
  the previous one finished, which for `pick-demo` is already at the goal — so
  only the first algorithm was ever exercised, and the second was reported with
  the first one's numbers. Every version now starts from the same cell. The two
  control versions come back with different following errors, which is what a
  comparison was supposed to produce all along.
- **The sanitizers now cover the code that actually runs.** Both jobs built with
  MuJoCo *off* — which sounded principled ("sanitizing vendored libraries is not
  our job") and meant that between them they excluded every suite with a worker,
  an idle ticker and a reader in it: the configuration this platform has crashed
  in four times. What was sanitized was the part that could not have been wrong.
  Turning physics on took two warning waivers on MuJoCo's own sanitizer shim
  (`asm` under strict C11, an `always_inline` position GCC would reject) —
  scoped to that vendored target, nothing of ours relaxed. Result: ASan and
  UBSan clean across all five physics-linked suites, ThreadSanitizer clean
  across all five, and CI runs both that way.
- **One owner for the live world.** `CellGateway` was 902 lines and held the
  scene, the world pool, the snapshot and the mutex over it, plus the eight
  methods that used them — and "everything that touches physics does so under
  the cell lock" was a convention every new caller had to be told. It is
  a type now: `SceneView` owns the scene and the snapshot, `with` is the only
  way in, and a reader gets frames rather than the world — so the gap between a
  step and the sample of it is not observable. `check-design.sh` fails the build
  on a raw `Scene*` anywhere else, verified by breaking it. ThreadSanitizer is
  clean across the threaded suites after the move.
- **Every one of the 143 matrix rows names what proves it, and the rule is now
  universal.** The exemption that let older rows ship blank is gone, because the
  backlog is gone: a new row arrives with its evidence or it does not arrive.
  What the column cost to fill is the argument for it — one row was demoted for
  resting on an example that merely compiled, one had no test at all, and
  `--server` turned out never to have been exercised.
- **`--server` had no test — the one mode that exists so an agent does not debug
  the wrong robot.** `robonode --server` drives a *running* cell instead of
  booting a private copy; without it `robonode telemetry` answers about a
  different robot than the one on screen, which is the reason the mode exists.
  Nothing anywhere exercised it. The smoke run now starts a server, drives it
  over HTTP, and checks the *server's* telemetry moved — the claim, end to end.
- **The UR adapter is asserted, not assumed.** J32 was demoted last pass for
  resting on an example that merely compiled. A build that links UR now has to
  offer `robonode.ur-wrist` as a selectable version — registration is the half
  of the claim that needs no hardware, and it is the half that was never
  checked.
- **81 of 143 matrix rows now name their evidence, and two rows did not survive
  being asked.** Backfilling is done by verifying, not by guessing — every name
  goes through the same gate, which rejected three of my first attempts (a test
  whose real name had a `gateway_` prefix I had dropped, a browser test whose
  title I had shortened, and a row I tried to back with an *example* rather than
  a test). J45 — "waiting is bounded by stall, not duration" — had no test at
  all and now has one that runs a program for half a minute under a five-second
  stall bound. J32 — "vendor hardware is a selectable driver" — has only an
  example that compiles, so it is **demoted to planned** rather than left green.
- **The gate catches a renamed test, confirmed rather than assumed.** Renaming
  the browser test J135 points at fails the build.
- **The matrix says where each claim is proved, and the build checks it.** Last
  pass audited it by reading, which fixed the rot but could not stop it coming
  back. There is an Evidence column now: a row names the test, spec, smoke check
  or design rule that proves it, and `check-design.sh` fails the build when that
  name is not in `tests/`, `e2e/` or `scripts/` — and when a row from J130 on
  ships with nothing named. Everything before J130 is honestly blank and fills
  in as it is verified: a blank cell is a to-do, a wrong one is a lie, and only
  the second is worth failing a build over.

  It caught something one commit after it was written: J143 had shipped green on
  hand-verification with no test behind it at all. It has one now. That is the
  whole argument for the column.
- **The scenario matrix audited against reality, for the first time.** It had
  been growing a row an iteration for 38 iterations with nothing checking that a
  green row meant anything. What the audit found: two rows sitting on ids
  already in use, both marked *planned* while describing features that had
  shipped — the ledger said work was pending and the code said it was done. Six
  more marks were stale in the other direction, including two claiming a CLI
  owed coverage for a theme toggle and a node inspector, which are not CLI
  concepts. And the inventory paragraph said "8 C++ suites, 4 browser journeys"
  against a real 22 and 54 — wrong by a factor of three, which is what happens
  when a count is written in prose instead of computed. That paragraph now
  points at the command that counts.

  `check-design.sh` fails the build on two rows under one journey id. The rest
  cannot be gated mechanically, so it was done by reading — and three rows that
  honestly said the CLI did not cover them are now covered rather than
  explained: a user-authored trajectory plans a real move, a module outlives the
  process that authored it, and the bring-your-own driver is selected and
  drives. No row is marked planned any more, and none of them got there by
  being re-marked.
- **A boolean that selected between two whole messages is two functions.**
  `ik_reason(residual, singular, iters)` ignored `residual` and `iters` entirely
  when `singular` was true — which is what a flag parameter choosing between
  messages always turns out to mean.
- **The integration suite is four suites.** `gateway_integration_tests.cpp` had
  become the largest file in the repository — 1230 lines and 49 tests about
  capabilities, concurrency, multi-cell and the facade all at once, so finding
  the test for a thing meant scrolling past three subjects that were not it.
  It is split by subject with one shared fixture header; all 49 tests still run,
  and the ThreadSanitizer job now names the suite the races actually live in.
- **Reading one document out of a collection is a table.** `run-record`,
  `capability` and `scene` were the same four lines three times over, and
  `sessions` was a hand-written branch for something the read table already
  knew how to do. The fifth would have been the one that read the wrong
  collection.
- **"Done" is defined once (ADR-12).** The facade and the CLI-over-HTTP each
  carried their own copy of the same predicate — applied caught up with
  accepted, not running, bounded by silence rather than by a guess at how long
  the work takes. Two copies of the only question a caller is waiting on is two
  chances to disagree about it. `progress_of` is that answer, both surfaces
  use it, and `check-design.sh` fails the build on a second one.
- **A comparison stopped being one function that did five things.**
  `Platform::compare` resolved the trial, reset the cell, ran it, judged the
  result and persisted the report in 58 lines. It is `trial_for` (what a
  capability is judged on, and what was running before), `judge` (one version,
  one trial, one row) and a `compare` that reads as the three sentences it is —
  all behind the facade rather than on it.
- **A disabled control says why it is disabled.** "Test against" in the
  algorithm editor sat greyed out with no explanation, which reads as broken; it
  now says what unlocks it, and afterwards what it will judge against.
- **"Smoothing follows more gently" was true of a toy and false of the robot.**
  J17 has claimed it unqualified since the control capability shipped, backed by
  a unit test on an ideal axis — a plant that answers the setpoint instantly, so
  smoothing strictly wins. On the cell, under inertia, a smoothing controller
  lags further through the acceleration phase and its **peak** following error is
  worse: 0.293 rad against 0.288, repeatably. The platform's own `compare` has
  been reporting this the whole time. Both results are now measured and named for
  the plant they are about, the row says so, and `docs/CHALLENGES.md` keeps it as
  what it is — a trade-off, not a winner.

  The inventory behind this: three claims in the repo say one algorithm beats
  another. J49 was unfair (fixed last pass), J17 was unqualified (this one), and
  the collision-avoidance claim measures both planners against one oracle from
  one start — fair, unchanged.
- **The moving-target scenario now proves what it claims.** J49's claim was
  false under equal budgets (below) and is true now — and the algorithms were
  never the problem. Two things about the SCENE were, both station config: the
  part ran to the end of the belt and **stopped**, where anything catches it,
  and one reach consumed most of a traverse (1.4 attempts per pass) so no
  tracker could converge. The line recirculates now, and the belt runs slow
  enough for about four attempts per pass. With five attempts each, snapshot
  fails and constant-velocity succeeds, three runs of three. `celld_tests`
  stopped pinning the belt speed as though a tuning value were a contract —
  that pin would have made this fix look like a regression.
- **A saved-documents list is written once.** The algorithm library and the kept
  comparisons rendered the same three parts — a name, a way to reopen, a way to
  throw away — from two copies differing only in which collection they read.
  `renderLibrary` joins `renderRecords` as the second shared list widget.
- **`verify.sh` refuses to run while a `cell_server` is up.** Two suites measure
  real time; a second physics engine on the same cores makes their numbers a
  fact about the machine. The earlier note blamed "parallel ctest load" — wrong,
  and recorded as wrong: ctest is serial and nothing sets `-j`. The load was a
  server left running from browser work.
- **A headline claim did not survive being measured.** J49 read "the naive
  tracker misses, prediction catches it". Hardening the test meant giving both
  algorithms the same attempt budget — and with an equal budget they are not
  reliably separable in that scene. The first reach of any tracker is blind (one
  sighting gives no velocity), the lead is estimated from how long the previous
  reach took and under-predicts while the reach is slow, and the part runs to the
  end of the belt and stops, where anything can catch it. Runs exist where the
  naive tracker wins and prediction does not. The test now states exactly what it
  proves — naive fails within two attempts, prediction succeeds within four — the
  row says the same, and `docs/CHALLENGES.md` records what the scenario would
  need to be a real discriminator. The row had looked proved for months.
- **Four verbs stopped refusing themselves.** `fork`, `compare`, `add-cell` and
  `drop-cell` answered "local-only" whenever the CLI was pointed at a running
  server — while that server served all four, and the browser used them. So the
  one mode built for driving a live platform was the one mode that could not
  fork a document, judge two algorithms or start a second robot. They go through
  the client now, in process or over the wire, and the smoke run walks the whole
  journey — use a shipped app, fork it, run yours, start a second robot, judge
  two algorithms — against a live server.
- **An e-stop empties the queue.** It latched immediately and then let every
  command already queued run into the latch and fail — a log full of refusals
  for a stop that worked, `applied_id` marching through commands that never ran,
  and a cell reporting an error for doing exactly what it was told. Now: what
  has not started is dropped, its ids are reported applied so nobody waits on
  them for ever, and the log says how many went. A motion cancelled *because
  someone asked* is the stop working, not the motion failing. A latched cell
  stays `held` — the operator's decision outranks whatever the interrupted
  command has to say about itself.
- **`GET /logs?cell=x` is finally about x.** Every cell shared one buffer, so a
  view that named a scope answered about every robot — the one global ADR-17
  recorded rather than hid. A record now remembers which cell produced it (the
  worker, the idle ticker and the constructor each say who they are working
  for), and a record made outside any cell belongs to the platform and shows
  everywhere, because it is equally true of all of them. Gated.
- **A second robot is waitable.** Fixing the log exposed it: `await_settled`
  polled the FIRST cell whatever you asked about, so anything driving a second
  robot returned the moment the first one happened to be idle. It names its
  cell now — through the facade, the CLI and `compare`.
- **A step argument the platform publishes is chosen, not typed.** The app
  editor computed suggestions for `station`, `capability` and `version` and then
  threw them away: every field rendered as free text, so a mistyped station
  saved cleanly and failed at run time. They are selects now — and "unset" stays
  expressible, because a select without a blank writes the first station into
  every step that had none.
- **The numeric verbs joined the table the rest of the CLI lives in.** An
  argument now says whether it is text or a number, which is what kept
  jog/movel/movep hand-written next to it: the wire wants `0.4`, not `"0.4"`.
- **The sections say what they are.** The sidebar had *Applications*, *Robots*
  and *Scenes* under one-word headings while the dock had tabs called *Apps*,
  *Scenes* and *Editor* — the same words for different things, and nothing
  anywhere said how an application differs from a scene. Each library now
  carries its own one line (an application is *what the robot does*, a robot is
  *which machine*, a scene is *where it works*), and the dock tabs are named
  after what they edit: **App editor**, **Algorithm editor**, **Scene editor**.
- **No half-features on the web surface.** Every control in the markup is now
  proven to be reached by code, and `check-design.sh` fails the build on one
  that is not — a button nothing wires looks exactly like a button that works.
  The sweep found a locale switch with no caller and no second locale, and three
  module-private helpers that were exported for no one; they are gone.
- **A tunable comes from the settings file, and nowhere else (ADR-17).** Three
  signatures defaulted to `Settings{}.motion.stop_time_s` and one to a bare
  `0.5` — a second source of truth in which the caller who forgot to pass a
  value compiles into something that looks deliberate. The defaults are gone,
  the gate keeps them gone, and removing them showed no product caller had been
  relying on one.
- **ADR-17 writes down what "MIL-style" means here**: what owns a lifetime, what
  a call gives back when it fails, whether a type is stateless or stateful, and
  where a setting comes from — including the one global that is left (the log is
  process-wide, so `GET /logs?cell=x` is not really per-cell) recorded as a
  known gap rather than described as something it is not.
- **A capability is judged from the card you were looking at.** Comparing two
  versions meant leaving the card, opening another panel, and re-picking the
  capability and both versions — so the panel opened on an empty form instead
  of on the question you already had. A card with more than one version and a
  trial now offers it directly: the live version on one side, the alternative on
  the other, one click away.
- **A command verb is a row too.** Fifteen verbs were a declaration block, a
  matching `if (*c_x) return act(...)` line, and a local variable per argument —
  three places to keep in step, and the wire name typed out in one of them with
  nothing checking it. They are now rows: what the user types, what goes on the
  wire, the arguments forwarded under their own names, and the view worth
  printing when it lands (`{key}` interpolates an argument, so a capability swap
  still prints that capability). `check-design.sh` fails the build on a row
  naming a command the gateway does not route — a verb that parsed, acked and
  did nothing was previously expressible. The numeric verbs keep their own
  lines on purpose: the wire wants numbers.
- **A mode stopped pretending to be a flag on the 1 kHz path.**
  `command_all(path_s, dt_s, bool hold)` was two behaviours behind a boolean —
  the call site named neither. `follow_all` advances the trajectory;
  `hold_all` parks every axis where its governor last had it. Same work per
  cycle, no allocation, and the existing safety-hold tests pin it.
- **A live view exists once, for every surface.** The eight views were spelled
  out four times over — eight HTTP routes, eight facade forwarders, a ten-branch
  string chain in the CLI's reader and thirteen CLI subcommands with their
  dispatch — so a ninth view would have been present in whichever of those four
  its author remembered. `Platform::kViews` is now the only place a view exists:
  name, reader and help text. The routes, the CLI verbs and the in-process
  reader are generated from it, the refusal of a robot that is not there is
  written once, and `check-design.sh` fails the build on a surface that declares
  a view by name. The CLI's document verbs come from the kinds table the same
  way. Adding a view is a row.
- **Two data races, found by running ThreadSanitizer over the suite that has
  threads in it.** The sanitizer job builds with MuJoCo off, so it had never
  covered the worker, the idle ticker and the readers running together — the
  configuration in which this platform has crashed four times. First run: 21
  warnings. The program's step and the running application's name were plain
  strings written by the worker and read by every publish; the kinematic chain
  was replaced while telemetry asked it where the tool was. Both are guarded,
  and the chain is shared so a reader keeps the one it started with. Zero
  warnings now, and a CI job that keeps it that way.

### Changed
- **The 1 kHz loop says whether it kept up.** Overruns and jitter were computed
  on every run and printed; nothing said what they were supposed to be. The
  budget is settings data, `GET /last-run` and `robonode run --quality` report
  `within_budget`, and the deploy smoke fails when a run misses it.
- **Warnings are errors in our own code.** `Status` has been `[[nodiscard]]` all
  along, and a wrapper still discarded one for a whole commit — the compiler
  said so, into a log nobody read. `-Werror` on `robonode::warnings` (never on
  vendored deps) makes the next one fail the build. It immediately found a test
  that existed and had never been called; it runs now, and passes.

### Fixed
- **A write that died mid-flight left debris.** Staging files are cleared when a
  store opens, which is the one moment nobody is mid-write.
- **`Cell::drive` answers with the work it ran.** It called the work and threw
  the result away — invisible while every caller returned void, and a silent
  liar the moment one had something to say. Converting the plan builder to
  return a reason made a refused plan report success and skip the motion; the
  wrapper propagates now, and a test pins it.

### Changed
- **The RT plan builder returns a reason instead of throwing one.**
  `SyncBlendPlan::make` replaces a factory that threw `std::invalid_argument` on
  ragged waypoints — an exception unwinding through the executive on the thread
  that owns the physics, for input a user can supply. `check-design.sh` now
  fails the build on `throw` in the seam-facing motion headers; the messages
  name the axis and the counts.
- **`Result<T>` where a value comes back.** The project is already C++23,
  so `std::expected` needs no dependency — the plan to vendor `tl::expected` was
  stale. Document reads are the first seam converted: `store.read(file)` and
  `platform.doc(kind, file, session)` return the text or the reason, instead of
  `Status f(…, std::string& out)` making every caller declare an empty string
  first and hope nobody reads it on the failing path. `Status` stays for calls
  that return nothing but can fail; the two compose, since a `Result` carries a
  `Status` as its error.

### Added
- **The header says which machine you are driving.** The Robots panel is at one
  end of the window and the transport bar at the other; "Run" has to be
  unambiguous from wherever you are looking.
- **Every surface follows the robot you chose.** The camera feed, the log, the
  verb table, a capability view and `POST /compare` all still answered about the
  first cell — a feed from a different machine than the one on screen is a lie
  the UI tells quietly. All of them take the robot now, and naming one that does
  not exist is a 404 rather than the first robot's answer.
- **A pose step can be written in degrees.** `move_pose` accepts
  `roll_deg`/`pitch_deg`/`yaw_deg` as well as a quaternion, so the application
  editor is usable by someone holding a drawing instead of a quaternion.
- **A second robot you can actually drive.** The catalogue could start one, and
  nothing could command it: every verb went to the platform's first cell. A
  command now names the robot it is for (`{"cmd":"run","cell":"fixed"}`), reads
  take `?cell=`, the event stream follows it, and the browser's Robots panel has
  a *drive* control that re-points the dashboard and the transport bar together —
  because a Run button that moves a machine you are not looking at is a trap.
  `robonode --cell <id>` does the same headless.

### Changed
- **The docs say what the code does, and a gate keeps them there.** Twenty
  iterations of features had left the roadmap snapshot claiming shipped work as
  open, the agent manual missing nine CLI verbs, and the architecture describing
  a document surface with three kinds where there are five.
  `check-design.sh` now fails the build when `AGENTS.md` does not list a verb
  `robonode --help` offers — a doc an agent reads as its surface is a doc that
  has to be true.
- **The browser suite no longer contains assertions that cannot fail.** Every
  class assertion names the whole class list; `check-design.sh` fails the build
  on a regex matcher, because `/on/` also matches `rreason`. Converting them
  found one assertion whose real value was `rstate run held` — the regex had
  been hiding a class nobody knew was set. A concurrency test that discarded
  every move's result now counts the ones that worked.
- **A failure is announced once and then recorded.** The toast used to stay up
  until the next command succeeded — a banner across the transport bar, over the
  controls you would use to recover. It fades after a few seconds; why the cell
  is faulted now sits beside the run state for as long as it is faulted, with
  the whole message in its tooltip.
- **Verbs that are pure delegation moved next to what they command.** `stop` /
  `estop` / `resume` live with `CellSupervisor`, `grasp` / `release` with the
  tool. The gateway registers them by calling one function each instead of
  owning their wiring.

### Fixed
- **Editor rows kept their controls off the edge of the panel.** The record
  widget draws a four-column row, but only emitted three cells when a record had
  no lead control (a scene override) — so the fields landed in the fixed column,
  the controls in the flexible one, and the row grew past the dock. Visible as
  three buttons where there should be four; measurable as 43px of overflow.
  Labels now sit above their inputs, because beside them they set a floor on how
  narrow a field can be, and five fields in a 300px panel then push everything
  right.

### Changed
- **The chrome fits the window it is given.** Applications were a two-column
  grid in a 300px rail, so a title took three lines and the edit button had a
  column to itself; they are rows now. Six dock tabs wrap instead of pushing
  "Compare" off the edge, no section of the rail claims the space the ones below
  it need, and nothing scrolls sideways at 900px — asserted at three viewport
  sizes, because a page that scrolls sideways is a page whose controls you
  cannot reach.

### Added
- **A robot you own is editable in the browser.** Fork a shipped one, then set
  what each axis is allowed to do — travel, speed, acceleration, jerk — in the
  same record widget the application and scene editors use (its third user, not
  a third implementation). The form is a VIEW of the document: joints,
  actuators, stations and motions it does not show survive the save untouched.

### Fixed
- **A cell dropped mid-request is no longer a use-after-free.** `CellManager`
  handed out a reference into its own map, so removing a cell while a request
  was reading it freed the gateway underneath that request. It hands out shared
  ownership now: the map may forget a cell, the work in flight may not.
- **A document is never read half-written.** Saving truncated the file and
  streamed into it, so two clients saving at once interleaved into a document
  neither wrote, and a reader in between saw an empty one. Writes land in a
  staging file and are renamed onto the target, which is atomic.
- **A crash under concurrent requests.** Every HTTP request resolves its
  session, httplib answers each on its own thread, and the map of open sessions
  was written from all of them without a lock — which corrupts the tree rather
  than losing an entry, so the crash landed inside `std::map`'s rebalance,
  nowhere near the cause. Found by the browser suite getting busy enough to
  trip it; pinned by a test that opens twelve sessions from six threads.
- **A parse failure says where it was.** `key 'position_min' not found` named
  the key and nothing else, which reads like a bug in the platform rather than a
  mistake in the file. Every element is parsed inside its own position now, so
  the message is `nodes[1] 'j2': ... position_min ...`.
- **A descriptor that contradicts itself is refused when it is saved.** A cell
  names its axes in four places (the node list, a robot's carrier and joints,
  each motion's waypoints) and nothing checked they agree — so a hand-edited
  robot that dropped an axis from one section loaded happily and failed
  somewhere unrelated later. Every reference is resolved at parse time now, on
  the one path a descriptor takes whether it came from a file at boot or from a
  user pressing Save; duplicate node ids, an axis that is both carrier and
  joint, and motions whose waypoint lists disagree are refused too. The message
  names the axis and the section that wanted it.

### Added
- **A robot catalogue.** Cell descriptors are a document kind
  (`robots`) like scenes and apps, so the platform ships a catalogue you can
  list, fork and edit — and starting a second machine is picking one from it.
  Ships `ur10e-fixed.cell.json`: the same arm bolted down, six joints where the
  first has seven, which is what makes it a different machine rather than a
  second copy. `GET /cells` now says what each running cell IS (chain length,
  family) instead of listing ids.
- **A comparison is kept.** Running one records it in the session that ran it,
  as a document of a new `runs` kind — listed, reopened and deleted like a scene
  or an app, in the browser and from the CLI (`robonode runs`, `run-record`).
  Adding that kind was one row in the workspace's table: the routes and the
  shipped-library map now build themselves from it, so no endpoint, no store
  method and no client learned a new word.
- **A planner that can see obstacles.** The Kinematics seam
  answers "would this configuration be touching something?" (MuJoCo counts
  contacts on the scratch world it already owns; impls that cannot answer say
  so, and the planner refuses rather than pretending). `robonode.avoid` plans
  the direct line, checks every configuration on it, and lifts over what is in
  the way — escalating clearance until the whole arm is clear. It never emits a
  path it knows is blocked; when it cannot find one it refuses and says how much
  clearance it tried.
- **The scene editor is a form.** Objects and overrides are rows of fields —
  name, shape, position, size, mass, collides — and the JSON underneath is a
  view of what will be saved, not a thing to type. The application editor and
  the scene editor now share one record-list widget (`web/records.js`), because
  they were the same list with different fields.
- **Comparisons say how well a version RAN**, not only whether it finished:
  worst following error (named axis, its own unit), cycles, jitter and overruns
  ride along in `POST /compare`. Two control versions take the same time and
  track the path differently — that difference is now the number on screen.
- **A stacking scenario.** `stacking-line.scene.json` +
  `stacking.app.json` place a part on top of a block; whether it stays is
  contact physics, not the plan. `place` takes an optional `slot`, because
  "on top of what is already there" is a choice, not a counter.
- **Scene overrides.** Composition was additive: a scene could add clutter
  but never move the conveyor, delete the decoy or make the part heavier. An
  `overrides` list edits what the BASE world declares, applied with the XML
  parser MuJoCo itself uses (tinyxml2) instead of string surgery — and an
  override naming a body the base does not have is refused, not ignored. Ships
  `clean-line.scene.json`.
- **A pose from the browser.** The transport bar takes roll/pitch/yaw in degrees
  (converted once, next to the inputs) and telemetry carries `tcp_quat`, so the
  boxes start from where the tool actually points.
- **6-DoF.** `Goal::kCartesianPose` has a planner: `robonode.moveP` walks
  the TCP in a straight line and slerps the orientation along it, solving a full
  pose at each step. The Kinematics seam answers real orientation and a real
  angular Jacobian (MuJoCo `mj_jacSite`), `ik_pose` solves the six-row residual
  through the SAME damped-least-squares core as the position solve, and
  `move_pose` reaches every surface at once — the wire, the CLI (`robonode
  movep`), and the application editor, which offers it because `GET /verbs`
  publishes it. A planner that cannot solve orientation refuses by name and says
  which version can. Ships `pose-demo.app.json`.
- **The application editor.** Open any app, add / reorder / remove steps, save it
  into your session. The step list is not the editor's: `GET /verbs` publishes
  the table `ProgramRunner` executes, so what can be authored is exactly what can
  run, and a program naming a verb nothing can run is refused *at save*.
  Argument suggestions come from the live capabilities and stations.
- **Trials.** Each capability names the application that puts *it* under load
  (`config/robonode.settings.json`, published in `GET /capabilities`): vision and
  camera on a bin pick, tracking on a moving-target intercept. A reach proves
  nothing about a detector.
- **`POST /compare` / `robonode compare <cap> <a> <b>`.** One implementation of
  "A vs B" in the Platform facade: run the capability's trial once per version,
  report outcome, seconds and the failure text, restore what was live. The web
  panel renders that answer instead of measuring a second one of its own.
- **Test-on-trial in the algorithm editor** — compile your version, then run it
  against the version it replaced without leaving the tab.
- **A third tracker**, `robonode.smoothed`: an exponential estimator between the
  naive snapshot and the window fit. Its factor is a setting.
- **`robonode watch --json --quiet <fields>`** so an agent loop pipes into `jq`
  and only wakes when the fields it named change; **`robonode verbs`**;
  **`robonode fork <kind> <from> <to>`** (apps and modules were unforkable).
- **Keyboard on the transport bar** — Space run · S stop · Esc e-stop · R resume,
  inert while typing, each key advertised on its button.
- **The running program says which step it is on** (`step` and `app` in
  telemetry): the transport bar shows `3/4 pick` and the library marks the card
  that is live.

### Fixed
- **A segfault while planning under a live telemetry stream.** The kinematics'
  scratch world is an mjData too, and asking it where the tool points (added
  with 6-DoF) happens on every frame while the worker plans against it.
  `MujocoKinematics` now serialises access to its own world — the lock is the
  object's invariant, not a note in its docs.
- **A segfault after every scene swap.** A planner holds a reference to the
  kinematics it solves against; swapping the scene builds a new world and
  destroyed the old chain, so the next Cartesian move dereferenced it. Found by
  running the new stacking scenario, pinned by a test.
- **An intermittent SIGSEGV in the physics.** The gripper and the conveyor
  touched MuJoCo without the cell lock while the idle ticker stepped the same
  `mjData`. `CellGateway::with_scene(fn)` is now the only way to reach the world,
  so a path that forgets the lock cannot be written.
- **A refused IK says what it knows** — "41 mm short after 200 iterations", or
  singular — instead of "did not converge", reporting the closest of both seeds.

### Changed
- **One document surface for three kinds.** Scenes, apps and modules were the
  same CRUD written three times (Platform methods, HTTP routes, CLI branches).
  One `docs(kind)` API, five routes mounted once, one fork.
- **Weak booleans became types**: `install(id, prog, Select::kNow)`,
  `constrain(name, Grip::kClosed)`, `run_conveyor(id, Belt::kRunning)`.
  `RunRecorder` lost its `enabled` flag — recording is wired, not branched.
- **Settings bind by section** through nlohmann instead of 39 field copies;
  station behaviour moved out of the gateway into `StationOps`.

## [0.8.3] — 2026-07-27 — the docs say what is true

Documentation is read by agents as instructions, so a stale doc is a bug with a
long fuse. This is a cut, not an addition: **3,659 → 2,642 lines of markdown**,
and every local link in the tree now resolves.

### Removed
- **`docs/SPEC.md`** and **`docs/REQUIREMENTS.md`** — a 2026-07-11 product brief
  describing a three-plane cloud architecture, milestones M0–M4, RBAC, OTA and
  fleet management. Almost none of it is what this became, and an agent reading
  it would plan against a system that does not exist. The one part worth keeping
  — the non-functional targets — moved into ARCHITECTURE, annotated with where
  each one actually stands rather than what we hoped.
- **`docs/REVIEW-2026-07.md`** — a point-in-time review whose findings are now
  ADRs and whose backlog is tracker. Two places to look for the same answer
  is one place too many.
- **`docs/DEPLOY-CLOUD.md`** — folded into `docs/DEPLOYMENT.md`. Deployment is
  one subject.

### Changed
- **AGENTS.md** stops describing things that do not exist (a per-issue "loop
  playbook" comment on every issue) and starts describing what does: the fast
  preset loop, `robonode --server` against a live cell, `contacts` for
  diagnosing a stalled move, and the fact that RT purity items are
  still *open* — written as if true, but not yet enforced.
- **ARCHITECTURE** carries the non-functional targets and honest issue pointers
  (the closed duplicates no longer stand in for open work).
- **REAL-ROBOTS** says what is done (a real UR10e, contacts, a weld grasp) and
  what is not (RT kinematics is still a hand-rolled position-only solve — the
  reason it exists).
- README's document table lists only documents that exist, in a stated read
  order; the design notes' "where the truth lives" point at the module map and the
  contribution docs instead of deleted files.

## [0.8.2] — 2026-07-27 — contribute to one system without touching the others

### Changed — the environment stops being a barrier
- **OpenCV is optional at configure time.** It was `REQUIRED`, so a contributor
  working on the UI, the CLI or the motion core hit a hard CMake failure over a
  dependency they did not need. Missing now costs you the `robonode.opencv`
  detector and nothing else, and the build says so once. A configure-time
  summary prints what is on and what is off, because "why is the thing I wanted
  missing" is the first question anyone asks.
- **`CMakePresets.json`**: `dev` (seconds to configure, no heavy engines),
  `sim` (the physics twin and the whole app), `full`, `asan` — with matching
  test presets, so `ctest --preset fast` is the two-second loop.
- **`scripts/setup.sh`** takes a fresh clone to a working build, and installs
  system packages only when asked (`--deps`).
- **Dev and prod are two files, not two flags.** `docker-compose.yml` is the
  immutable definition — image only, state in a named volume;
  `docker-compose.override.yml` adds the source mounts and compose loads it
  automatically. Both paths verified end to end.

### Fixed
- **The Python service image did not build.** Robotics Toolbox pulls
  `swift-sim`, which ships no arm64 wheel and compiles a C++ extension the slim
  image had no compiler for. It is now a multi-stage build: the compiler lives
  in a stage that gets thrown away. Its dependencies are pinned — an unpinned
  scientific stack means a contributor's failure is not CI's failure — and
  `make setup/run/test/lint` gives the service its own venv workflow.

### Added
- **`docs/MODULE-MAP.md`** — per area: what it owns, what it may depend on,
  which tests cover it, and the shortest loop that proves a change. Plus
  recipes: a better algorithm needs no C++, a new engine is a directory under
  `engines/` and an option defaulting OFF, the UI needs no C++ build at all.
- `.github/CODEOWNERS` routes review by area, so a change that crosses a seam is
  visible as one.

## [0.8.1] — 2026-07-27 — open to contributions

### Added
- **Apache-2.0 licence** and a `NOTICE` listing every engine we reuse and its
  licence — the project ships other people's work and says so.
- `CONTRIBUTING.md` (the bar is `scripts/verify.sh` exiting 0, and nothing
  else), `CODE_OF_CONDUCT.md`, `SECURITY.md`, and issue templates for the three
  things worth filing: a bug, a scenario, a capability.
- **`ROBONODE_PUBLIC=1`** — a public instance still lets a visitor fork a scene,
  author an algorithm and run it, but refuses the destructive minority: deleting
  someone else's session, and spawning cells (a physics cell is a CPU).
- **`fly.toml`** for a one-command cloud deploy that stays up, has a volume for
  authored work, and runs on a dedicated vCPU — the shape a stateful physics
  cell needs. `docs/DEPLOYMENT.md` covers self-hosting, Fly, Cloud Run, and
  why serverless and static hosting cannot host this application.
- A devcontainer, so a contributor can build and run it in Codespaces with no
  local toolchain.

### Changed
- The README opens for someone who has never seen the project: what it is, one
  command to run it, five things to try, and a screenshot of the physics
  reporting its own contacts.
- CI runs the **whole** suite on the physics build. The integration tests are
  where concurrency bugs surface, and previously only the unit tests ran there.

## [0.8.0] — 2026-07-26 — the physics is real

Until now the cell only *looked* like physics: every collision geom in the
world carried `contype="0" conaffinity="0"`, so the arm passed through parts and
fixtures, and a "grasp" was a pose the platform wrote down. Both are fixed.

### Changed — contacts, and a grip that holds
- **Contact groups by role** (robot · structure · parts · work surface · tool
  tip): the arm is stopped by fixtures and by the parts in its way, never by
  itself; parts rest on surfaces and on each other; the belt carries parts and
  ignores the arm; and the suction tip may reach into the part it is about to
  hold. Scene objects a user places are parts, so an obstacle really obstructs.
- **A grasp is a constraint.** The model declares a weld; closing the tool
  activates it *on the pose the bodies are in right now*, so the part is carried
  by the physics and dropped by it. `Scene` grew `constrain` and `place_body`;
  `SimGripper` closes them, and telemetry reports where the part IS.
- **The workpiece is a free body** carried by belt friction, not a pose driven
  along a slide joint. It can be picked up, stacked, knocked over and dropped.
- **The robot has a tool**: a 10 cm suction cup, the TCP at its tip. A bare
  flange put the wrist where the part was — every pick shoved its target away.
- **IK retries from mid-travel** before declaring a pose unreachable: iterative
  IK finds the solution nearest its seed, and a far carrier position is an
  awkward seed, not an unreachable target.
- The cell gained a **pallet** to place onto, and the palletizing app now has
  the line **deliver** a fresh part each cycle.

### Fixed — one owner for the physics
- **MuJoCo's `mjData` has a single owner.** Reading it from an HTTP thread while
  the worker stepped it corrupted the solver's stack — a crash inside
  `mj_collideTree`, about one run in four. The thread that drives the physics
  now samples frames and contacts; every other thread (telemetry, capability
  views, the tool pose) reads that snapshot. Regression test included: it
  hammers four reader threads through a moving-target run, and it reproduces the
  crash on the old code.

### Fixed — the shipped container ships the shipped vision
- The Docker image built without OpenCV, so the container lacked
  `robonode.opencv` — the detector the moving-target application selects. The
  build and runtime stages now install it: the flagship scenario runs in the
  image, not only on a developer's machine.
- **A tracker refuses to claim a speed it cannot know.** Sightings crowded into
  one instant fit a slope through noise; the interception then aimed metres past
  the cell. A version now needs a real time baseline (`tracking.min_span_s`)
  before it reports a velocity — and `gate_m`/`min_span_s` are settings, not
  literals.

### Added — say what is touching what
- `contacts` in telemetry, on the dashboard, in the log (one line when it
  changes), and as `robonode contacts`. "The move stopped short" and "the upper
  arm is leaning on the pallet" are now the same sentence.
- **Task steps are commands**: `pick`, `place`, `intercept`, `conveyor`,
  `deliver`, `motion` over `/command` and as CLI verbs — the same code an
  application runs, one step at a time, so debugging a cell never means
  authoring an app. Plus `robonode watch` to follow telemetry as it changes.

### Added — the CLI can drive the cell you are looking at
- **`--server <url>`**: every verb goes to a running `cell_server` over the same
  HTTP contract the web app uses. Without it the CLI booted a cell of its own,
  so an agent debugging the robot on screen was reading a different robot —
  the one trap that makes a headless surface worse than none.
- **`GET /logs`**: the log ring as a plain read. Following it no longer requires
  holding the SSE stream open.

### Changed — the UI says less and shows more
- The left rail was a second copy of the capability cards; it is now the
  workspace (session · applications · scenes) and the dock is the cell.
- Nodes + Inspect merged into one **Cell** panel: modules, then the robot's
  joints. Five tabs instead of six.
- The static "Environment" card is now **Contacts**, live from the physics; the
  station card shows what it is and how fast it runs instead of dumping its
  descriptor.

## [0.7.0] — 2026-07-25 — scenes you can edit, sessions you can work in

### Added — the scene is data
- **A scene is JSON**: the world it starts from, plus the objects placed in it
  (type, size, position, material, whether it collides, whether it slides).
  `MjcfComposer` builds the physics model from that, so moving an obstacle is
  editing a number rather than an XML tag. A scene with no objects *is* its base
  world — composing nothing costs nothing.
- **A scene library to start from**: `GET /scenes`, `GET/PUT/DELETE
  /scenes/{file}` and `POST /scenes/{file}/fork`; `robonode scenes`,
  `scene <file>`, `fork-scene <from> <to>`. Ships `conveyor-line` and
  `cluttered-line` (a crate to work around and a second part on the belt).
- The cell descriptor's `world` accepts either a physics model or a scene
  descriptor, so a scenario is chosen the same way everything else is.
- **Swap the scenario on a running platform**: `load_scene` is a command like
  any other — identified, queued behind whatever is moving, and reported when
  the world is actually live. `robonode use-scene <file>`, or Run scene in
  the browser.

### Added — sessions
- **A session is a workspace**: name one (`?session=`, `X-Robonode-Session`, or
  `--session`) and get your own scenes, applications and algorithms. Reads fall
  back to the shipped library, writes always land in the session — so the
  scenario everyone starts from cannot be broken, and forking is how you make
  one yours. Listings tag each entry `origin: library` or `origin: session`.
- `GET /sessions`, `DELETE /sessions/{id}`, `robonode sessions`.
- **In the browser**: a session box in the sidebar and a Scenes tab — the
  library, a JSON editor, save / delete / run. Naming a session reloads the
  applications, scenes and algorithms in that workspace.

## [0.6.0] — 2026-07-25 — moving-target bin picking, and the challenge model

The release where the platform starts **posing problems** rather than
demonstrating solutions. Full write-up: [docs/CHALLENGES.md](docs/CHALLENGES.md).

### Added — a workpiece that actually moves
- **The conveyor is real physics.** `conveyor-1` drives a MuJoCo slide joint at
  a commanded speed; the part is a body on that joint, so it travels down the
  belt and out of the workspace. A `Scene` seam (`celld/scene.hpp`,
  `MujocoScene`) exposes the **live** world — the one the adapters drive — so a
  station can run a belt and a sensor can watch it.
- **The world runs when the robot does not.** An idle ticker advances physics
  between commands, because a cell whose belt freezes between moves cannot pose
  a moving-target problem.

### Added — perception, for real
- **`robonode.opencv`** is a selectable vision version: HSV threshold → largest
  contour → centroid → back-projection through a `CameraModel` onto the work
  plane. Real OpenCV doing the image work, behind the existing `Detector` seam.
- **`Camera` seam + `SceneCamera`** with a pinhole model that projects and
  un-projects. A rendered or physical camera replaces it without touching a
  detector.

### Added — the Tracking capability (the seam the challenge turns on)
- **`Tracker`**: consume timestamped sightings, answer "where will it be at
  time *t*?". Ships `robonode.snapshot` (no motion model — the baseline that
  **fails**, deliberately), `robonode.constant-velocity` (least squares over a
  window, with **track gating** so a new part opens a fresh track), and a
  sandboxed user version (`x y z vx vy vz dt` → predicted pose, output
  validated before anything is planned against it).
- **`intercept`** app verb: predict where the part will be, plan there, grasp,
  and feed each attempt's measured duration back as the next attempt's lead —
  an interception has to learn how long its own reach takes.
- **`conveyor`** app verb starts, stops and recycles a line.
- **Moving bin picking** application, and `docs/CHALLENGES.md` describing the
  shape every future scenario should take.

### Added — sensing that is actually hard
- **The camera is late, noisy and cluttered.** 20 fps with ~120 ms of latency,
  sensor noise, and a decoy on the line that is the same colour as the workpiece
  and larger. Frames and detections carry a **capture time**, and the tracker
  extrapolates from *then* — acting on a stale pose as if it were current is the
  latency problem, and now the platform poses it.
- **Size-plausibility detection.** "Biggest red blob" loses to the decoy; the
  part's known size projected through the camera model wins. Detections carry a
  confidence.
- **`Goal` carries the target's velocity**, and `robonode.rendezvous` plans an
  approach that runs *with* a moving part. Measured honestly in
  [docs/CHALLENGES.md](docs/CHALLENGES.md): it costs more time than it saves on
  this cell, and beating that is left as an open problem.
- **`wait` app verb** and a bounded wait for the part to come into view — a step
  that waits now advances the world, because the scene is otherwise frozen while
  a command runs.

### Changed — the swap stack
- **The camera is its own node.** `Camera` gained a registry and a
  `CameraCapability`; a detector is now *handed* a sensor rather than building
  one, so "change the camera" and "change the vision algorithm" are independent
  choices. Ships `robonode.overhead` and `robonode.overhead-slow` (8 fps,
  350 ms latency, more noise) — swapping between them visibly degrades what the
  tracker has to work with, and nothing above the seam changes.
- **The scene is named by the cell descriptor** (`"world": "rail_ur10e.xml"`).
  Both apps hardcoded the MJCF path, which made the scene the one layer of the
  stack that could not be swapped. A different scenario is now a different
  descriptor; `add_cell` takes only an id and a descriptor.
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) documents **the swap stack** —
  every layer, its seam, how it is selected, and honestly what is not swappable
  yet (no robot catalogue, file-level scene composition, no score).

### Fixed
- **The reported TCP came from the scratch kinematics world** while the part
  came from the live scene — 0.55 m apart, so a grasp could never succeed. The
  actual TCP is now read from the live scene; forward kinematics stays on the
  scratch world where it belongs (answering "where *would* the tool be").
- **A fault was sticky**: one failed command made every later `await_settled`
  report failure. A command that succeeds clears the last error.
- **The home keyframe addressed actuators by index** and a new actuator was
  inserted in the middle, so the belt was commanded at −1.57 m/s at boot and the
  workpiece sat against its stop. The belt actuator is last, and the keyframe
  names every joint.

## [0.5.0] — 2026-07-25 — the architecture review, implemented

A full review across dead code, branding, state, robotics and modularity, then
every finding fixed. Record: the 2026-07 review (since folded into the ADRs and tracker );
the resulting design: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

### Added — the control plane
- **Commands have identity.** `CommandBus` gives every command an id, bounds the
  queue, and coalesces repeats of an idempotent verb. Telemetry carries
  `applied_id` / `accepted_id` / `state` / `latched` / `last_error`, so
  completion is observed instead of guessed.
- **The cell has a declared state machine** (`CellSupervisor`: idle · building ·
  planning · moving · held · faulted) visible on every surface.
- **A robot can be stopped.** `stop` decelerates *along the planned path* (the
  executive ramps the path clock to zero), `estop` latches the cell, `resume`
  clears it — on Platform, CLI and the transport bar. This is a software stop,
  documented as such; it is not a safety function.
- **`jog`** — jerk-limited single-axis motion through the OTG, which puts Ruckig
  on a real product path for the first time.
- **`RunRecorder`** writes every run to MCAP with the capability versions that
  produced it, so a Compare result can be reproduced.

### Added — the node model
- **`RobotNode`**: a robot is a node addressed by id, binding its carrier and
  joints *by name*. The gateway no longer indexes `nodes[1..6]`, so a second
  robot or a different arm is a descriptor change (a design gate now enforces
  this).
- **`ToolNode` / `SimGripper` / `Workpiece`**: pick and place approach, descend,
  grasp, retract and release — the palletizing app actually transports a part.
- **The carrier is part of the reach**: the rail is in the kinematic chain, and
  weighted DLS keeps it parked unless the arm cannot reach.
- **Named cell motions** (`motions` in the cell descriptor) replace the demo
  waypoint table that was hardcoded in the gateway and asserted in two others.

### Added — one capability contract
- **`CapabilityDescriptor` + `CapabilityRegistry` → `GET /capabilities`**: title,
  icon, sandbox ABI, `authorable`, versions and provenance. The web app, the CLI
  and Compare all read it; a design gate fails the build if a surface hardcodes
  a capability id. `control` declares `authorable: false`, so the editor stops
  offering an editor that could only fail.
- **Version provenance** (origin + source hash) on every registered module.

### Added — settings, not literals
- **`config/robonode.settings.json`** holds every tunable: IK damping,
  tolerance, iteration budget, step clamp, carrier weight, interpolation steps;
  motion rate, settle, stop window, max duration, limit tolerance; approach
  clearance; queue capacity, stream period, telemetry decimation; sandbox
  workspace bounds; recorder. Override with `ROBONODE_SETTINGS`; a partial file
  is a valid override.

### Added — branding + accessibility
- **`tokens.css`** is the single source for colour, type scale, space, radii,
  elevation, motion and layout — and `theme.js` reads it at runtime, so the
  **3D viewport follows the theme** (previously light mode left the canvas dark).
- **`i18n.js`** holds all copy, `Intl` formats all numbers.
- WAI-ARIA tab pattern for the dock, focusable chips and rows, one visible focus
  ring, and a live error channel: **a rejected command is now visible in the UI**.

### Fixed
- **Crash + data race**: `publish_telemetry` dereferenced the cell unguarded and
  could be null after a failed build.
- **Deadlock**: the RT telemetry hook re-entered the cell lock the worker held.
- **17-minute "moves"**: unbounded DLS IK could wind a joint many turns past its
  stop. IK is now joint-limit-projected, and `TrajectoryValidator` refuses
  out-of-travel, non-finite or absurdly long trajectories *before* anything moves.
- **Silent failures**: a bad descriptor, a half-built cell, a missing detector
  and a non-7-node cell all failed silently. They now report; `pick` refuses
  instead of driving the TCP to the world origin.
- **`Otg::duration_s` was 0 before the first update**, so a jog could truncate
  its own horizon; `Otg::estimate_duration` gives a bound from the limits.
- **A latched cell could never resume** — resume was itself blocked by the latch.

### Added — reachability of what was already modelled
- **Vendor hardware is selectable**: `CellGateway::DriverHook` lets the
  composition root register the drivers it links, so `robonode.ur-wrist` shows
  up as another version on every node when the UR adapter is built. The gateway
  library still links no vendor.
- **Multi-cell is reachable**: `POST /cells`, `DELETE /cells/{id}`,
  `robonode add-cell` / `drop-cell`. A second robot runs its own worker,
  telemetry and capabilities; the main cell cannot be removed.
- **`contracts/`** — JSON Schemas for `/telemetry`, `/nodes`, `/capabilities`
  and the command ack, **validated against the live server** by
  `e2e/contract.spec.ts`. `robonode-idl/` is now labelled what it is: the design
  document for the future gRPC surface, generating nothing today.
- **`JsonDocStore`** replaces `AppStore`: one store, with the document kind as a
  suffix, serving both applications and user modules.
- The workpiece is drawn only once located and carries a distinct held colour,
  so a grasp is visible rather than inferred.
- **`GET /model`** serves the kinematic chain from the MJCF the physics runs —
  links, joint axes and types, meshes, materials and named sites. The web viewer
  rebuilds the robot from it and poses links by joint **name**, deleting the
  hand-transcribed UR10e link table that was the last place the robot was
  described twice.

### Fixed — found by driving the CLI and reading the logs
- **The gripper could grasp thin air**: `grasp` now requires a located part
  within `app.grasp_reach_m` of the tool, and reports the distance when it
  refuses.
- **Store-backed commands returned no id** (`run_app` by file, `define_module`),
  breaking command identity on the path the UI uses. One `ack_json` now serves
  every path, covered by a contract test.
- **An unknown capability answered `{}`** with a success code; it is now refused
  (CLI exit 1, HTTP 404).
- **Rebuilding the live driver family** did the whole build again; it is a no-op.

### Added — the log accounts for a run
- Numbered program steps with their arguments, tool grasp/release events, and a
  per-move real-time summary (cycles · overruns · jitter p99/max · worst
  following error, named by axis and reported in that axis's unit) from the
  `CycleStats` the executive already computed and previously discarded.
- Every command logs its id and duration, tying the log to the acknowledgement
  the caller holds.
- A new **Inspection pass** application, authored through the store API
  (`PUT /apps/...`) and run from the CLI.

### Changed — waiting is bounded by stall, not by a guess
- `Platform::await_settled` waits while the cell reports progress and fails only
  when it goes silent (`gateway.stall_timeout_ms`). The CLI's per-verb timeouts
  (5 s / 30 s / 180 s) are deleted — a slow machine is no longer a failure.

### Changed — cleanup pass
- `apps/` is product surface only: `arm_dev` deleted (superseded by `robonode
  run` + the recorder), `rtb_dev` and `ur_governed_move` moved to `examples/`.
- Vendor choice moved out of the gateway into `robonode::app_support`; the
  boundary lint now fails if a library names an adapter.
- Driver families (`physics`, `sim`) are declared by the cell descriptor instead
  of branched on in the gateway.
- One program parser (`celld::parse_program`) serves both the saved-app path and
  the wire verb; a saved app is parsed once and run typed.
- `GET /model` carries every drawable — primitives as well as meshes, with
  materials and visibility groups — so the viewer draws the rail and carriage
  from the model rather than from constants, and filters collision geometry.
- Docs collapsed: three review documents became one review file, with
  the resulting design folded into `docs/ARCHITECTURE.md`; ADR-12…15 added.

### Removed
- `Twist`, `PlannedTrajectory`, `TimedAxisSource` and `Planner::plan_trajectory`
  — an output shape nothing produced or consumed.
- `Executive` — a **second RT loop** that had already diverged from
  `SyncExecutive` on safety semantics. One executive; a single axis is a
  one-element cell.
- `WorkerQueue` (superseded by `CommandBus`), the dead CLI `--json` flag, the
  duplicated light palette, the three re-implementations of the version-chip
  renderer, and the two command routers (now one).
- `ROBONODE_BUILD_RTB` defaults to **OFF**: it pulled a Python service no
  product path called.

## [0.4.0] — 2026-07-21 — the swappable-algorithm platform

The release where **every algorithm is a swappable module** and users + agents
can **bring their own**, behind one clean interface, over real physics.

### Added — capabilities (ADR-11: interface + registry + `set_version`)
- **Vision**, **Trajectory**, and **Control** are swappable capabilities, each a
  seam + `ModuleRegistry` + a live `set_version` verb. Ships with two versions
  each (toy-detector / top-grasp · moveL / moveJ · direct / smooth), swappable
  live from the UI and the CLI.
- **Bring-your-own algorithms**: a sandbox engine (safe expression VM — no I/O,
  fuel-limited, host-validated output) runs untrusted user code behind the
  Vision and Trajectory seams. Author from the **web editor** or `robonode
  define`; modules **persist and reload** across restarts.
- **Compare**: run two versions of a capability on the same task and read the
  metric (TCP error · peak following-error · time) — the platform's reason to
  exist, live in the UI.

### Added — surfaces
- **Web UI rebuilt** into a real dashboard: left rail (cell tree) · 3D stage +
  HUD · tabbed dock (Nodes / Editor / Inspect / Logs / Compare) · bottom
  transport bar with a live running indicator. Modular ES modules (`api`,
  `capabilities`, `dock`, `compare`, `rail`).
- **CLI at facade parity (ADR-8)**: `version`, `define`, and `vision / planner /
  control / stations / modules` reads — the whole capability surface is headless.

### Changed — architecture / consolidation
- Gateway de-godded: per-capability coordinators over a `Capability<T,Ctx>`
  base, plus reusable **`WorkerQueue<T>`** (single-consumer command queue) and
  **`TelemetryPublisher`** (snapshot assembly + storage).
- `define_module` / `set_version` dispatch is a **capability descriptor table**
  (data, not branches).
- Vendor engines grouped under `engines/`; demo apps consolidated to `arm_dev`.
- Toolchain: **C++23**, spdlog 1.15.1, ctest `fast`/`slow` labels,
  `.clang-format`, a PR-into-main CI gate.

### Fixed
- Clean C++23 build broke on spdlog v1.14.1's bundled fmt (consteval); bumped to
  1.15.1.

### Added — real WASM sandbox
- **Wasmtime engine**, built with `-DROBONODE_BUILD_WASM=ON` (vendors the
  Wasmtime C-API per platform). User source compiles to a real WebAssembly
  module — the sandbox bytecode lowers 1:1 to WASM f64 ops (`wasm::emit`), so
  **no external wasm toolchain is needed** — and runs in Wasmtime with fuel
  (instruction budget) + no WASI/imports/host access. A `WasmDetector` runs a
  user algorithm behind the `Detector` seam; a fuel trap or out-of-bounds answer
  yields no detection. Proven end to end (author → WASM → Wasmtime → validated
  pose). Default build keeps the Tier-B interpreter; the WASM engine is the same
  seam with stronger isolation.

### Known limitations (deferred, tracked)
- **Vision is a scene oracle**, not real CV (OpenCV/ONNX + camera feed).
- **Planner is DLS-IK line/point** (cuRobo/OMPL, GPU/collision-aware).
- **Error model is `Status`** (a unified, `[[nodiscard]]`, exception-free model);
  the `std::expected` migration is deferred as optional.
- **Capability folders** don't yet mirror the seams.

## [0.3.0] — real UR10e, ready apps, station-as-data, Docker stack, facade.
## [0.2.0] — Cartesian FK/IK, per-node driver swap, MuJoCo twin, web dashboard.
## [0.1.0] — motion spine: 1 kHz executive, governor, blend, descriptors.

[Keep a Changelog]: https://keepachangelog.com/en/1.1.0/
