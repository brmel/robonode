# Test strategy — and how we see the system grow

> Goal: every iteration **adds** verifiable behaviour, and that growth is **visible**. The scenario matrix below is the ledger — rows flip ▶ → ✅ as capabilities land and stay green. A slice isn't done until it adds a test here.

## Layers (a pyramid, not a pile)

| Layer | What | Where | Runs in |
|---|---|---|---|
| **Unit / module** | Pure logic per module, through its public surface (no vendor internals) | `tests/*_tests.cpp` | `robonode` + `sanitizers` CI |
| **Integration** | Cross-module through a seam we own — the contract CLI + UI both bind to | `tests/gateway_{integration,capability,concurrency,multicell}_tests.cpp`, one fixture in `gateway_fixture.hpp` | `mujoco` CI |
| **Service** | The Python kinematics service self-checks the mature library | `services/rtb-kinematics/test_service.py` | `rtb-service` CI |
| **E2E / browser** | The real user (or agent) journey in the live web app | `e2e/*.spec.ts` (Playwright) | `e2e` CI (+ Playwright MCP live) |
| **CLI e2e** | Same journeys headless, `--json` — agent parity with the UI | `robonode` CLI (#43) + robonode_cli_nodes ctest | `mujoco` CI |

**Discipline (ADR-8):** the browser e2e and the CLI e2e assert the **same journeys** — because both surfaces are thin clients of one facade, a journey that passes in one must pass in the other. That parity is the anti-divergence guard.

## Current inventory

Counted, not claimed: `scripts/verify.sh` prints the live totals at the end of a
run, and `ctest --test-dir build -N` lists the suites by name. A number written
down here goes stale the week after — this section said "8 C++ suites, 4 browser
journeys" for long enough to be wrong by a factor of three.

What the layers are is in the table above. What runs is what the build says.

## Scenario matrix — the growth ledger

Each row is a user/agent journey. ✅ verified & guarded · ▶ planned (issue) · ⏸ later.

**Evidence is the point, not the count.** A row claims a layer; the last column
says *where* — a test function, a spec name, a smoke check, a `check-design.sh`
rule, a CI job. `check-design.sh` fails the build when a row names something that
is not there, and when a row ships naming nothing at all. Every row names
something today, so there is no backlog and no exemption: a new row arrives with
its evidence or it does not arrive.

Filling this column is what demoted J32 (a vendor driver backed only by an
example that compiled) and what found that `--server` — the mode that exists so
an agent does not debug the wrong robot — had never been exercised at all.

| # | Journey | Integration | Browser e2e | CLI e2e | Status | Evidence |
|---|---|---|---|---|---|---|
| J1 | Boot cell → 7 nodes, driver versions listed | ✅ | ✅ | ✅ | **✅ live** | `test_gateway_boots_seven_nodes_with_driver_versions`; smoke "cell boots (descriptor-driven, 7 nodes)" |
| J2 | Run coordinated move → telemetry leaves home (physics) | ✅ | ✅ | ✅ | **✅ live** | `test_gateway_run_moves_the_cell_and_streams_telemetry`; smoke "RT motion + physics + worker queue + telemetry" |
| J3 | Per-node driver swap, live (try each version; incl. clock-owner swap #50) | ✅ | ✅ | ✅ | **✅ live** | `test_gateway_swaps_one_node_driver_live`; `test_gateway_swap_clock_owner_keeps_physics_alive`; smoke "trajectory capability swap (moveJ)" |
| J4 | Bad command / unknown driver fails closed | ✅ | — | ✅ | **✅** | `test_gateway_rejects_bad_commands`; `test_cell_rejects_unknown_driver` |
| J5 | Physics vs Sim family toggle rebuilds the cell | ✅ | ✅ | ✅ | **✅** | `cell.spec.ts` "driver-family control is built from what the cell declares" |
| J6 | Cartesian goal → IK → TCP arrives (in-process) | ✅ | ✅ | ✅ | **✅** | `test_gateway_cartesian_move_reaches_target`; smoke "Cartesian moveL (real IK reaches target)" |
| J7 | Bring-your-own node appears + drives | ✅ | ✅ | ✅ | **✅** | smoke "the bring-your-own driver can be selected and drives the axis" |
| J8 | Node inspector: capability/limits + live in/out/Δfollow | — | ✅ | — | **✅** | `cell.spec.ts` "node inspector shows capability/limits + live in/out" |
| J9 | Vision → pose → target reached (toy detector → pick) | ✅ | ✅ | ✅ | **✅** | `test_gateway_vision_detects_part`; `test_gateway_bin_picking_app_places_part`; smoke "program engine (bin-picking app runs to completion)" |
| J11 | Real UR10e mesh + ready pose · moveL ~0.4 cm from folded home | ✅ | ✅ | ✅ | **✅** | `test_gateway_cartesian_move_reaches_target`; `test_every_shipped_robot_is_valid`; smoke "Cartesian moveL (real IK reaches target)" |
| J10 | Bin-picking app: deploy → pick (vision) → place, one program | ✅ | ✅ | ✅ | **✅** | `test_gateway_bin_picking_app_places_part`; smoke "program engine (bin-picking app runs to completion)" |
| J12 | Logs/traces/telemetry followable from one surface (UI panel + CLI) | ✅ | ✅ | ✅ | **✅** | `test_gateway_logs_surface_records_events`; `test_log_records_steps_tool_events_and_rt_quality`; smoke "the log ring is readable without holding a stream open" |
| J13 | Theme toggle flips light/dark and persists | — | ✅ | — | **✅** | `cell.spec.ts` "theme toggle flips light/dark, persists, and repaints the 3D view" |
| J14 | Dashboard overlay (robot 3D + live physics · env · camera) | — | ✅ | — | **✅** | `cell.spec.ts` "dashboard overlay — live physics and tracking cards" |
| J15 | Application library — deploy a ready app (Pick demo) | — | ✅ | ✅ | **✅** | `cell.spec.ts` "application library — data-driven from the store, deploy runs it" |
| J16 | Version manager — chips list versions, click swaps live | ✅ | ✅ | ✅ | **✅** | `cell.spec.ts` "per-node driver swap is live (try each version)" |
| J17 | Control capability swap — direct/smooth in-loop. Smoothing is gentler on an **ideal** axis and costs peak following error on the **real** plant; both are measured | ✅ | ✅ | ✅ | **✅** | `test_control_smooth_tracks_gentler_than_direct_on_an_ideal_axis`; `test_smoothing_costs_peak_error_on_a_real_plant`; smoke "control capability swap (smooth)" |
| J18 | User authors a sandboxed vision algorithm — compile → run → validate (#69) | ✅ | ✅ | ✅ | **✅** | `test_gateway_define_user_vision_module`; smoke "vision authoring (sandbox compile + register)" |
| J19 | Web algorithm editor — write → compile → register a new version live (#70) | ✅ | ✅ | — | **✅** | `test_gateway_define_user_vision_module`; `cell.spec.ts` "freshly authored module" |
| J20 | User trajectory code — sandboxed approach planner (via+end), reaches goal (#69) | ✅ | ✅ | ✅ | **✅** | smoke "user trajectory code plans the move" |
| J21 | User modules persist — define, restart, reload + re-register on boot (#70) | ✅ | — | ✅ | **✅** | smoke "a user module outlives the process that authored it" |
| J22 | Command identity — every command acked with an id, completion observed via `applied_id` | ✅ | ✅ | ✅ | **✅** | `test_store_backed_commands_acknowledge_with_an_id`; `contract.spec.ts` "an accepted id is reported as applied" |
| J23 | Stop cancels a run by decelerating on the planned path (stop category 2) | ✅ | ✅ | ✅ | **✅** | `test_gateway_stop_cancels_a_running_motion`; `test_cancel_decelerates_on_path_and_reports` |
| J24 | E-stop latches the cell (state `held`) and refuses motion until resume (software stop) | ✅ | ✅ | ✅ | **✅** | `test_gateway_estop_latches_until_resume`; smoke "grasping thin air is refused, with the distance it measured" |
| J25 | Jog — jerk-limited single-axis move through the OTG (Ruckig) | ✅ | ✅ | ✅ | **✅** | `test_gateway_jog_moves_one_axis_via_otg`; `test_otg_reaches_target_jerk_limited`; smoke "jog drives one axis through the OTG, in that axis's own unit" |
| J26 | Robot node addressed by id; carrier + joints bound by name, units converted once | ✅ | — | — | **✅** | `test_robot_node_binds_by_name_and_scales_units` |
| J27 | Pick and place actually transport a part (grasp → carry → release at a slot) | ✅ | — | ✅ | **✅** | `test_a_pick_moves_a_real_body`; `test_gateway_palletize_uses_station_slots`; smoke "stations (data-driven)" |
| J28 | Trajectory refused before motion when out of travel / non-finite / absurd duration | ✅ | — | — | **✅** | `test_trajectory_validator_refuses_bad_plans`; `test_a_bad_plan_is_refused_with_a_reason` |
| J29 | Capabilities describe themselves — one `/capabilities` contract, no hardcoded ids | ✅ | ✅ | ✅ | **✅** | `test_gateway_capabilities_describe_themselves`; `contract.spec.ts` "capabilities match their schema"; smoke "one document out of a collection reads the same way whatever the collection" |
| J30 | Theme repaints the 3D viewport too (one token palette) | — | ✅ | — | **✅** | `cell.spec.ts` "theme toggle flips light/dark, persists, and repaints the 3D view" |
| J31 | Tuning is data — partial settings file overrides only what it names | ✅ | — | — | **✅** | `test_settings_partial_override_keeps_defaults` |
| J32 | Vendor hardware is a selectable driver — a build that links UR offers a UR version (registration, not connection) | ✅ | — | — | **✅** | `test_a_linked_vendor_driver_is_offered_as_a_version` |
| J33 | Multi-cell — a second robot is added and removed at run time | ✅ | ✅ | ✅ | **✅** | `test_platform_adds_and_removes_cells`; smoke "a second robot starts from the catalogue with its own chain" |
| J34 | Document store filters by kind (apps vs modules share one implementation) | ✅ | — | — | **✅** | `test_json_doc_store_filters_by_suffix`; `contract.spec.ts` "every document collection lists, reads, forks and deletes the same way" |
| J35 | Wire contract validated against `contracts/*.schema.json` on the live server | — | ✅ | — | **✅** | `contract.spec.ts` "telemetry matches its schema" |
| J36 | Uncaught page errors fail the browser suite (a dead initialiser is not silent) | — | ✅ | — | **✅** | `cell.spec.ts` "no console or page errors during the session" |
| J37 | The 3D view is rebuilt from `GET /model` — one kinematic source of truth | — | ✅ | — | **✅** | `cell.spec.ts` "the 3D view renders the robot from the model" |
| J38 | Every driven axis names a joint that exists in the served chain | — | ✅ | — | **✅** | `contract.spec.ts` "the kinematic chain matches its schema and binds to the node tree" |
| J39 | Driver families are descriptor data — an unknown family is refused by name | ✅ | — | ✅ | **✅** | `test_cell_rejects_unknown_driver`; `cell.spec.ts` "the driver-family control is built from what the cell declares" |
| J40 | The model carries all drawable geometry (meshes + primitives + groups) | ✅ | ✅ | — | **✅** | `contract.spec.ts` "the kinematic chain matches its schema and binds to the node tree" |
| J41 | Author an application through the store API, deploy it, run it | ✅ | ✅ | ✅ | **✅** | `test_app_store_lists_and_loads`; `test_app_descriptor_loads_program`; smoke "program engine (bin-picking app runs to completion)" |
| J42 | Grasping requires a part at the tool — closing on air is refused | ✅ | — | ✅ | **✅** | `test_gripper_refuses_to_grasp_thin_air`; smoke "grasping thin air is refused, with the distance it measured" |
| J43 | Every accepted command answers with an id, including store-backed verbs | ✅ | ✅ | ✅ | **✅** | `test_store_backed_commands_acknowledge_with_an_id`; `contract.spec.ts` "store-backed verbs acknowledge like every other command" |
| J44 | The log accounts for a run: numbered steps, tool events, RT quality, command id | ✅ | — | ✅ | **✅** | `test_log_records_steps_tool_events_and_rt_quality` |
| J45 | Waiting is bounded by stall, not duration — a slow machine is not a failure | ✅ | — | ✅ | **✅** | `test_waiting_is_bounded_by_stall_not_duration` |
| J46 | A conveyor drives a real joint; the workpiece travels and vision sees it move | ✅ | — | ✅ | **✅** | `test_vision_sees_the_part_move_on_the_belt` |
| J47 | OpenCV vision node recovers the world pose from pixels and follows the part | ✅ | — | — | **✅** | `vision_cv_tests` |
| J48 | Tracking predicts a moving target; a new part opens a fresh track (gating) | ✅ | — | — | **✅** | `test_tracker_gates_a_new_part_into_a_fresh_track`; `test_constant_velocity_tracker_extrapolates` |
| J49 | **Moving-target pick, judged fairly: with the SAME attempt budget the naive tracker misses and prediction catches** | ✅ | ✅ | ✅ | **✅** | `test_moving_target_needs_prediction_to_be_picked` |
| J50 | User-authored tracker runs sandboxed; an absurd prediction is refused | ✅ | — | — | **✅** | `test_sandboxed_tracker_runs_user_prediction`; `test_sandboxed_tracker_rejects_absurd_predictions` |
| J51 | The camera is late: the first look sees nothing, sightings carry a capture time | ✅ | — | — | **✅** | `test_vision_reports_capture_time_and_ignores_clutter`; `test_speed_needs_a_time_baseline` |
| J52 | Clutter of the same colour is rejected by size plausibility | ✅ | — | ✅ | **✅** | `test_vision_reports_capture_time_and_ignores_clutter` |
| J53 | A goal carries the target's velocity; a rendezvous planner uses it | ✅ | — | — | **✅** | `test_rendezvous_planner_approaches_along_the_targets_motion` |
| J54 | Tracking is a first-class node in the UI (rail entry + live speed card) | — | ✅ | — | **✅** | `cell.spec.ts` "dashboard overlay — live physics and tracking cards" |
| J55 | The camera is its own swappable node; a detector is handed one, never builds it | ✅ | — | ✅ | **✅** | `test_gateway_capabilities_describe_themselves`; smoke "one document out of a collection reads the same way whatever the collection" |
| J56 | The cell descriptor names its scene — no world path in any binary | ✅ | — | — | **✅** | `test_cell_descriptor_loads_nodes_and_stations`; `test_every_shipped_robot_is_valid` |
| J57 | A scene is JSON: placing an object puts a body in the physics model | ✅ | — | ✅ | **✅** | `test_composed_scene_contains_the_placed_objects`; `test_scene_descriptor_reads_objects_as_data`; smoke "swapping the scenario on a running platform changes the model" |
| J58 | An empty scene is its base world, untouched | ✅ | — | — | **✅** | `test_empty_scene_uses_the_base_world_untouched` |
| J59 | Sessions keep each user's scenes/apps/modules apart; the library is read-only | ✅ | — | ✅ | **✅** | `test_sessions_keep_each_users_work_apart`; `test_sessions_are_opened_from_many_threads_at_once`; smoke "every document kind the platform declares is reachable" |
| J60 | Fork a shipped scenario into your session and edit it | ✅ | — | ✅ | **✅** | `cell.spec.ts` "a forked scene is edited, run, and the physics model follows"; smoke "a fork is yours: the shipped document is untouched underneath it" |
| J61 | Swap the scenario on a running platform — the model follows | ✅ | — | ✅ | **✅** | `test_scene_swap_rebuilds_the_running_world`; `test_a_cartesian_move_survives_a_scene_swap`; smoke "swapping the scenario on a running platform changes the model" |
| J62 | Edit a scene in the browser and run it; the physics agrees | — | ✅ | ✅ | **✅** | `cell.spec.ts` "a scene is authored in the form and runs" |
| J63 | The workpiece rests on the belt because contacts hold it there | ✅ | ✅ | ✅ | **✅** | `cell.spec.ts` "contacts from the physics reach the dashboard" |
| J64 | A pick carries a real body; a release drops it | ✅ | ✅ | ✅ | **✅** | `test_a_pick_moves_a_real_body`; `cell.spec.ts` "a pick carries the part with the tool" |
| J65 | Every task step (pick/place/intercept/conveyor/deliver) is a command | ✅ | — | ✅ | **✅** | `test_store_backed_commands_acknowledge_with_an_id`; smoke "stations (data-driven)" |
| J66 | Surfaces read the physics concurrently without racing the worker | ✅ | — | — | **✅** | `test_readers_do_not_race_the_physics`; `test_tool_and_stations_do_not_race_the_idle_ticker` |
| J67 | The CLI drives a RUNNING cell (`--server`), not a private copy | ✅ | — | ✅ | **✅** | smoke "--server drives the RUNNING cell, not a private copy of it" |
| J68 | The log ring is readable without holding a stream open | — | ✅ | ✅ | **✅** | `test_gateway_logs_surface_records_events`; smoke "the log ring is readable without holding a stream open" |
| J69 | The verbs a program may use come from the table that runs them (`GET /verbs`) | — | — | ✅ | **✅** | `contract.spec.ts` "program verbs match their schema and are the ones a program may use"; smoke "verbs are published (an editor authors what the runner runs)" |
| J70 | **Compose an application in the browser — pick steps, save it, deploy it** | — | ✅ | ✅ | **✅** | `cell.spec.ts` "application editor — compose a program, save it, run it" |
| J71 | A program naming a verb nothing can run is refused at save, not at deploy | — | — | ✅ | **✅** | `contract.spec.ts` "program verbs match their schema and are the ones a program may use"; smoke "verbs are published (an editor authors what the runner runs)" |
| J72 | Every document kind (scenes/apps/modules) lists, reads, forks and deletes identically | — | — | ✅ | **✅** | `contract.spec.ts` "every document collection lists, reads, forks and deletes the same way"; smoke "every document kind the platform declares is reachable" |
| J73 | **Compare runs the trial the capability names — a detector is judged on a pick, not a reach** | — | ✅ | ✅ | **✅** | `cell.spec.ts` "compare runs the trial the capability names"; `test_compare_runs_each_version_on_the_capabilitys_trial`; smoke "compare reports outcome AND how well each version ran" |
| J74 | The transport bar answers the keyboard, and never while someone is typing | — | ✅ | ✅ | **✅** | `cell.spec.ts` "the transport bar answers the keyboard, except while typing" |
| J75 | `watch --json --quiet <fields>` emits machine lines only when those fields change | ✅ | — | — | **✅** | smoke "watch --json --quiet pipes one machine line per change" |
| J76 | The tool and the stations cannot race the idle ticker — one lock owns the physics | ✅ | — | — | **✅** | `test_tool_and_stations_do_not_race_the_idle_ticker` |
| J77 | A third tracker version (smoothed) is selectable with no seam change | ✅ | — | ✅ | **✅** | `test_smoothed_tracker_converges_and_survives_a_jump` |
| J78 | A refused IK says how far short it got, not just that it failed | ✅ | — | — | **✅** | smoke "a refusal says what the solver knew, not just that it failed" |
| J79 | **The running program says which step it is on, in the UI** | — | ✅ | ✅ | **✅** | `cell.spec.ts` "the running program says which step it is on" |
| J80 | Comparing two versions is ONE implementation in the facade (`POST /compare`, `robonode compare`) | ✅ | ✅ | ✅ | **✅** | `test_compare_runs_each_version_on_the_capabilitys_trial`; smoke "compare reports outcome" |
| J81 | **Author a version, then test it against the one it replaced, without leaving the editor** | — | ✅ | ✅ | **✅** | `cell.spec.ts` "a freshly authored module can be tested against the version it replaced" |
| J82 | The library says which application is running, and which step it is on | — | ✅ | — | **✅** | `cell.spec.ts` "the running program says which step it is on" |
| J83 | **The dashboard shows the frame the detector reads, with the platform's crosshair on it** | — | ✅ | — | **✅** | `cell.spec.ts` "the dashboard shows the frame the detector reads" |
| J84 | Waiting for the cell wakes on publication, and a faulted cell settles with its reason | ✅ | — | ✅ | **✅** | `test_one_definition_of_done`; `test_waiting_is_bounded_by_stall_not_duration` |
| J85 | A refusal names what the code knew: which goal kind, how many joints, which version | ✅ | — | ✅ | **✅** | smoke "a refusal says what the solver knew, not just that it failed"; `test_an_impossible_motion_is_refused_not_swallowed` |
| J86 | **6-DoF: a pose goal reaches the point AND the angle (`robonode.moveP`, `move_pose`)** | ✅ | — | ✅ | **✅** | `test_move_pose_reaches_a_full_pose`; smoke "6-DoF: an app selects the pose planner" |
| J87 | The Kinematics seam answers real orientation and a real angular Jacobian | ✅ | — | — | **✅** | `test_move_pose_reaches_a_full_pose` |
| J88 | A planner that cannot solve orientation refuses by name and says which version can | ✅ | — | ✅ | **✅** | `test_an_impossible_motion_is_refused_not_swallowed`; smoke "a refusal says what the solver knew, not just that it failed" |
| J89 | The transport bar commands a pose (degrees in, quaternion on the wire) | — | ✅ | ✅ | **✅** | `cell.spec.ts` "the transport bar commands a pose, not just a point" |
| J90 | **A scene can move or remove what the base world declares; an unknown body is refused** | ✅ | ✅ | — | **✅** | `test_overrides_move_and_remove_what_the_base_declares`; `cell.spec.ts` "a scene override removes a body the base world declared" |
| J91 | Comparing versions reports how well each RAN (worst Δfollow, cycles, jitter), not only whether | ✅ | ✅ | ✅ | **✅** | smoke "compare reports outcome AND how well each version ran" |
| J92 | **Stacking: a part placed on a block is judged by contact, not by the plan** | ✅ | — | ✅ | **✅** | `test_a_pick_moves_a_real_body` |
| J93 | A Cartesian move after a scene swap does not use the previous world's kinematics | ✅ | — | — | **✅** | `test_a_cartesian_move_survives_a_scene_swap` |
| J94 | **A scene is authored in a form — objects and overrides as fields, never typed JSON** | — | ✅ | — | **✅** | `cell.spec.ts` "a scene is authored in the form and runs" |
| J95 | A whole program still runs after a scene swap (vision, planner and tool all rebuilt) | ✅ | — | — | **✅** | `test_scene_swap_rebuilds_the_running_world` |
| J96 | The Kinematics seam answers "would this configuration be touching something?" | ✅ | — | — | **✅** | `test_the_avoiding_planner_never_plans_through_the_crate` |
| J97 | **A straight line plans through the crate; `robonode.avoid` never does — it routes round or refuses** | ✅ | — | ✅ | **✅** | `test_the_avoiding_planner_never_plans_through_the_crate` |
| J98 | Planning does not race the telemetry readers on the kinematics' own world | ✅ | — | — | **✅** | `test_planning_does_not_race_the_telemetry_readers` |
| J99 | **A comparison is kept as a document — listed, reopened and deleted like a scene** | — | ✅ | ✅ | **✅** | `cell.spec.ts` "compare runs the trial the capability names"; smoke "compare reports outcome AND how well each version ran" |
| J100 | A new document kind is one row: no new route, no new store method, no new client code | ✅ | — | ✅ | **✅** | `test_json_doc_store_filters_by_suffix`; `contract.spec.ts` "every document collection lists, reads, forks and deletes the same way"; smoke "every document kind the platform declares is reachable" |
| J101 | **The driver-family control is built from what the cell declares, not from markup** | — | ✅ | — | **✅** | `cell.spec.ts` "the driver-family control is built from what the cell declares" |
| J102 | The cell's state is a declared set the contract pins, and the server reports one of them | — | ✅ | — | **✅** | `contract.spec.ts` "the reported state is one the contract declares" |
| J103 | **A robot is a catalogue entry: a second one starts with its own chain (6 joints, not 7)** | ✅ | ✅ | ✅ | **✅** | `test_a_second_robot_runs_with_its_own_chain`; smoke "a second robot starts from the catalogue with its own chain" |
| J104 | A descriptor the catalogue does not have is refused, and nothing starts | ✅ | — | — | **✅** | `test_platform_adds_and_removes_cells` |
| J105 | **A descriptor that contradicts itself is refused when it is SAVED, naming the axis** | ✅ | ✅ | — | **✅** | `test_a_descriptor_that_contradicts_itself_is_refused`; `cell.spec.ts` "a forked robot is yours, and a self-contradicting one is refused" |
| J106 | Every robot the platform ships is one it can actually run | ✅ | — | — | **✅** | `test_every_shipped_robot_is_valid` |
| J107 | A parse failure names the element it was in (`nodes[1] 'j2'`), not just the missing key | ✅ | — | — | **✅** | `celld_tests` — the message names `nodes[1] 'j2'` |
| J108 | **A robot you own is edited in a form; what the form does not show survives the save** | — | ✅ | — | **✅** | `cell.spec.ts` "a forked robot is yours, and a self-contradicting one is refused" |
| J109 | Sessions are opened from many request threads at once without corrupting the map | ✅ | — | — | **✅** | `test_sessions_are_opened_from_many_threads_at_once` |
| J110 | **A cell dropped while a request reads it stays alive until that request is done** | ✅ | — | — | **✅** | `test_a_cell_can_be_dropped_while_it_is_being_read` |
| J111 | A document is never read half-written: a save lands whole or not at all | ✅ | — | — | **✅** | `test_a_document_is_never_read_half_written` |
| J112 | **The chrome fits the window: nothing scrolls sideways, every dock tab stays reachable** | — | ✅ | — | **✅** | `cell.spec.ts` "the chrome fits the window it is given" |
| J113 | An editor row keeps its controls: a panel with content in it stays inside the dock | — | ✅ | — | **✅** | `cell.spec.ts` "a panel with content in it stays inside the dock" |
| J114 | **A failure is announced once, and why the cell is faulted stays visible until it is not** | — | ✅ | — | **✅** | `cell.spec.ts` "a failure is announced once and the reason stays" |
| J115 | The agent manual lists every CLI verb, and every document kind is reachable | ✅ | — | ✅ | **✅** | smoke "every document kind the platform declares is reachable" |
| J116 | **A command drives the robot it names; the one it does not name stays put** | ✅ | — | ✅ | **✅** | `test_a_command_drives_the_robot_it_names`; smoke "naming a robot that is not there fails instead of answering about another" |
| J117 | The browser (and `robonode --cell`) point every read and command at one robot | — | ✅ | ✅ | **✅** | `cell.spec.ts` "the browser drives the robot it is pointed at" |
| J118 | **Every read follows the chosen robot — camera, logs, verbs, capabilities, compare** | ✅ | ✅ | — | **✅** | `test_every_declared_view_answers_and_refuses_a_ghost` |
| J119 | A pose step can be authored in degrees; the platform still stores a quaternion | ✅ | — | — | **✅** | `test_a_pose_step_can_be_written_in_degrees` |
| J120 | A stream for a robot that does not exist is refused, not opened empty | — | — | ✅ | **✅** | `test_every_declared_view_answers_and_refuses_a_ghost`; smoke "naming a robot that is not there fails instead of answering about another" |
| J121 | **A read hands back the document or the reason — the failing path carries no value** | ✅ | — | — | **✅** | `test_a_read_returns_the_document_or_the_reason` |
| J122 | A plan that cannot be made says why instead of throwing through the executive | ✅ | — | — | **✅** | `test_a_bad_plan_is_refused_with_a_reason` |
| J123 | An impossible motion is refused, not swallowed by the wrapper that catches exceptions | ✅ | — | — | **✅** | `test_an_impossible_motion_is_refused_not_swallowed` |
| J124 | **Every document the platform SHIPS is one it can run — apps name only verbs that exist** | ✅ | — | — | **✅** | `test_every_shipped_document_is_runnable` |
| J125 | A write that died mid-flight leaves no debris behind it | ✅ | — | — | **✅** | `test_debris_from_a_dead_write_is_cleared` |
| J126 | **The threaded suite is clean under ThreadSanitizer, in CI, with MuJoCo linked** | ✅ | — | — | **✅** | CI job `robonode-races` |
| J127 | The 1 kHz loop reports whether it met its budget, and the smoke fails if it did not | ✅ | — | ✅ | **✅** | smoke "the 1 kHz loop kept up" |
| J128 | **Every suite that links physics is clean under ASan + UBSan — the sanitizers cover the code that actually runs** | ✅ | — | — | **✅** | CI job `robonode-sanitizers` |
| J129 | The threaded suites are ALL clean under ThreadSanitizer, not just the one that crashed | ✅ | — | — | **✅** | CI job `robonode-races` (all four threaded suites) |
| J130 | A live view of a robot that is not there is refused — for every view, because there is one implementation | ✅ | ✅ | — | **✅** | `contract.spec.ts` "refused, for every view" |
| J131 | **Every live view the platform declares answers on every surface — the routes and the CLI verbs are generated from one table** | ✅ | ✅ | ✅ | **✅** | `test_every_declared_view_answers_and_refuses_a_ghost`; smoke "a command verb prints the view its own row names, not a generic one" |
| J132 | A CLI command verb names a command the gateway routes, and prints the view its own row names | — | — | ✅ | **✅** | smoke "a command verb prints the view its own row names"; `check-design.sh` 6a2 |
| J133 | **One owner for the live world: physics is reachable only under the cell lock, and the gate says so** | ✅ | — | — | **✅** | `check-design.sh` 5b; `gateway_concurrency_tests` |
| J134 | **A comparison reports a version's quality only from the trial that version ran** — a trial that never moved reports none | ✅ | — | ✅ | **✅** | smoke "each from its OWN trial"; `test_run_records_are_identified` |
| J135 | A capability is judged from its own card, pre-loaded against the version that is live | — | ✅ | — | **✅** | `cell.spec.ts` "judged from its own card" |
| J136 | **No half-features on the web surface: every control in the markup is reached by code, and the gate says so** | ✅ | — | — | **✅** | `check-design.sh` 6z |
| J137 | **A tunable comes from the settings file, never from a default argument (ADR-17)** | ✅ | — | — | **✅** | `check-design.sh` 5a |
| J138 | **A cell-scoped view answers about THAT cell — including the log, the last global left** | ✅ | — | — | **✅** | `test_logs_belong_to_the_cell_that_made_them`; `check-design.sh` 5c |
| J139 | Waiting for a cell to settle names the cell; a second robot is waitable | ✅ | — | ✅ | **✅** | `test_logs_belong_to_the_cell_that_made_them` (waits on "second") |
| J140 | A step argument the platform publishes is chosen, not typed — and "unset" stays expressible | — | ✅ | — | **✅** | `cell.spec.ts` "a choice, not a text box" |
| J141 | **One definition of "done" (ADR-12): the facade and the CLI wait on the same predicate, and the gate forbids a second** | ✅ | — | — | **✅** | `test_one_definition_of_done`; `check-design.sh` 4c |
| J142 | A CLI verb that parses and dispatches nothing fails the build | ✅ | — | — | **✅** | `check-design.sh` 6y |
| J143 | Reading one document out of a collection is one table, not one branch per collection | — | — | ✅ | **✅** | smoke "one document out of a collection reads the same way whatever the collection" |
| J144 | **An e-stop empties the queue: nothing that was about to start does, every id is accounted for, and the cell is `held` rather than faulted** | ✅ | ✅ | ✅ | **✅** | `test_an_estop_empties_the_queue_and_is_not_a_failure`; smoke "e-stop latches the cell, and a stop that worked is not a failure"; `cell.spec.ts` "the transport bar answers the keyboard, except while typing" |
| J145 | **Every verb works against a RUNNING server, not only in process** — fork, compare, add-cell and drop-cell were refused as "local-only" by a client talking to a server that serves them | — | — | ✅ | **✅** | smoke "the whole journey works against a running server, not just in process" |

**Green: 20/21 journeys (J1–J4, J6–J9, J12–J21, +CLI parity).** Add its integration + e2e cell here and flip its row.

## Run the sanitizers where the threads are

Both sanitizer jobs used to build with MuJoCo **off** — which excluded every
test that runs a worker, an idle ticker and a reader at once, so between them
they had never seen a single one of the four races that crashed this platform.
A sanitizer pointed at the code that could not have been wrong is a job that
passes forever and tells you nothing.

Both now link physics:

```sh
# threads
cmake -B build-tsan -DCMAKE_BUILD_TYPE=RelWithDebInfo -DROBONODE_BUILD_UR_ADAPTER=OFF \
  -DCMAKE_CXX_FLAGS="-fsanitize=thread -fno-omit-frame-pointer" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=thread"
cmake --build build-tsan -j --target gateway_concurrency_tests gateway_integration_tests \
  gateway_multicell_tests mujoco_tests arm_tests \
  scene_tests celld_tests
for t in gateway_concurrency gateway_integration gateway_multicell mujoco arm scene celld; do ./build-tsan/tests/${t}_tests || break; done

# memory + undefined behaviour (C flags too: MuJoCo is C, and its sanitizer
# shim only compiles with the extensions sim-mujoco/CMakeLists.txt turns on)
cmake -B build-asan -DCMAKE_BUILD_TYPE=RelWithDebInfo -DROBONODE_BUILD_UR_ADAPTER=OFF \
  -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer" \
  -DCMAKE_C_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined"
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=print_stacktrace=1 ctest --test-dir build-asan
```

TSan's first run: 21 warnings, all real. With physics linked, ASan and UBSan
are clean across all five suites — but that is now a fact about the code that
runs, not about the code that was left out.

## A claim that did not survive being measured — and what it took to earn it

J49 read "the naive tracker misses, prediction catches it". Hardening the test
meant giving both algorithms the same attempt budget, and with an equal budget
they were **not separable at all**: runs existed where snapshot succeeded and
constant-velocity did not. The old test compared two attempts against four.

The algorithms were not the problem. The scene was, in two measurable ways:

- **the part stopped.** It ran to the end of the belt and stayed there, and a
  stationary part is catchable by anything. The line recirculates now — a
  conveyor that has carried its part off the end puts a fresh one at the head,
  unless the gripper is holding it.
- **one reach took most of a traverse.** 0.85 m at 0.08 m/s is 10.6 s while a
  reach takes ~7.7 s: 1.4 attempts per traverse, so no tracker could converge
  before the part was gone. At 0.03 m/s that is 3.7 attempts — room to correct
  a lead.

With both fixed and **five attempts each**, snapshot fails and constant-velocity
succeeds, repeatably. The row looked proved for months, and what proved it was a
pair of tuned budgets.

## Tests that fail for the wrong reason

Two suites measure real time: the 1 kHz loop's jitter against its budget, and an
arm reaching for a part on a moving belt. Both have failed and both passed
immediately afterwards.

The first diagnosis here said "under a full parallel `ctest` run" — which was
wrong, and worth recording as wrong. `ctest` is serial: nothing passes `-j` and
nothing sets `CTEST_PARALLEL_LEVEL`. The load was **another `cell_server`** left
running from browser work, i.e. a second physics engine competing for the same
cores. The number those tests then report is about the machine, not the code.

`verify.sh` refuses to start while a `cell_server` is up, and says why. That is
cheaper than making a real-time assertion loose enough to pass on a busy machine,
which would be the same as not asserting it.

## Assertions that cannot fail

A test that passes whether or not the behaviour is there is worse than no test:
it occupies the place where a real one would go. Two shapes have been found here
and both are now gated or fixed:

- **`toHaveClass(/on/)`** also matches an element whose base class is `rreason`.
  Browser assertions name the **whole** class list (`'rreason on'`), which is
  enforced by `scripts/check-design.sh`. Converting the suite immediately found
  one assertion whose real value was `rstate run held`, not `rstate held` — the
  regex had been hiding a class nobody knew was there.
- **A discarded result in a loop.** `(void)p.move_l(...)` inside a concurrency
  test proves only that the readers ran; if every move failed the test still
  passed. Where the point is that work HAPPENS under load, count the successes
  and assert on the count.

When a test cannot fail, delete it or make it real. Leaving it is the worst of
the three options, because the row in the matrix says the journey is covered.

## Running the layers

```sh
# Unit + integration (C++) — configure once, then:
cmake -B build && cmake --build build -j && ctest --test-dir build --output-on-failure

# Browser e2e — needs a live cell_server:
docker compose up -d                       # or: ROBONODE_WORLDS_DIR=engines/sim-mujoco/worlds ./build/apps/cell_server/cell_server &
npm install && npx playwright install --with-deps chromium
E2E_BASE_URL=http://localhost:8080 npm run test:e2e
```

An agent can also drive the live app directly through the **Playwright MCP server** (navigate / snapshot / click / screenshot) for exploratory or ad-hoc verification — the checked-in `e2e/*.spec.ts` is the same journey pinned for CI.

## How growth stays honest

- **Every issue ships a test** in the layer it belongs to; PRs that add behaviour without a matrix row are incomplete.
- **CI is the gate**: unit+integration (`ctest`) and browser (`playwright`) must be green to merge.
- **The matrix is reviewed each iteration** — ▶ → ✅ is the unit of progress, and ✅ rows never regress (a red row blocks release).
