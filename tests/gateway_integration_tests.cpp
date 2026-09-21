// Integration test — the CellGateway seam end to end: JSON command in →
// worker thread → celld → motion → MuJoCo physics → telemetry JSON out. This
// is the SAME contract the web UI and the (coming) CLI both bind to (#43), so
// exercising it here is exercising both surfaces at once. Cross-module by
// construction: gateway + celld + motion + sim_mujoco on one shared world.
//
// Built only with ROBONODE_BUILD_MUJOCO (needs the physics world).

#include "gateway_fixture.hpp"

using namespace robonode::testing;
using nlohmann::json;
using namespace std::chrono_literals;

namespace {

void test_gateway_boots_seven_nodes_with_driver_versions() {
    auto gw = gateway();
    const auto j = json::parse(gw.nodes_json());
    CHECK(j.at("family") == "physics");
    CHECK(j.at("nodes").size() == 7);
    CHECK(j.at("nodes")[0].at("id") == "rail-x");
    CHECK(j.at("nodes")[0].at("driver") == "robonode.mujoco-axis");
    // Every node offers the same forced-interface version list.
    const auto avail = j.at("available");
    bool has_mj = false, has_sim = false, has_soft = false;
    for (const auto& d : avail) {
        has_mj |= d == "robonode.mujoco-axis";
        has_sim |= d == "robonode.sim-axis";
        has_soft |= d == "robonode.sim-axis-soft";
    }
    CHECK(has_mj && has_sim && has_soft);
}

// Command in → coordinated move → telemetry out: the rail traverses to its
// final waypoint (400 mm) under real physics, proving the whole loop streamed.
void test_gateway_run_moves_the_cell_and_streams_telemetry() {
    auto gw = gateway();
    CHECK(json::parse(gw.submit_command(R"({"cmd":"run"})")).at("ok") == true);

    // rail-x (pos[0]) must reach its final waypoint (400 mm) within the run.
    const bool arrived = poll_until(
        [&] { return gw.telemetry_json(); },
        [](const json& j) {
            const auto& pos = j.at("pos");
            return !pos.empty() && std::abs(double(pos[0]) - 400.0) < 20.0;
        },
        20s);
    CHECK(arrived);
}

// A single node's implementation is swapped live through the registry — the
// per-node "try each version" contract. The published tree reflects it.
void test_gateway_swaps_one_node_driver_live() {
    auto gw = gateway();
    CHECK(json::parse(gw.submit_command(R"({"cmd":"set_driver","node":"j3","driver":"robonode.sim-axis"})"))
              .at("ok") == true);

    const bool swapped = poll_until(
        [&] { return gw.nodes_json(); },
        [](const json& j) {
            for (const auto& n : j.at("nodes")) {
                if (n.at("id") == "j3") return n.at("driver") == "robonode.sim-axis";
            }
            return false;
        },
        5s);
    CHECK(swapped);
}

// Regression (#50): swapping the node that used to be the physics "clock
// owner" (rail-x, built first) to a non-physics driver must NOT freeze the
// shared world — the remaining physics joints still move. World-stepping is
// the executive's job now, not any adapter's identity.
void test_gateway_swap_clock_owner_keeps_physics_alive() {
    auto gw = gateway();
    CHECK(json::parse(gw.submit_command(
                          R"({"cmd":"set_driver","node":"rail-x","driver":"robonode.sim-axis"})"))
              .at("ok") == true);
    CHECK(poll_until(
        [&] { return gw.nodes_json(); },
        [](const json& j) {
            for (const auto& n : j.at("nodes")) {
                if (n.at("id") == "rail-x") return n.at("driver") == "robonode.sim-axis";
            }
            return false;
        },
        5s));

    CHECK(json::parse(gw.submit_command(R"({"cmd":"run"})")).at("ok") == true);
    // j1 (pos[1], still robonode.mujoco-axis) must move off zero under physics.
    const bool j1_moved = poll_until(
        [&] { return gw.telemetry_json(); },
        [](const json& j) {
            const auto& pos = j.at("pos");
            return pos.size() > 1 && std::abs(double(pos[1])) > 0.05;
        },
        20s);
    CHECK(j1_moved);
}

// Bring-your-own (#23): the user example driver is registered, shows up in
// every node's version list, and drives a node when selected — proving a
// third-party AxisAdapter needs nothing but the seam.
void test_gateway_byo_example_driver_selectable_and_drives() {
    auto gw = gateway();
    const auto avail = json::parse(gw.nodes_json()).at("available");
    bool has_byo = false;
    for (const auto& d : avail) has_byo |= d == "robonode.byo-example";
    CHECK(has_byo);

    CHECK(json::parse(gw.submit_command(
                          R"({"cmd":"set_driver","node":"j2","driver":"robonode.byo-example"})"))
              .at("ok") == true);
    CHECK(poll_until(
        [&] { return gw.nodes_json(); },
        [](const json& j) {
            for (const auto& n : j.at("nodes")) {
                if (n.at("id") == "j2") return n.at("driver") == "robonode.byo-example";
            }
            return false;
        },
        5s));

    CHECK(json::parse(gw.submit_command(R"({"cmd":"run"})")).at("ok") == true);
    const bool j2_moved = poll_until(
        [&] { return gw.telemetry_json(); },
        [](const json& j) {
            const auto& pos = j.at("pos");
            return pos.size() > 2 && std::abs(double(pos[2])) > 0.05;
        },
        20s);
    CHECK(j2_moved);
}

// #42/#44: events land on the log surface — after a run, logs_json carries the
// run record. The same followable stream the UI panel and `robonode logs` tail.
void test_gateway_logs_surface_records_events() {
    auto gw = gateway();
    CHECK(json::parse(gw.submit_command(R"({"cmd":"run"})")).at("ok") == true);
    CHECK(poll_until(
        [&] { return gw.telemetry_json(); },
        [](const json& j) {
            const auto& pos = j.at("pos");
            return !pos.empty() && std::abs(double(pos[0]) - 400.0) < 20.0;
        },
        20s));
    const auto logs = json::parse(gw.logs_json());
    CHECK(logs.is_array() && !logs.empty());
    bool has_run = false;
    for (const auto& l : logs) has_run |= std::string(l).find("run") != std::string::npos;
    CHECK(has_run);
}

// #22: a Cartesian goal — real IK plans a straight moveL and the arm reaches
// the TCP target in physics (within servo tolerance).
void test_gateway_cartesian_move_reaches_target() {
    auto gw = gateway();
    const auto home = json::parse(gw.telemetry_json());
    CHECK(home.contains("tcp"));
    const auto& h = home.at("tcp");
    const double tx = double(h[0]) + 0.06, ty = double(h[1]) - 0.05, tz = double(h[2]) - 0.08;
    const json c = {{"cmd", "move_l"}, {"x", tx}, {"y", ty}, {"z", tz}};
    CHECK(json::parse(gw.submit_command(c.dump())).at("ok") == true);

    const bool arrived = poll_until(
        [&] { return gw.telemetry_json(); },
        [&](const json& j) {
            if (!j.contains("tcp")) return false;
            const auto& t = j.at("tcp");
            const double dx = double(t[0]) - tx, dy = double(t[1]) - ty, dz = double(t[2]) - tz;
            return dx * dx + dy * dy + dz * dz < 0.03 * 0.03;  // real UR10e servos reach ~cm
        },
        20s);
    CHECK(arrived);
}

// #61/#64: the program engine deploys a real saved app end to end. The
// bin-picking program is family → pick (vision → part) → move_l (place); the
// TCP must finish at the place target — the vision→pick→place primitive.
void test_gateway_bin_picking_app_places_part() {
    auto p = platform(ROBONODE_APPS);
    CHECK(p.run_app("bin-picking.app.json").ok());
    // Wait on the platform's own completion signal, not a wall clock: a loaded
    // machine makes a physics run slower, and that is not a failure.
    CHECK(p.await_settled(kProgramTimeout).ok());

    const auto j = json::parse(p.telemetry_json());
    const auto& t = j.at("tcp");
    const double dx = double(t[0]) - 0.5, dy = double(t[1]) + 0.3, dz = double(t[2]) - 0.5;
    CHECK(dx * dx + dy * dy + dz * dz < 0.05 * 0.05);  // finished at the place target
}

// #31: the palletizing app is data-driven — `place` asks the pallet station for
// its next slot (the station computes the target, the program does not). The TCP
// finishes at the last computed slot.
void test_gateway_palletize_uses_station_slots() {
    auto p = platform(ROBONODE_APPS);
    const auto st = json::parse(p.stations_json());
    bool has_pallet = false;
    for (const auto& s : st) has_pallet = has_pallet || s.at("type") == "pallet";
    CHECK(has_pallet);
    CHECK(p.run_app("palletizing.app.json").ok());
    CHECK(p.await_settled(kProgramTimeout).ok());

    // The part is actually carried: the line delivers it, the tool grasps at the
    // detection, and lets go over the station's slot. Where it comes to rest is
    // the physics' answer — it lands on the pallet and stays there.
    const auto j = json::parse(p.telemetry_json());
    CHECK(j.contains("workpiece"));
    const auto& w = j.at("workpiece");
    CHECK(!w.value("held", true));
    const auto& pose = w.at("pose");
    CHECK(std::abs(double(pose[0]) - 0.55) < 0.3);  // over the pallet footprint
    CHECK(std::abs(double(pose[1]) + 0.55) < 0.25);
    CHECK(double(pose[2]) > 0.15);                   // resting on it, not on the floor
}

// The forced interface refuses what it does not know — bad commands and
// unregistered driver families fail closed, never corrupt the cell.
void test_gateway_rejects_bad_commands() {
    auto gw = gateway();
    CHECK(json::parse(gw.submit_command(R"({"cmd":"nonsense"})")).at("ok") == false);
    CHECK(json::parse(gw.submit_command(R"({"cmd":"driver","family":"warp"})")).at("ok") == false);
    CHECK(json::parse(gw.submit_command("not json at all")).at("ok") == false);
    // The cell is still intact and serving 7 nodes.
    CHECK(json::parse(gw.nodes_json()).at("nodes").size() == 7);
}

// #33: the Platform facade — one typed entry (run / set_node_driver /
// nodes_json), no JSON hand-crafting; the same object the CLI (#43) binds to.
void test_platform_facade_typed_verbs() {
    auto p = platform();
    CHECK(json::parse(p.nodes_json()).at("nodes").size() == 7);
    CHECK(p.run().ok());
    const bool arrived = poll_until(
        [&] { return p.telemetry_json(); },
        [](const json& j) {
            const auto& pos = j.at("pos");
            return !pos.empty() && std::abs(double(pos[0]) - 400.0) < 20.0;
        },
        20s);
    CHECK(arrived);
    CHECK(p.set_node_driver("j3", "robonode.sim-axis").ok());
}


// One definition of "done" (ADR-12). The facade and the CLI-over-HTTP both wait
// on the same predicate, so it is worth pinning what it says: a frame that has
// applied everything and stopped is settled; one still working reports a token
// that changes while it makes progress.
void test_one_definition_of_done() {
    const auto settled = robonode::progress_of(json{{"applied_id", 4}, {"accepted_id", 4}, {"running", false}});
    CHECK(settled.settled);
    CHECK(settled.error.empty());

    const auto failed = robonode::progress_of(
        json{{"applied_id", 4}, {"accepted_id", 4}, {"running", false}, {"last_error", "nope"}});
    CHECK(failed.settled);
    CHECK(failed.error == "nope");

    // Accepted more than it has applied: not done, whatever `running` says.
    const auto behind = robonode::progress_of(json{{"applied_id", 3}, {"accepted_id", 4}, {"running", false}});
    CHECK(!behind.settled);
    // Still moving on the command it has applied: also not done.
    const auto moving = robonode::progress_of(
        json{{"applied_id", 4}, {"accepted_id", 4}, {"running", true}, {"t", 1.5}});
    CHECK(!moving.settled);
    CHECK(moving.token != behind.token);   // progress is visible between frames
    CHECK(!robonode::progress_of(json{}).settled);   // a frame that parsed to nothing is not "done"
}

// A vendor driver the app links shows up as a selectable version, without a
// robot on the other end of it. Registration is the claim — that a build which
// links UR offers a UR version everywhere a version can be chosen — and it is
// the half that does not need hardware. This row sat green for months on the
// strength of an example that merely compiled.
#ifdef ROBONODE_WITH_UR
void test_a_linked_vendor_driver_is_offered_as_a_version() {
    auto gw = gateway();
    const auto available = json::parse(gw.nodes_json()).at("available");
    bool offered = false;
    for (const auto& version : available) {
        offered = offered || version.get<std::string>() == "robonode.ur-wrist";
    }
    CHECK(offered);
}
#else
void test_a_linked_vendor_driver_is_offered_as_a_version() {}
#endif

// An e-stop means nothing new starts — including the things already queued.
// Letting them run to be refused one by one filled the log with failures for a
// stop that worked, marched applied_id through commands that never ran, and left
// the cell reporting an error for doing exactly what it was told.
void test_an_estop_empties_the_queue_and_is_not_a_failure() {
    auto gw = gateway();
    for (int i = 0; i < 3; ++i) {
        (void)gw.submit("move_l", {{"x", 0.85}, {"y", 0.25}, {"z", 0.55}});
    }
    const auto ack = json::parse(gw.submit("estop", {}));
    CHECK(ack.at("ok") == true);

    // Everything accepted is accounted for: nothing is left pending, and no id
    // is stranded for a caller waiting on it.
    const bool settled = poll_until(
        [&] { return gw.telemetry_json(); },
        [](const json& j) {
            return j.value("applied_id", 0ULL) >= j.value("accepted_id", 0ULL) &&
                   !j.value("running", true);
        },
        30s);
    CHECK(settled);

    const auto t = json::parse(gw.telemetry_json());
    CHECK(t.at("latched") == true);
    CHECK(t.at("state") == "held");          // a latch is a latch, not a fault
    CHECK(t.value("last_error", "").empty());  // the stop worked; nothing failed
}

// The wait is bounded by SILENCE, not by how long the work takes. A program that
// runs for half a minute must survive a stall bound of five seconds, because it
// is reporting progress the whole time — the alternative is a platform that
// calls a slow machine a failure.
void test_waiting_is_bounded_by_stall_not_duration() {
    auto p = platform(ROBONODE_APPS);
    const auto ack = json::parse(p.submit_command(R"({"cmd":"run_app","file":"bin-picking.app.json"})"));
    CHECK(ack.at("ok") == true);
    CHECK(p.await_settled(std::chrono::seconds{5}).ok());
}

// A run record belongs to the run that made it. Without an identity on the
// record, "the last run" is whatever ran most recently — and a comparison whose
// trial never moved reports the PREVIOUS version's numbers under this version's
// name, which reads exactly like evidence.
void test_run_records_are_identified() {
    auto gw = gateway();
    const auto before = json::parse(gw.last_run_json()).value("run", std::uint64_t{0});
    CHECK(gw.submit("run", {}).find("\"ok\":true") != std::string::npos);
    const bool recorded = poll_until(
        [&] { return gw.last_run_json(); },
        [&](const json& q) { return q.value("run", std::uint64_t{0}) > before; }, 30s);
    CHECK(recorded);
}

// Every live view the platform declares answers, and every one of them refuses
// a robot that is not there. The surfaces are generated from this table, so a
// view that works over HTTP but not in-process (or vice versa) would have to be
// a view that does not work here either.
void test_every_declared_view_answers_and_refuses_a_ghost() {
    auto p = platform();
    for (const auto& v : robonode::Platform::kViews) {
        const auto body = p.view_json(v.name);
        CHECK(body.has_value());
        CHECK(!json::parse(*body, nullptr, false).is_discarded());
        CHECK(!p.view_json(v.name, "ghost").has_value());
    }
    CHECK(!p.view_json("not-a-view").has_value());
}

// A run can be stopped: the command lands, the cell leaves "moving", and the
// motion reports that it ended before the goal.
void test_gateway_stop_cancels_a_running_motion() {
    auto p = platform();
    CHECK(p.run().ok());
    CHECK(poll_until([&] { return p.telemetry_json(); },
                     [](const json& j) { return j.value("state", "") == "moving"; }, 15s));
    CHECK(p.stop().ok());
    CHECK(poll_until([&] { return p.telemetry_json(); },
                     [](const json& j) { return j.value("state", "") != "moving"; }, 20s));
}

// E-stop is a LATCH: it holds the cell until resume, and refuses new motion in
// between. Software stop, not a safety function — that is hardware's job.
void test_gateway_estop_latches_until_resume() {
    auto p = platform();
    CHECK(p.estop().ok());
    CHECK(poll_until([&] { return p.telemetry_json(); },
                     [](const json& j) { return j.value("latched", false); }, 10s));

    CHECK(p.run().ok());  // accepted by the router...
    CHECK(poll_until([&] { return p.telemetry_json(); },
                     [](const json& j) {  // ...and refused by the supervisor
                         return j.value("latched", false) &&
                                j.value("last_error", "").find("latched") != std::string::npos;
                     },
                     10s));

    CHECK(p.resume().ok());
    CHECK(poll_until([&] { return p.telemetry_json(); },
                     [](const json& j) { return !j.value("latched", true); }, 10s));
}

// Jog: the OTG drives one axis to a target through the same governor and safety
// gate, addressed by axis id.
void test_gateway_jog_moves_one_axis_via_otg() {
    auto p = platform();
    CHECK(p.jog("rail-x", 250.0).ok());
    CHECK(p.await_settled(40s).ok());
    const auto j = json::parse(p.telemetry_json());
    CHECK(std::abs(double(j.at("pos")[0]) - 250.0) < 5.0);
    CHECK(!p.jog("no-such-axis", 1.0).ok() || !p.await_settled(10s).ok());
}

// Closing the gripper on nothing is not a grasp. Without this the tool reports
// success in mid-air and every downstream step believes it carries a part.
void test_gripper_refuses_to_grasp_thin_air() {
    auto p = platform();
    CHECK(p.grasp().ok());                    // accepted onto the queue...
    CHECK(!p.await_settled(10s).ok());        // ...and refused by the tool
    const auto j = json::parse(p.telemetry_json());
    CHECK(!j.value("holding", true));
    CHECK(j.value("last_error", "").find("grasp") != std::string::npos);
}

// Every path that accepts a command answers with an id, including the ones that
// resolve a store first — otherwise completion is unobservable on that path.
void test_store_backed_commands_acknowledge_with_an_id() {
    auto p = platform(ROBONODE_APPS);
    CHECK(p.run_app("pick-demo.app.json").ok());
    CHECK(p.last_ack().accepted && p.last_ack().id > 0);

    const auto wire = json::parse(
        p.submit_command(R"({"cmd":"run_app","file":"pick-demo.app.json"})"));
    CHECK(wire.at("ok") == true);
    CHECK(wire.contains("id") && wire.at("id") > 0);

    const auto bad = json::parse(p.submit_command(R"({"cmd":"run_app","file":"nope.json"})"));
    CHECK(bad.at("ok") == false);
    CHECK(bad.contains("error"));
}

// Changing the scenario is a command on a running platform: the user edits
// their own copy of a shipped scene, applies it, and the physics model — the
// thing every surface renders — is the one they described.
void test_scene_swap_rebuilds_the_running_world() {
    auto p = platform();
    const auto before = json::parse(p.model_json()).at("links");
    CHECK(std::none_of(before.begin(), before.end(),
                       [](const json& l) { return l.at("name") == "crate"; }));

    CHECK(p.apply_scene("cluttered-line.scene.json").ok());
    CHECK(p.await_settled().ok());
    const auto after = json::parse(p.model_json()).at("links");
    CHECK(std::any_of(after.begin(), after.end(),
                      [](const json& l) { return l.at("name") == "crate"; }));

    CHECK(!p.apply_scene("no-such.scene.json").ok());  // a scene nobody has is refused
    const auto still = json::parse(p.model_json()).at("links");
    CHECK(still.size() == after.size());  // and the running world is untouched
}

// Physics, not bookkeeping: the workpiece rests on the belt because contacts
// hold it there, a grasp carries it because a constraint closes on it, and
// letting go drops it. Every claim here passed silently when nothing in the
// world collided with anything.
void test_a_pick_moves_a_real_body() {
    auto p = platform(ROBONODE_APPS);
    // The world needs a tick or two before the part has settled onto the belt.
    const bool on_belt = poll_until(
        [&] { return p.telemetry_json(); },
        [](const json& j) {
            for (const auto& c : j.at("contacts")) {
                if (c.at(0) == "conveyor" && c.at(1) == "part") return true;
            }
            return false;
        },
        10s);
    CHECK(on_belt);
    const auto at_rest = json::parse(p.telemetry_json());
    const double x0 = at_rest.at("workpiece").at("pose")[0];

    CHECK(p.run_app("bin-picking.app.json").ok());
    CHECK(p.await_settled(kProgramTimeout).ok());
    const auto carried = json::parse(p.telemetry_json());
    CHECK(carried.at("workpiece").at("held") == true);
    const auto& held = carried.at("workpiece").at("pose");
    const auto& tcp = carried.at("tcp");
    // The part is where the tool is: it is welded to it, not tracked by us.
    CHECK(std::abs(double(held[0]) - double(tcp[0])) < 0.02);
    CHECK(std::abs(double(held[2]) - double(tcp[2])) < 0.02);
    CHECK(std::abs(double(held[0]) - x0) > 0.1);  // it was actually moved

    const double let_go_z = double(held[2]);
    CHECK(p.release().ok());
    CHECK(p.await_settled().ok());
    // Let go, it falls until something holds it up — and what holds it up is a
    // contact, not a rule about where parts belong.
    const bool landed = poll_until(
        [&] { return p.telemetry_json(); },
        [&](const json& j) {
            return double(j.at("workpiece").at("pose")[2]) < let_go_z - 0.02 &&
                   !j.at("contacts").empty() && j.at("workpiece").at("held") == false;
        },
        10s);
    CHECK(landed);
}

// 6-DoF (#92): a tool that must arrive at an ANGLE can say so. The pose planner
// is a capability version like any other, so this is a swap plus a verb — and
// the planner that cannot solve orientation refuses by name rather than
// quietly reaching the point with the wrist wherever it landed.
void test_move_pose_reaches_a_full_pose() {
    auto p = platform(ROBONODE_APPS);

    // The position-only planner is honest about what it was handed.
    CHECK(p.set_version("planner", "robonode.moveL").ok());
    CHECK(p.await_settled().ok());
    (void)p.move_pose(0.85, 0.25, 0.55, 0.0, 1.0, 0.0, 0.0);
    const auto refused = p.await_settled();
    CHECK(!refused.ok());
    CHECK(refused.message().find("Cartesian pose") != std::string::npos);
    // A refusal faults the cell but does not LATCH it: the next command runs.

    CHECK(p.set_version("planner", "robonode.moveP").ok());
    CHECK(p.set_version("control", "robonode.direct").ok());
    CHECK(p.move_pose(0.85, 0.25, 0.55, 0.0, 1.0, 0.0, 0.0).ok());
    CHECK(p.await_settled(kProgramTimeout).ok());

    const auto j = json::parse(p.telemetry_json());
    const auto tcp = j.at("tcp");
    CHECK(std::abs(tcp[0].get<double>() - 0.85) < 0.02);
    CHECK(std::abs(tcp[1].get<double>() - 0.25) < 0.02);
    CHECK(std::abs(tcp[2].get<double>() - 0.55) < 0.02);
}

// A scene swap builds a NEW world, and the planner holds a reference to the
// kinematics of the old one. Moving after a swap used to dereference a
// destroyed object — the crash this test exists to keep dead.
void test_a_cartesian_move_survives_a_scene_swap() {
    auto p = platform(ROBONODE_APPS);
    CHECK(p.apply_scene("stacking-line.scene.json").ok());
    CHECK(p.await_settled(kProgramTimeout).ok());
    CHECK(p.move_l(0.9, 0.25, 0.5).ok());
    CHECK(p.await_settled(kProgramTimeout).ok());

    const auto j = json::parse(p.telemetry_json());
    CHECK(std::abs(j.at("tcp")[0].get<double>() - 0.9) < 0.05);

    // The fuller path: a PROGRAM after a swap uses vision, the planner and the
    // tool, each holding something the swap rebuilt.
    CHECK(p.run_app("pick-demo.app.json").ok());
    CHECK(p.await_settled(kProgramTimeout).ok());
}

// A person authoring a pose step has a drawing with angles on it, not a
// quaternion. Both forms are accepted and both reach the same pose.
void test_a_pose_step_can_be_written_in_degrees() {
    auto p = platform(ROBONODE_APPS);
    CHECK(p.set_version("planner", "robonode.moveP").ok());
    CHECK(p.await_settled(kProgramTimeout).ok());

    const std::string program = R"({"cmd":"run_app","program":[
        {"verb":"move_pose","args":{"x":"0.85","y":"0.25","z":"0.55","roll_deg":"180"}}]})";
    const auto ack = json::parse(p.submit_command(program));
    CHECK(ack.at("ok") == true);
    CHECK(p.await_settled(kProgramTimeout).ok());

    const auto j = json::parse(p.telemetry_json());
    CHECK(std::abs(j.at("tcp")[0].get<double>() - 0.85) < 0.03);
    // 180° of roll is (0, 1, 0, 0) as a quaternion, which is what it stored.
    const auto q = j.at("tcp_quat").get<std::vector<double>>();
    CHECK(std::abs(std::abs(q[1]) - 1.0) < 0.05);
}

// Every document the platform SHIPS must be one it can run. A saved app is
// checked when it is written; a shipped one never passed through that path, so
// a verb removed from the runner would break the library silently.
void test_every_shipped_document_is_runnable() {
    auto p = platform(ROBONODE_APPS);
    const auto verbs = json::parse(p.verbs_json());
    std::set<std::string> known;
    for (const auto& v : verbs) known.insert(v.at("verb").get<std::string>());

    for (const auto& entry : json::parse(p.docs_json("apps"))) {
        const auto file = entry.at("file").get<std::string>();
        const auto body = p.doc("apps", file);
        CHECK(body.has_value());
        const auto app = json::parse(*body, nullptr, false);
        CHECK(!app.is_discarded());
        for (const auto& step : app.at("program")) {
            const auto verb = step.value("verb", "");
            if (known.count(verb) == 0) std::printf("  %s: unknown verb '%s'\n", file.c_str(), verb.c_str());
            CHECK(known.count(verb) == 1);
        }
    }

    // And every shipped scene still parses as one.
    for (const auto& entry : json::parse(p.docs_json("scenes"))) {
        const auto file = entry.at("file").get<std::string>();
        const auto body = p.doc("scenes", file);
        CHECK(body.has_value());
        robonode::SceneDescriptor scene;
        CHECK(robonode::parse_scene_descriptor(json::parse(*body, nullptr, false), scene, file).ok());
    }
}

// The log is the operator's account of a run: numbered steps, the tool's state
// changes, and the real-time quality of each move.
void test_log_records_steps_tool_events_and_rt_quality() {
    auto p = platform(ROBONODE_APPS);
    CHECK(p.run_app("bin-picking.app.json").ok());
    CHECK(p.await_settled(180s).ok());

    const auto logs = json::parse(p.logs_json());
    bool numbered = false, tool = false, quality = false, correlated = false;
    for (const auto& line : logs) {
        const auto text = line.get<std::string>();
        numbered |= text.find("app step 1/") != std::string::npos;
        tool |= text.find("tool: grasped") != std::string::npos;
        quality |= text.find("jitter p99") != std::string::npos;
        correlated |= text.find("#1 run_app ok") != std::string::npos;
    }
    CHECK(numbered && tool && quality && correlated);
}


}  // namespace

int main() {
    test_platform_facade_typed_verbs();
    test_every_declared_view_answers_and_refuses_a_ghost();
    test_run_records_are_identified();
    test_waiting_is_bounded_by_stall_not_duration();
    test_an_estop_empties_the_queue_and_is_not_a_failure();
    test_a_linked_vendor_driver_is_offered_as_a_version();
    test_one_definition_of_done();
    test_gateway_boots_seven_nodes_with_driver_versions();
    test_gateway_run_moves_the_cell_and_streams_telemetry();
    test_gateway_swaps_one_node_driver_live();
    test_gateway_swap_clock_owner_keeps_physics_alive();
    test_gateway_byo_example_driver_selectable_and_drives();
    test_gateway_logs_surface_records_events();
    test_gateway_cartesian_move_reaches_target();
    test_gateway_bin_picking_app_places_part();
    test_gateway_palletize_uses_station_slots();
    test_gateway_rejects_bad_commands();
    test_gripper_refuses_to_grasp_thin_air();
    test_store_backed_commands_acknowledge_with_an_id();
    test_scene_swap_rebuilds_the_running_world();
    test_a_pick_moves_a_real_body();
    test_move_pose_reaches_a_full_pose();
    test_a_cartesian_move_survives_a_scene_swap();
    test_a_pose_step_can_be_written_in_degrees();
    test_every_shipped_document_is_runnable();
    test_log_records_steps_tool_events_and_rt_quality();
    test_gateway_stop_cancels_a_running_motion();
    test_gateway_estop_latches_until_resume();
    test_gateway_jog_moves_one_axis_via_otg();
    std::puts("robonode gateway integration: all tests passed");
    return 0;
}
