# Journey Audit — 2026-07-12

Every runnable entry point traced to its deepest calls; each layer checked for input/output handling, safety, performance, state, and types. Findings numbered F1–F14; every finding fixed in the same change unless marked *deferred*.

## Journey 1 — `robonode_dev` (dev-twin demo, motion-core)

**Call tree**

```
main (robonode_dev.cpp)
├─ SimAxis{tau=5ms} · Governor{descriptor limits} · Executive{1 kHz}
├─ MotionPlan::scurve/trapezoid ──► trajlib::SCurveProfile / TrapezoidalProfile (ctor case tree)
├─ Executive::execute
│   └─ per cycle: sleep_until(absolute deadline) → adapter.read [safety gate]
│       → plan.sample(t) → Governor::apply → adapter.write_setpoint
│       → adapter.step (plant model) → adapter.read → rows.push_back
└─ write_csv → report
```

| # | Layer | Finding | Class | Status |
|---|---|---|---|---|
| F1 | Governor | NaN/Inf setpoint sailed through `std::clamp` and poisoned `prev_mm_`, the adapter, and the plant — a buggy plan or future Tier C plugin could drive a real axis with NaN | **safety** | fixed: non-finite or `dt≤0` ⇒ hold previous governed position, `rejected_setpoints` counter; test added |
| F2 | Executive | `AxisState.safety` was read but never consulted — protective-stop/e-stop observation (FR-8.2, SPEC I4) did not exist even as a stub | **safety** | fixed: per-cycle gate before commanding; non-NORMAL ⇒ hold last governed setpoint, `safety_hold_cycles` stat; `SimAxis::set_safety` fault-injection hook; mid-move trip/recover test added (hold frozen, catch-up rate-limited, move completes) |
| F3 | Executive | `plan.duration_s()` (variant visit + 7-segment sum) evaluated every cycle | perf | fixed: hoisted before loop |
| F4 | write_csv | `fopen` failure silently dropped telemetry | I/O | fixed: stderr warning |
| F5 | Governor | `dt_s ≤ 0` produced `dp_max = 0` and clamped everything with no signal | input | fixed: folded into F1 rejection path |

State machine note: executive still runs a single plan to completion — full motion-state machine (Idle/Jogging/…/EStop, FR-2.7) is M1 scope, tracked in SPEC §3.1. Types: counters `uint64_t`, cycle math `double`-cast explicitly; no narrowing found.

## Journey 2 — `motion_core_tests`

Previously 3 CHECK-based tests; now 5 (F1, F2 coverage added). Earlier audit round had already caught: Release `NDEBUG` made `assert`-based tests vacuous (replaced by always-on `CHECK`), and the governor's strict `>` rate check false-clamping at exact `v_max` cruise (float ulps) — both fixed and retained.

## Journey 3 — `profile_demo` → `plot_profiles.py` (trajectory-lab)

```
main → TrapezoidalProfile + SCurveProfile ctors → sample @500 Hz → CSV stdout
plot_profiles.py → pandas.read_csv → matplotlib → PNG
```

| # | Finding | Class | Status |
|---|---|---|---|
| F6 | `plot_profiles.py` crashed with bare `IndexError` when run without args | input | fixed: usage exit |
| F7 | Lint noise (E501 long lines; E402 from the *required* `matplotlib.use("Agg")`-before-pyplot order) | hygiene | fixed: wrapped lines, `noqa: E402` with reason comment |

`profile_demo` itself clean (float `t += dt` accumulation acceptable at demo scale; `d=0`/triangular degeneracies handled in both profile ctors, covered by Catch2 suite — 100 % pass re-verified).

## Journey 4 — `stream_demo` (trajectory-lab, producer/consumer streaming)

```
main → producer thread [SCHED_FIFO attempt → sleep_until → profile.sample → SpscRing::push]
     → consumer loop [pop → vector] → join → report
```

| # | Finding | Class | Status |
|---|---|---|---|
| F8 | `received.back()` on a potentially empty vector = UB | correctness | fixed: empty guard, error exit |
| F9 | `std::atomic<long>` for µs jitter — 32-bit on LLP64 (Windows) while `duration_cast<microseconds>().count()` is `int64` | types | fixed: `std::atomic<std::int64_t>` + cast in printf |

`SpscRing` verified correct: single-writer-per-index, release-store publish / acquire-load consume, one-slot-sacrifice full/empty discrimination, `alignas(64)` split of head/tail; TSan job covers it in CI. Jitter max tracked by producer only ⇒ no lost-update race. Re-run: 1176 setpoints @500 Hz, 0 overruns.

## Journey 5 — `read_state` / `stream_joint` (ur-stream-playground)

```
read_state:   main → RTDEClient{recipes} → init/start → getDataPackage(100 ms) → getData("actual_q") → print
stream_joint: main → UrDriver{scripts+recipes} → getDataPackage → SCurveProfile plan
              → 500 Hz loop: sleep_until → writeJointCommand(SERVOJ) → stopControl
```

| # | Finding | Class | Status |
|---|---|---|---|
| F10 | `stream_joint` streamed servoj setpoints with **zero safety-mode monitoring** — would keep commanding a robot in protective stop/e-stop | **safety** | fixed: per-cycle `safety_mode` gate (NORMAL/REDUCED pass; missing field fails closed), abort + `stopControl()` |
| F11 | `writeJointCommand` and `getData` return values ignored — lost connection ⇒ silent no-op loop | I/O | fixed: checked, abort with reason |
| F12 | `read_state` requested `safety_mode`/`robot_mode` in the recipe but never used them | I/O | fixed: safety transitions printed; `getData` checked |
| F13 | **Whole component never compiled**: pinned `ur_client_library 1.6.0` uses Linux-only `<endian.h>` → fatal on macOS | correctness/build | fixed: bumped to 2.13.0 (portable socket layer); in-tree alias is `ur_client_library::urcl` not `urcl::urcl` (link names fixed); both executables now build clean |
| F14b | Allocating `getDataPackage()` (deprecated, removed May 2027) used per 500 Hz cycle — violates the no-alloc-in-loop rule this codebase preaches | perf/API | fixed: pre-allocated `DataPackage{recipe}` reused via the bool overloads; zero warnings |

Remaining known gap (*deferred*, documented in file header): UrDriver resource paths are TODO until the M1 adapter wires them; runtime validation needs URSim.

## Journey 6 — CI workflow

| # | Finding | Class | Status |
|---|---|---|---|
| F14 | Workflow lived at `trajectory-lab/.github/workflows/` — GitHub only reads the **repo root**, so CI was entirely dead | workflow | fixed: root `.github/workflows/ci.yml` building+testing trajectory-lab **and** motion-core (Release, ASan/UBSan matrix, TSan for the ring); nested copy deleted; ur-stream-playground excluded with reason comment |

## Cross-cutting review

- **Types**: mm/s/µs unit suffixes consistent across IDL and code; explicit casts at printf boundaries; no implicit narrowing found in any journey.
- **State**: node lifecycle + motion states specified (SPEC §2.3/§3.1) but only partially implemented — M1 scope, not silently missing.
- **Performance**: RT loop allocation-free after reserve; O(1) profile sampling; remaining known cost is `sleep_until` scheduler jitter on macOS (mean ~340 µs), which is the documented host-scheduler reality until the PREEMPT_RT rig (NFR-1).
- **Docs**: stale root-README "next actions" (git init, unresolved OQs) refreshed to point at M1 + this audit.

## Verification matrix

| Component | Build | Tests | Demo run |
|---|---|---|---|
| motion-core | ✅ | ✅ 5/5 | ✅ both demos, hold/recover exercised in tests |
| trajectory-lab | ✅ | ✅ Catch2 suite | ✅ stream_demo 0 overruns |
| ur-stream-playground | ✅ builds clean vs urcl 2.13.0, zero warnings (runtime needs URSim — M1) | n/a | n/a |
| CI | root workflow added (runs on next push to GitHub) | — | — |
