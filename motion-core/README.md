# motion-core (M0 skeleton)

RT-domain slice of the cell controller ([SPEC §3](../docs/SPEC.md)): fixed-rate executive → plan sampling → **governor** → **vendor adapter**, with the sim adapter proving invariant I5 (twin and real behind one interface).

```
MotionPlan (trajlib profiles — OTG slot placeholder)
   └─> Executive @1 kHz (absolute deadlines, RT-clean loop body)
         └─> Governor (descriptor limits: position clamp + velocity rate-limit)
               └─> AxisAdapter (SimAxis today; EtherCAT CiA402 / UR next)
```

What is real already: descriptor limits as data, absolute setpoints, following-error telemetry, jitter stats (max/p99 — max is what matters), governor-below-everything.
What is placeholder: profile sampling instead of retargetable OTG (Ruckig, M1), single axis (motion tree M1), host scheduler (PREEMPT_RT rig later — NFR-1).

## Build & run

```sh
cmake -B build && cmake --build build
./build/motion_core_tests     # or: ctest --test-dir build
./build/robonode_dev          # demo 1: S-curve 0→500 mm; demo 2: governor holds 1450 mm envelope
```

Outputs `telemetry-move.csv` / `telemetry-governed.csv` (gitignored) — plot with `python3 ../trajectory-lab/scripts/plot_profiles.py` style tooling or any CSV viewer.
