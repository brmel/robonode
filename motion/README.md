# robonode::motion

RT-domain motion module ([SPEC §3](../docs/SPEC.md)): plans → executive → **governor** → **AxisAdapter seam**, with the sim adapter proving invariant I5 (twin and real behind one interface).

```
MotionPlan / SyncBlendPlan (trajlib-backed — OTG slot placeholder)
   └─> Executive / SyncExecutive (absolute deadlines, per-cycle safety gate)
         └─> Governor (descriptor limits: position clamp + velocity rate-limit, NaN-proof)
               └─> AxisAdapter (SimAxis · UrWristAdapter · EtherCAT next)
```

Module rules (enforced by [scripts/check-boundaries.sh](../scripts/check-boundaries.sh)):

- Public surface speaks **core types only** (`robonode::State`, `AxisLimits`, `MotionProfile`, `TelemetryRow`); trajlib is an internal detail nothing else may include.
- Lifecycle verbs (`configure/activate/deactivate` → `Status`) are the non-RT surface; `write_setpoint/step/read` are the RT surface — noexcept, allocation-free, faults latch into `AxisState.safety`.

What is placeholder: profile sampling instead of retargetable OTG (Ruckig, per STACK.md), host scheduler timing (PREEMPT_RT rig later — NFR-1).

Build from repo root: `cmake -B build && cmake --build build && ctest --test-dir build`.
