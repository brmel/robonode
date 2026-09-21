# Topic key space (Zenoh)

> **Status: design, not a live contract.** Nothing publishes these topics yet —
> the Zenoh data plane is [#7](https://github.com/brmel/robonode/issues/7). What
> the platform serves today is JSON over HTTP+SSE, defined in
> [`contracts/`](../contracts/).

Scheme: `rn/<cell>/<node>/<capability>/<kind>`

| Kind | Direction | Content | Rate |
|---|---|---|---|
| `state` | node → | `NodeDescriptor` deltas (lifecycle, safety) | on change |
| `telemetry` | node → | capability telemetry (e.g. `AxisTelemetry`) | capability-defined, ≤ native rate |
| `cmd` | → node | capability commands (acked) | client-driven |
| `evt` | node → | faults, incidents, command completions | on occurrence |

Examples:

```
rn/mtl-line1-c3/rail-x/MotionAxis/telemetry
rn/mtl-line1-c3/ur10e-1/ArmKinematics/state
rn/mtl-line1-c3/**                            # cell-wide wildcard (UI, recorder)
```

Rules:

1. `<cell>`, `<node>` are lowercase kebab ids; capability names match descriptor `type` minus version (`MotionAxis`, not `MotionAxis@1` — version travels in payload).
2. Flight recorder subscribes `rn/<cell>/**` at native rates (FR-6.2); cloud sync applies per-stream downsampling policies (FR-6.4).
3. Descriptors and params are Zenoh queryables at the same keys with kind `descriptor` / `params` — pull, not push.
4. Large payloads (camera frames) publish shared-memory references on-cell; bytes never traverse the router twice (SPEC §5).
