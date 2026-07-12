# robonode-idl

Source of truth for every contract in the platform: capability interfaces, node descriptors, topic naming. Everything north of the motion core (SDKs, UI, Tier B `CellClient`, cloud APIs) is generated from these files; the motion core implements them natively.

## Layout

- `proto/robonode/v0/` — protobuf definitions (wire types + gRPC services)
  - `common.proto` — shared enums/types: lifecycle, safety, stop kinds, motion profile, limits
  - `descriptor.proto` — node & capability descriptors (limits are **data**, FR-1.3)
  - `motion_axis.proto` — `MotionAxis@1` commands, telemetry, streaming
  - `cell_client.proto` — Tier B algorithm surface (subscribe / command / setpoint streams)
- `topics.md` — Zenoh key-space naming scheme
- `examples/` — canonical descriptor instances used in docs and tests

## Rules

1. Versioned packages (`robonode.v0`); breaking change ⇒ new package version, never in-place edits after a tagged release.
2. Every field documented; units in field names where ambiguity is possible (`_mm`, `_ms`, `_hz`) — SI-ish shop-floor units: mm, mm/s, mm/s², mm/s³, N, s.
3. Capability semver (`MotionAxis@1`) is carried in descriptors, not in proto package names; additive fields don't bump capability version.
4. CI (later): `buf lint` + `buf breaking` against last tag.

Spec context: [docs/SPEC.md §2](../docs/SPEC.md).
