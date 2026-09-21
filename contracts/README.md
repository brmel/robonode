# Contracts

The wire contract the platform actually serves today is **JSON over HTTP+SSE**,
and it is defined here as JSON Schema. The schemas are executable: a test
validates live payloads against them, so "one contract, many surfaces" (ADR-8)
is checked rather than asserted.

| Schema | Endpoint |
|---|---|
| `telemetry.schema.json` | `GET /telemetry`, SSE default event |
| `nodes.schema.json` | `GET /nodes`, SSE `nodes` event |
| `capabilities.schema.json` | `GET /capabilities`, `GET /capabilities/{id}` |
| `command-ack.schema.json` | `POST /command` response |
| `model.schema.json` | `GET /model` — the kinematic chain the physics runs |
| `scene.schema.json` | `GET /scenes/{file}`, `PUT /scenes/{file}` — the scenario a user authors |

`robonode-idl/` holds the **protobuf** definitions for the future gRPC surface
. Nothing generates from them yet, so they are a design document, not a
contract — when the SDK lands they become the generated source of truth and
these schemas become the JSON projection of the same types.
