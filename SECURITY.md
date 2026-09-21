# Security

## Reporting a vulnerability

Please report privately through GitHub's **Report a vulnerability** button
(Security → Advisories) rather than opening a public issue. Include what you ran
and what you observed; a proof of concept is welcome but not required. We aim to
acknowledge within a week.

## What this software is

RoboNode runs a physics simulation, a web/HTTP surface, and — deliberately —
**code that its users write**. That last part is the whole point of the platform
and also the centre of its threat model.

## Running user code

A user-authored capability version ("bring your own algorithm") is compiled and
run in a sandbox:

- **Default: an expression VM.** No I/O, no syscalls, no host access, fuel
  limited, and every output validated (arity, finiteness, and — for
  trajectories — workspace bounds) before the platform acts on it. A program
  that returns nonsense is refused; it cannot reach the robot.
- **Opt-in: Wasmtime**, for richer user modules. Wasm is a real sandbox but a
  larger attack surface than an expression evaluator. Enable it knowingly.
- **Never on the 1 kHz path.** ADR-5 excludes in-loop control from user code, so
  a user module cannot stall or destabilise the real-time loop. It shapes
  trajectories, which the host validates, and the control law selects among
  built-in versions.

If you extend the sandbox, the invariant to preserve is: **the host validates
what comes out**, and a module that misbehaves produces a reported failure, not
a plausible-looking wrong answer.

## Exposing an instance to the internet

The server ships **no authentication and no authorisation**, because the unit of
deployment is "one cell, one operator". Everything below assumes you are aware
of that.

If you put an instance on a public address:

- Set `ROBONODE_PUBLIC=1`. It refuses the destructive endpoints — deleting a
  session and adding or removing cells — so a visitor can fork a scene, author
  an algorithm and run it, but cannot delete someone else's work or exhaust the
  machine by spawning cells.
- Put it behind a reverse proxy that terminates TLS and rate-limits. The
  physics loop is CPU-bound; a stream of `run_app` requests is a denial of
  service on any host.
- Expect one cell per instance. Sessions separate *documents*, not physics: two
  visitors share the running robot.
- Treat the filesystem as untrusted-writable: sessions, scenes and modules are
  files that any visitor can create. Mount that directory with a quota.
- Do not run it as root, and do not mount a host path you care about into the
  container.

## Real hardware

Driving a real robot (`-DROBONODE_BUILD_UR_ADAPTER=ON`, a node pointed at a
`robot_ip`) removes the simulation as a safety net. The software stop, the
e-stop verb and the latched state are **software** functions (ADR-15): they
cancel motion and refuse new commands, and they are not, and must never be
presented as, a safety function. A real cell needs a certified hardware E-stop,
guarding, and a risk assessment. Never expose an instance that can command real
hardware to an untrusted network.

## Supported versions

The latest release on `main`. Security fixes land there first; older tags are
not maintained.
