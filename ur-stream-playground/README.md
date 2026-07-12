# ur-stream-playground

Stream self-generated jerk-limited trajectories to a (simulated) Universal Robots
arm using the official [ur_client_library](https://github.com/UniversalRobots/Universal_Robots_Client_Library)
— the exact "robot streaming integration" named in industrial motion-control
job descriptions (UR client library, servoj-style setpoint streaming @ 500 Hz).

**Status: scaffold — builds against ur_client_library; streaming loop untested
until URSim is up. Follow the steps below in order.**

## 1. Run URSim (UR's free controller simulator) in Docker

```sh
docker run --rm -it \
  -p 5900:5900 -p 6080:6080 \
  -p 29999:29999 -p 30001-30004:30001-30004 \
  --name ursim universalrobots/ursim_e-series
```

Open the simulated teach pendant at http://localhost:6080/vnc.html
(power on the robot, release brakes — same buttons as a real UR).

For external control you also need the **External Control URCap** installed in
URSim and a program containing the ExternalControl node running — see the
ur_client_library [docs](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/doc/ur_client_library/doc/index.html).
The official image ships with it preinstalled on recent tags.

## 2. Build

```sh
cmake -B build && cmake --build build -j
```

ur_client_library is pulled via FetchContent — no system install needed.

## 3. Milestones (do them in order, each is a talking point)

- [ ] **M1 — read state:** connect RTDE @ 500 Hz, print joint positions/velocities.
      Proves: network plumbing, RTDE handshake.
- [ ] **M2 — stream a move:** generate a jerk-limited profile for joint 5
      (wrist — safe), stream via the library's trajectory/servoj interface,
      plot commanded vs RTDE-reported position. The lag you see = the
      controller's internal lookahead smoothing.
- [ ] **M3 — blend two waypoints:** stream your own blended corner, compare
      against UR's native movej blend radius.
- [ ] **M4 — abuse it:** drop every 10th setpoint, add 5 ms jitter — watch how
      the controller copes. THIS is the interview-story milestone: what does
      the receiving side tolerate, what faults, what should the sender
      guarantee.

## Layout

- `src/read_state.cpp` — M1, RTDE state reader
- `src/stream_joint.cpp` — M2 skeleton, profile → setpoint stream
- profiles come from ../trajectory-lab (header-only, included via CMake)
