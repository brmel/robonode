# Examples

Runnable demonstrations of a single seam. **Not product surface** — nothing here
is linked into `cell_server` or the `robonode` CLI, and nothing depends on them.

| Example | Shows | Build flag |
|---|---|---|
| [rtb_dev/](rtb_dev/) | A second `Kinematics` implementation: a real UR10 model from the Robotics Toolbox, over the `services/rtb-kinematics` sidecar. Proves the seam is engine-agnostic | `-DROBONODE_BUILD_RTB=ON` |
| [ur_governed_move/](ur_governed_move/) | A real UR joint driven through the same `AxisAdapter`, governor and executive the sim uses, recorded to MCAP | `-DROBONODE_BUILD_UR_ADAPTER=ON` (default) |

The product path for a real robot is different and does not need these: the app
registers vendor drivers (`robonode::vendor_drivers`), and a node is pointed at
hardware with a live driver swap — see [docs/REAL-ROBOTS.md](../docs/REAL-ROBOTS.md).
