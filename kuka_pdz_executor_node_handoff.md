# KUKA pdz plumbers_block — status & quick-start (2026-08-25)

This is now the single status file for this effort — the various per-session handoff files
this superseded (controller/actuation, gripper mount, gazebo regen, collision tuning, the
original gazebo handoff, and the abandoned Grasp_Planning-fallback plan) were deleted once
their content was either done+confirmed or folded in below.

## TL;DR

- **The arm/gripper motion plan itself is confirmed correct.** `plan_executor_node.py`, replaying
  Fabrica's own `logs/plumbers_block_sim/{motion.pkl,traj.npy}`, ran all 43 plan entries
  end-to-end on the real dual-arm-KUKA + pdz gripper Gazebo topology with no errors, at
  near-real-time speed, in a gravity-off / fixture-removed diagnostic scene — user watched it
  live and confirmed reach/timing/pick-place sequencing all look right.
- **A full physically-realistic run (real gravity + fixture collision) is not yet practical**:
  real-time factor collapses to ~0.028 (36x slower than real-time) once the fixture's real-mesh
  collision is back in the scene. Confirmed numerically this session (was previously suspected
  but never measured). See Open issue #1.
- Two real code bugs in `plan_executor_node.py` were found and fixed this session (see below) —
  the node had apparently never been run end-to-end before this.

## Quick-start: fast motion-plan-only check (gravity off, no fixture)

This is what was verified today. Reproduces quickly (near-real-time) but does **not** validate
real grasp physics (parts don't fall, and nothing makes them track the gripper — see Open issue
#3), only arm/gripper motion.

```bash
export GZ_IP=127.0.0.1 IGN_IP=127.0.0.1
ros2 launch lbr_dual_arm_pdz_bringup gazebo.launch.py \
  robot_name:=lbr_dual_arm_pdz gripper:=pdz world:=plumbers_block
# wait for "Configured and activated" x3 (joint_state_broadcaster + both JTCs)

python3 ~/franka_ros2_ws/src/lbr_fri_ros2_stack/lbr_demos/lbr_dual_arm/lbr_dual_arm_pdz_bringup/\
lbr_dual_arm_pdz_bringup/plan_executor_node.py
# no args needed -- defaults to ~/Fabrica/logs/plumbers_block_sim
```

Requires the two temp diagnostic changes below to still be in place (they are, as of this
writing).

## Quick-start: full physically-real run (fixture + gravity)

Same commands as above, but first revert the two temp diagnostic changes. **Not recommended yet**
— will hit the RTF problem (Open issue #1) and take a very long time.

## Temporary diagnostic changes (uncommitted, in `~/franka_ros2_ws`) — revert before a real run

- `lbr_dual_arm_pdz_bringup/worlds/plumbers_block.sdf`: added `<gravity>0 0 0</gravity>`.
- `lbr_dual_arm_pdz_bringup/launch/gazebo.launch.py`: spawn loop now has an `if model_name ==
  "plumbers_block_fixture": continue` skip.

Both are marked inline with `TEMP diagnostic (2026-08-25)` comments — search for that string to
find and revert both when ready to test with real physics.

## Fixed this session (`plan_executor_node.py`, uncommitted ROS workspace)

- **`use_sim_time` double-declared**: the node called `self.declare_parameter('use_sim_time',
  ...)`, but rclpy's `Node.__init__` already auto-declares it — this crashed immediately with
  `ParameterAlreadyDeclaredException` the first time the node was ever run. Fixed: just
  `self.set_parameters([Parameter('use_sim_time', value=True)])`, no explicit declare.
- **`self._clients` name collision**: the node stored its two per-arm `ActionClient`s in
  `self._clients = {}`, which shadows rclpy's own internal `Node._clients` list (used by
  `create_client`) — crashed with `AttributeError: 'dict' object has no attribute 'append'` the
  moment the node tried to create the `SetEntityPose` client. Renamed to `self._action_clients`
  throughout.
- (From a prior session, still in place, unaffected by the above:) `GRIPPER_OPEN_RATIO_TO_M`
  corrected from an unverified `0.04` to the confirmed-real `0.032`.

## Open issues for the future

1. **RTF collapses to ~0.028 with the fixture's real-mesh collision in the scene** (vs. ~0.99
   measured without it, both today). This is the actual blocker for a full physically-real run.
   Root cause not isolated — likely narrowphase cost of mesh-vs-convex_decomposition contact
   (base/top parts against the fixture's raw mesh), worsened by ODE's own "trimesh-trimesh
   contact hash table bucket overflow" warnings seen flooding the log on launch. Things to try:
   reduce `max_convex_hulls` on base/top in `generate_plumbers_block_gz_assets.py`, try a
   decimated/simplified fixture mesh (**note**: keeping the fixture as real mesh, not a box, was
   an explicit firm user requirement in the prior collision-tuning session — don't silently
   revert to a box, ask first if considering it), or profile which specific contact pair
   dominates the per-step cost.
2. **`screw_b` settles ~5cm from its intended pickup pose** in the real (fixture-on, gravity-on)
   scene — unresolved, carried over from prior sessions. `screw_a` (its mirror across the
   fixture's centerline) settles correctly with identical collision/mass/friction config, so this
   points at an asymmetry in the fixture mesh near screw_b's slot specifically, not a
   config/collision-shape-choice bug. Not re-investigated this session. A vertex-proximity check
   against the fixture mesh was attempted once before and aborted with a frame-mismatch bug in
   the check itself (not a real finding) — redo it properly rather than reusing that code.
3. **Held-part visual following is unavailable**: `/world/plumbers_block/set_pose` (the
   `SetEntityPose` service `plan_executor_node.py` uses to teleport held parts to match
   `traj.npy`, since there's no reliable Gazebo "stick to gripper" mechanism) is not being
   advertised in this world. Cause not investigated — likely a missing `ros_gz_bridge` service
   config, or a service-name/world-name mismatch. Until fixed, held parts won't visually follow
   the gripper even after issue #1 is resolved.
4. Once #1 and #3 are fixed, re-run the full plan with gravity+fixture restored (reverting the
   temp diagnostic changes) and get a final visual confirmation against
   `records/blender/plumbers_block_sim/plumbers_block.mp4`'s last frame — that's the original
   deliverable of this whole multi-session effort.

## Environment gotchas (carried forward)

- Must `export GZ_IP=127.0.0.1 IGN_IP=127.0.0.1` before any `ros2 launch` that starts gz-sim —
  the launch file doesn't set these itself.
- Process cleanup: `SIGINT` (not `SIGKILL`) to the `ros2 launch` parent PID, wait ~5s, then
  verify no orphaned `ign gazebo`/`gzserver`/`gzclient` via
  `ps -eo pid,ppid,cmd | grep -E "lbr_dual_arm_pdz_bringup|ign gazebo"`.
- This machine sometimes has a **remote real KUKA control stack** (`/lbr`, `/lbr_one`,
  `/lbr_two` nodes — move_group, servo_node, controller_manager, gripper controllers) visible
  over the network via ROS2 DDS discovery (`ROS_DOMAIN_ID=0`, `ROS_LOCALHOST_ONLY=0`). Confirmed
  today these are **not local processes** (no matching PID on this host — `ros2 node list`
  still showed them after a `ros2 daemon` restart, and they vanished only once the network was
  physically disconnected). If controller loading looks flaky (spawners failing/timing out with
  `Failed getting a result from calling .../load_controller`), check for these nodes first — it's
  DDS cross-talk with the remote stack, not a local bug. Disconnecting the network fixed it
  today; using a distinct `ROS_DOMAIN_ID` for the sim would be the alternative if staying
  networked.
- If `plan_executor_node.py` looks "frozen" mid-goal, check real-time factor first
  (`ign topic -e -t /world/plumbers_block/stats -n 1`) before assuming a control bug — a manual
  single-joint `FollowJointTrajectory` test goal this session showed genuine, correct closed-loop
  tracking even while RTF was ~0.028, it was just very slow, not stuck.
- Screenshot procedure (if needed again): `xwd -id <window-id>` can capture stale content if the
  gz-sim window is in WM `Hidden` state — send an EWMH `_NET_WM_STATE` remove-`Hidden` +
  `_NET_ACTIVE_WINDOW` client message via `python3-xlib` first. Use `/gui/move_to/pose`
  (`ignition.msgs.GUICamera`) to frame a specific area; `/gui/screenshot` returns success but the
  file can't be reliably located, don't use it.

## What's confirmed working (don't re-litigate)

- Controller topology/actuation (one `gz_ros2_control` plugin instance, two per-arm
  `JointTrajectoryController`s), pdz gripper mount geometry, fixture/part-alignment planning math
  (`planning/robot/{geometry,util_arm,workcell}.py`), and the gripper open-ratio formula (0.032)
  — all fixed and user-confirmed in prior sessions.
- Arm/gripper motion plan itself (path, timing, reach, pick/place sequencing across both arms and
  all 5 parts) — confirmed correct today via the gravity-off/no-fixture diagnostic run, by direct
  user visual confirmation.

## Key file locations

- `~/franka_ros2_ws/src/lbr_fri_ros2_stack/lbr_demos/lbr_dual_arm/lbr_dual_arm_pdz_bringup/` —
  the node, launch file, and world SDF (own git state, uncommitted, not a Fabrica-tracked repo).
- `~/Fabrica/logs/plumbers_block_sim/{motion.pkl,traj.npy}` — the plan data, already correct, no
  action needed.
- `~/Fabrica/planning/utils/generate_plumbers_block_gz_assets.py` — regenerates
  `~/franka_ros2_ws/src/plumbers_block_description/`; its module docstring documents the
  per-part collision-shape choices and the screw_b issue in more code-level detail than this file.
- `~/Fabrica/records/blender/plumbers_block_sim/plumbers_block.mp4` — the "correct" reference for
  the eventual full physically-real run's final-state comparison.
