# KUKA dual-arm Gazebo — Cartesian-impedance plan replay — quick-start

Same Gazebo bring-up as `kuka_pdz_gripper_grasp_quickstart.md` (dual-arm iiwa7 + pdz gripper,
`plumbers_block` world, Fabrica `motion.pkl` replay), but the **arms run under the KUKA
Cartesian-impedance controller** (`cartesian_impedance_controller/CartesianImpedanceController`,
effort interface, compliant task-space control) instead of the position
`JointTrajectoryController`s.

Selected with one launch arg: `arm_control:=cartesian_impedance`. Default
(`arm_control:=joint_trajectory`) behaviour is unchanged — see the grasp quick-start for that.

## What this needs (one-time)

- `cartesian_impedance_controller`, `effort_controller_base`, `cartesian_impedance_msgs` built in
  the ROS workspace (they come from `~/franka_ros2_ws/src/kuka_lbr_control`, already required by
  the real-hardware `cartesian_impedance.launch.py` path):
  ```bash
  cd ~/franka_ros2_ws && colcon build --packages-select \
    cartesian_impedance_controller effort_controller_base cartesian_impedance_msgs \
    lbr_description lbr_dual_arm_description lbr_dual_arm_pdz_bringup --symlink-install
  source install/setup.bash
  ```
- The xacro/config wiring is already in the stack:
  - `lbr_description/ros2_control/lbr_system_interface.xacro` — `gazebo_command_mode:=effort`
    makes the 7 arm joints expose an **effort-only** command interface under gz_ros2_control.
  - `lbr_dual_arm_description/urdf/lbr_dual_arm.xacro` — `arm_control:=cartesian_impedance`
    threads that through and picks the controllers YAML below.
  - `lbr_dual_arm_description/ros2_control/dual_arm_gazebo_cartesian_impedance_controllers.yaml`
    — one controller_manager: `joint_state_broadcaster`, `cartesian_impedance_lbr_one/_two`
    (7 arm joints each), `gripper_controller_lbr_one/_two` (one pdz finger joint each).

## ⚠️ Gravity: pick the matching `compensate_gravity`

The Cartesian-impedance controller adds `+G(q)` gravity-compensation torque whenever
`compensate_gravity: true`. That value **must match the Gazebo world's gravity** or the arms
misbehave:

| `plumbers_block.sdf` gravity | set in `dual_arm_gazebo_cartesian_impedance_controllers.yaml` | result |
| --- | --- | --- |
| **OFF** — the `TEMP diagnostic (2026-08-25)` `<gravity>0 0 0` still in the world file | `compensate_gravity: false` (edit both `cartesian_impedance_lbr_one`/`_two` blocks) | arms hold; nothing to cancel |
| **ON** — physical end state (needs the real fixture to catch parts) | `compensate_gravity: true` (the file's default) | controller holds the arms against gravity |

The file ships with `true` (the intended end state). **For a quick check against the world as it
is right now (gravity off), flip both blocks to `false` before launching**, otherwise the arms
drift upward under the uncancelled `+G(q)`.

Re-enabling world gravity also brings back the fixture's real-mesh collision cost (~36x slower
than real-time — see open issue #1 in `kuka_pdz_executor_node_handoff.md`). There is still no
fast gravity-on config.

## 1. Launch Gazebo (Cartesian-impedance arms)

```bash
export GZ_IP=127.0.0.1 IGN_IP=127.0.0.1
ros2 launch lbr_dual_arm_pdz_bringup gazebo.launch.py arm_control:=cartesian_impedance
```

`robot_name` defaults to `lbr_dual_arm_pdz`. Controllers are spawned **sequentially** (five of
them race the controller_manager otherwise), so wait for `Configured and activated` to appear
**five** times, in this order:

```
joint_state_broadcaster
cartesian_impedance_lbr_one
cartesian_impedance_lbr_two
gripper_controller_lbr_one
gripper_controller_lbr_two
```

Stale-launch check and process hygiene are identical to
`kuka_pdz_gripper_grasp_quickstart.md` §1 (`ps -eo pid,ppid,etime,cmd | grep -E
"lbr_dual_arm_pdz_bringup|ign gazebo|gzserver|gzclient"`, `kill -SIGINT <launch PID>` if stale).

### Optional sanity checks (before playing the plan)

```bash
# All five controllers active:
ros2 control list_controllers -c /lbr_dual_arm_pdz/controller_manager

# Arm joints on effort, no position command interface claimed:
ros2 control list_hardware_interfaces -c /lbr_dual_arm_pdz/controller_manager | grep 'lbr_one_A1'

# The impedance FollowJointTrajectory actions the executor will use:
ros2 action list | grep cartesian_impedance
#   /lbr_dual_arm_pdz/cartesian_impedance_lbr_one/follow_joint_trajectory
#   /lbr_dual_arm_pdz/cartesian_impedance_lbr_two/follow_joint_trajectory

# Compliance poke: nudge one flange's spring target 3 cm in +z, watch it move there and
# spring back when you retract the target. (frame = that arm's link_0)
ros2 topic pub --once /lbr_dual_arm_pdz/cartesian_impedance_lbr_one/target_frame \
  geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "lbr_one_link_0"}, pose: {position: {x: 0.5, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}'
```

## 2. Play the motion plan

Second terminal — pass the **matching** `--arm-control`:

```bash
python3 ~/franka_ros2_ws/src/lbr_fri_ros2_stack/lbr_demos/lbr_dual_arm/lbr_dual_arm_pdz_bringup/\
lbr_dual_arm_pdz_bringup/plan_executor_node.py --arm-control cartesian_impedance
```

Positional args are unchanged: `[log_dir] [world_name]`, defaulting to
`~/Fabrica/logs/plumbers_block_sim` and `plumbers_block`. `--arm-control` can go in any
position; omit it (or pass `joint_trajectory`) only if you launched step 1 without
`arm_control:=cartesian_impedance` — the two must agree or the executor's action clients hang on
`Waiting for cartesian_impedance_lbr_one action server...`.

What the executor does differently in this mode:
- Arm paths (`motion.pkl` `arm` entries) go to `cartesian_impedance_lbr_{one,two}/
  follow_joint_trajectory`. The patched controller samples the joint trajectory, drives the
  Cartesian spring target through FK, and uses the trajectory as the nullspace target — so the
  arm tracks the plan **compliantly**, not rigidly.
- Gripper entries go to the separate `gripper_controller_lbr_{one,two}` (position), not the arm
  controller.

Same known limitation as the grasp quick-start: `/world/plumbers_block/set_pose` isn't available
in this world (open issue #3), so held parts don't visually track the gripper — watch the
arm/gripper motion, not the parts.

## 3. Gain tuning

`stiffness` (1500 N/m trans, 75 Nm/rad rot) and `trajectory_nullspace_stiffness` in
`dual_arm_gazebo_cartesian_impedance_controllers.yaml` are the **real-FRI-hardware** values
carried over as a starting point. Sim inertia/contact differ (kuka_control's own Gazebo default
is 1000 / 30). If the arms oscillate, overshoot plan waypoints, or buzz:
- lower `stiffness.trans_*` / `stiffness.rot_*` first;
- the diagonal `stiffness.*` params are live-re-readable — `ros2 param set
  /lbr_dual_arm_pdz/cartesian_impedance_lbr_one stiffness.trans_x 800.0` etc. while the arm is
  stationary (damping re-derives itself at a fixed sqrt(2)/2 ratio);
- if A7 times trajectories out into `ABORTED`, raise `trajectory_nullspace_stiffness` (needs a
  relaunch — it's not in the live-reread set).

## 4. Shut down cleanly

Identical to `kuka_pdz_gripper_grasp_quickstart.md` §3 — SIGINT the `plan_executor_node.py`
terminal first, then the launch terminal; use `SIGINT` not `SIGKILL` to avoid orphaned
`ign gazebo`/`gzserver`/`gzclient`:

```bash
ps -eo pid,ppid,cmd | grep -E "plan_executor_node|lbr_dual_arm_pdz_bringup" | grep -v grep
kill -SIGINT <plan_executor_pid> <launch_pid>
sleep 5
ps -eo pid,ppid,cmd | grep -E "plan_executor_node|lbr_dual_arm_pdz_bringup|ign gazebo|gzserver|gzclient" | grep -v grep
# empty output = fully clean
```

## Environment gotchas

Same set as `kuka_pdz_gripper_grasp_quickstart.md`:
- `export GZ_IP=127.0.0.1 IGN_IP=127.0.0.1` before `ros2 launch` — the launch file doesn't set
  them.
- Controller spawners failing/timing out (`Failed getting a result from calling
  .../load_controller`) → check for a **remote real KUKA control stack** on the network
  (`ros2 node list` showing `/lbr`, `/lbr_one`, `/lbr_two` from another machine) before assuming
  a local bug; a distinct `ROS_DOMAIN_ID` fixes it.
- Sim "frozen" mid-goal → check RTF (`ign topic -e -t /world/plumbers_block/stats -n 1`) before
  assuming a control bug — with world gravity on, ~36x slower is expected, not a hang.
