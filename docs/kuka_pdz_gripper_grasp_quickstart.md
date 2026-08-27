# KUKA pdz gripper grasp check — quick-start

Reproduces the live Gazebo session from 2026-08-25 with the current grasp-tuning settings applied.
See `kuka_pdz_gripper_friction_handoff.md` for the full diagnosis/history.

## ⚠️ Gravity must stay OFF

The world file has gravity forced to `0 0 0` via a `TEMP diagnostic (2026-08-25)` comment in
`~/franka_ros2_ws/src/lbr_fri_ros2_stack/lbr_demos/lbr_dual_arm/lbr_dual_arm_pdz_bringup/worlds/plumbers_block.sdf`.
**Do not re-enable it for a quick check.** Gravity only makes physical sense together with the
real fixture (to catch/support parts), and the fixture's real-mesh collision is what collapses
real-time factor to ~0.028 (36x slower than real-time) — see open issue #1 in
`kuka_pdz_executor_node_handoff.md`. There is currently no fast gravity-on diagnostic
configuration. If you need gravity on, expect the sim to run ~36x slower and budget accordingly.

Because gravity is off, this quick-start will **not** show a part falling out of a bad grip — it
only lets you watch the gripper close and judge geometry/contact by eye, or apply your own
disturbance to a held part manually. It does not by itself validate the mu=30/effort=5000 change.

## Current settings baked into the URDF

`assets/pdz_gripper_description/urdf/pdz_gripper_macro.xacro` (live path, no rebuild step needed
— `xacro` reads it directly at launch time):
- Pad friction: `mu1`/`mu2` = **30** on both `left_tpu_pad` and `right_tpu_pad`
- Finger joint effort limit: **5000** (was 100) on both `pdz_gripper_left_finger_joint` and
  `pdz_gripper_right_finger_joint`

## 1. Launch Gazebo

```bash
export GZ_IP=127.0.0.1 IGN_IP=127.0.0.1
ros2 launch lbr_dual_arm_pdz_bringup gazebo.launch.py \
  robot_name:=lbr_dual_arm_pdz gripper:=pdz world:=plumbers_block
```

Wait for `Configured and activated` to appear 3 times (`joint_state_broadcaster` +
`joint_trajectory_controller_lbr_one` + `joint_trajectory_controller_lbr_two`) before doing
anything else.

> To run the arms under the **Cartesian-impedance controller** instead of position
> `JointTrajectoryController`s, add `arm_control:=cartesian_impedance` here and follow
> `docs/kuka_gazebo_cartesian_impedance_quickstart.md` — different controller set (5 spawns),
> a `compensate_gravity` setting that must match world gravity, and the executor needs a
> matching `--arm-control cartesian_impedance`.

If Gazebo doesn't come up cleanly, first check for a stale/leftover launch from a previous
session:
```bash
ps -eo pid,ppid,etime,cmd | grep -E "lbr_dual_arm_pdz_bringup|ign gazebo|gzserver|gzclient" | grep -v grep
```
A healthy launch has `ign gazebo server` and `ign gazebo gui` child processes. If you find an old
one that never spawned those (just the launch parent + `robot_state_publisher`), it's stale —
`kill -SIGINT <launch PID>`, wait ~5s, confirm the process list is empty, then relaunch.

## 2. Play the motion plan

In a second terminal:
```bash
python3 ~/franka_ros2_ws/src/lbr_fri_ros2_stack/lbr_demos/lbr_dual_arm/lbr_dual_arm_pdz_bringup/\
lbr_dual_arm_pdz_bringup/plan_executor_node.py
```
No args needed — defaults to `~/Fabrica/logs/plumbers_block_sim`. Runs all 43 plan entries
end-to-end at near-real-time speed (gravity is off, so RTF stays ~0.99). Watch the Gazebo window
for gripper closing on each part.

Known limitation: `/world/plumbers_block/set_pose` isn't available in this world, so held parts
won't visually track the gripper once grasped (open issue #3) — the arm/gripper motion itself is
what to watch, not the part.

## 3. Shut down cleanly

```bash
# Ctrl-C (SIGINT) the plan_executor_node.py terminal first, then the launch terminal.
# Or from another shell, find and SIGINT both PIDs:
ps -eo pid,ppid,cmd | grep -E "plan_executor_node|lbr_dual_arm_pdz_bringup" | grep -v grep
kill -SIGINT <plan_executor_pid> <launch_pid>
sleep 5
ps -eo pid,ppid,cmd | grep -E "plan_executor_node|lbr_dual_arm_pdz_bringup|ign gazebo|gzserver|gzclient" | grep -v grep
# empty output = fully clean
```
Use `SIGINT`, not `SIGKILL` — avoids orphaned `ign gazebo`/`gzserver`/`gzclient` processes.

## Environment gotchas (carried from `kuka_pdz_executor_node_handoff.md`)

- Must `export GZ_IP=127.0.0.1 IGN_IP=127.0.0.1` before the `ros2 launch` — the launch file
  doesn't set these itself.
- If controller spawners fail/time out (`Failed getting a result from calling
  .../load_controller`), check for a **remote real KUKA control stack** on the network first
  (`ros2 node list` showing `/lbr`, `/lbr_one`, `/lbr_two`, etc. from another machine via DDS
  discovery) before assuming a local bug — disconnecting the network or using a distinct
  `ROS_DOMAIN_ID` fixes it.
- If the sim looks "frozen" mid-goal, check real-time factor
  (`ign topic -e -t /world/plumbers_block/stats -n 1`) before assuming a control bug.
