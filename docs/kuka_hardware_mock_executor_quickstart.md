# KUKA dual-arm hardware/mock plan executor — quick-start

Runs a Fabrica `motion.pkl` arm trajectory against `lbr_dual_arm_bringup`'s real FRI hardware or
`mode:=mock` (not the pdz/Gazebo bringup — different package, different topology). See
`kuka_hardware_plan_executor.md`-style notes in memory for the design; this doc is just the
commands. **Status: build/import-verified only, never run live** — do the mock dry run below
before ever pointing this at `hardware.launch.py`. The gripper wiring specifically has not been
run against a live `gripper_controller` node at all yet (no open/close round trip observed) —
see `kuka_gripper_executor_handoff.md`.

Arm goals go through `lbr_dual_arm_bringup`'s `joint_trajectory_controller` as before. Gripper
entries can be actuated for real, via the standalone `servo_gripper` node (ST3215 servo
grippers) — **not** through ros2_control, and **not** mock even when the arms are: there is no
mock/sim variant of the gripper hardware, so gripper motion happens on the real servos regardless
of `mode:=mock` vs `mode:=hardware` for the arms.

**Gripper actuation is opt-in via the `--gripper` flag on `hardware_plan_executor_node.py`
(default off), and is binary open/close only** — the executor calls `servo_gripper`'s `~/open` /
`~/close` `Trigger` services, not `~/position_command`; there is no fine width control.
`motion.pkl`'s `open_ratio` is thresholded (`> 0.5` → `~/open`, else `~/close`, see
`GRIPPER_OPEN_RATIO_THRESHOLD`), so `~/close` drives to the torque-limited closed endpoint (a
force-limited grip). Without `--gripper`, the node never touches the grippers at all (no service
wait, no homing, no commands) and gripper entries in `motion.pkl` are logged and skipped —
arm-trajectory-only replay. Note `--gripper` is a **different arg** from the
`use_gripper:=true|false` launch argument on `hardware.launch.py` / `mock.launch.py` (defaults
`true`), which only controls whether the Y-gripper geometry is attached to `link_7` in the
URDF/SRDF — that launch arg does not actuate anything.

See `kuka_gripper_executor_handoff.md` for the full design/status; the essentials for running it
(all only relevant when you pass `--gripper`):

- Requires the `gripper_controller` nodes running for **both** sides — namespaced `/left` and
  `/right` on the live rig (**not** `/lbr_one` / `/lbr_two`; an older `servo_gripper` README
  still says the arm names, but the launch was changed). Only `~/open` and `~/close` are needed,
  so either `servo_gripper` or `servo_gripper_julien` would technically serve — but the executor
  checks the `/left`,`/right` + `gripper_controller` node name, which is `servo_gripper`. Start
  them with:
  ```bash
  ~/Workspaces/gripper/ros2/servo_gripper/start_dual_grippers.sh
  ```
  on the machine the gripper USB adapters are attached to (currently `s3c@192.170.20.3`). If
  `servo_gripper_julien` is already running there (check with `ros2 node list`), stop it first —
  it and `servo_gripper` both claim the same serial ports.
- The executor maps arm → gripper side via `ARM_TO_GRIPPER_NS` in
  `hardware_plan_executor_node.py` (`lbr_one` → `left`, `lbr_two` → `right`). Verify the live
  namespaces first with `ros2 service list | grep gripper_controller`; if they differ (e.g. the
  physical grippers were swapped between USB adapters), edit that constant before running —
  otherwise the node raises `"/left/gripper_controller/open service not available"` at startup.
- At startup, `hardware_plan_executor_node.py` waits for both sides' `~/open` and `~/close`
  services, then runs a `~/close` → `~/open` cycle per side: confirms the jaws move to known
  endpoints on this run, clears anything left held from a prior run, and the final `~/open`
  re-establishes the required post-restart multi-turn zero (mandatory per servo_gripper's README
  even with a saved calibration). It does **not** call `~/calibrate` itself — that requires empty
  jaws and stays a manual, deliberate step. A calibration for both grippers already exists on the
  rig (`~/.ros/servo_gripper_calibration.json`), so this should just work if the nodes are up.
- If either side's `~/open` or `~/close` service isn't reachable, the node raises immediately at
  startup rather than silently falling back to arm-only replay. Each plan-time open/close call
  also blocks on the service result and raises the whole plan on a failed/timed-out result.

## 1. Launch the grippers

Skip this whole step if you are not passing `--gripper` in step 4 (arm-only replay).

On the machine with the gripper USB adapters attached (`s3c@192.170.20.3`):
```bash
~/Workspaces/gripper/ros2/servo_gripper/start_dual_grippers.sh
```
Confirm both `/left/gripper_controller` and `/right/gripper_controller` show up in
`ros2 node list` (and `ros2 service list | grep gripper_controller`) before continuing — step 4's
executor will fail fast at startup if either is missing or under a different namespace than
`ARM_TO_GRIPPER_NS` expects. This step actuates real servos (a homing close→open cycle happens
automatically once step 4 starts), independent of whether step 2 below uses mock or real arm
hardware.

## 2. Launch the mock (or real) bringup

Mock (fake hardware, safe, default choice for a dry run):
```bash
ros2 launch lbr_dual_arm_bringup mock.launch.py
```

Real FRI hardware — only once the mock dry run below has been verified end-to-end, and the
KUKA controller-side FRI application is running with a matching send-period/position-mode
config (`lbr_one_system_config.yaml` port 30200, `lbr_two_system_config.yaml` port 30201, both
`client_command_mode: position`):
```bash
ros2 launch lbr_dual_arm_bringup hardware.launch.py
```
(`use_gripper` defaults to `true` and only affects whether the Y-gripper geometry is in the
URDF/SRDF — it is unrelated to the executor's `--gripper` flag in step 4, which is what actually
drives the servos. Pass `use_gripper:=false` only if you want the bare flange in the model.)

Either way, wait for `Configured and activated` to appear **twice**
(`joint_state_broadcaster` then `joint_trajectory_controller` — one combined controller for
both arms here, unlike the Gazebo/pdz bringup's per-arm split) before doing anything else.

## 3. (Optional) MoveIt + RViz for visualization

In a second terminal, same `robot_name`/`mode` as step 1 so it attaches to the same running
controller_manager instead of starting its own:
```bash
ros2 launch lbr_dual_arm_bringup move_group.launch.py mode:=mock rviz:=true
```
(`mode:=hardware` for the real-hardware case.) This is visualization only — the executor talks
directly to `joint_trajectory_controller`'s `FollowJointTrajectory` action, not through
move_group, so RViz's `RobotModel`/TF display just reflects live `/joint_states` as the plan
runs; move_group itself sits idle.

## 4. Play the motion plan

In a third terminal:
```bash
# Arm trajectory only (no gripper actuation, step 1 not needed):
python3 ~/franka_ros2_ws/src/lbr_fri_ros2_stack/lbr_demos/lbr_dual_arm/lbr_dual_arm_pdz_bringup/\
lbr_dual_arm_pdz_bringup/hardware_plan_executor_node.py

# Also home and actuate the real grippers (requires step 1 running for both arms):
python3 ~/franka_ros2_ws/src/lbr_fri_ros2_stack/lbr_demos/lbr_dual_arm/lbr_dual_arm_pdz_bringup/\
lbr_dual_arm_pdz_bringup/hardware_plan_executor_node.py --gripper
```
Positional args default to `~/Fabrica/logs/plumbers_block_sim` (log dir) and `lbr_dual_arm`
(`robot_name`, matching step 2's default namespace). Pass a different log dir or robot_name as
`hardware_plan_executor_node.py <log_dir> <robot_name> [--gripper]` if needed — `--gripper` is a
flag and can go in any position.

The node waits for the action server and an initial `/joint_states` message before doing
anything; with `--gripper` it additionally waits for both sides' `~/open` and `~/close`
services, so it's safe to start this terminal as soon as step 1's gripper nodes (if using
`--gripper`) and step 2's controller are active. With `--gripper` it then homes both grippers
(a `~/close` → `~/open` cycle, actual motion on the real servos) before starting the plan;
without it, gripper entries in the plan are logged and skipped and only the arm path is replayed.
Each plan gripper entry becomes a single `~/open` or `~/close` call (from the thresholded
`open_ratio`) that blocks on the service result. On any rejected arm goal, failed trajectory
execution, unreachable gripper service, or a failed/timed-out (`GRIPPER_MOTION_TIMEOUT_S`, 30s)
open/close call, it raises and stops the whole plan immediately rather than continuing — treat
that as a hard stop, not something to retry blindly, especially against real hardware.

## 5. Shut down cleanly

```bash
# Ctrl-C (SIGINT) the executor terminal first, then RViz/move_group (if running), then the
# bringup launch terminal, then the gripper launch terminal. Or from another shell:
ps -eo pid,ppid,cmd | grep -E "hardware_plan_executor_node|lbr_dual_arm_bringup|move_group|rviz2" | grep -v grep
kill -SIGINT <pids>
sleep 3
ps -eo pid,ppid,cmd | grep -E "hardware_plan_executor_node|ros2_control_node|move_group|rviz2" | grep -v grep
# empty output = fully clean
```
No `ign gazebo`/`gzserver` processes to worry about here (that's Gazebo-only) — just
`ros2_control_node`, the controller spawners, and, for real hardware, the FRI UDP session on the
KUKA controller side (which drops on its own once `ros2_control_node` exits). On the gripper
machine, SIGINT `start_dual_grippers.sh`'s launch PID separately (its two `gripper_controller`
nodes are unrelated processes on `s3c@192.170.20.3`, not covered by the `ps` above) — check with
`ps -eo pid,ppid,cmd | grep gripper_controller | grep -v grep`.

## Environment gotchas

- This machine's ROS2 DDS discovery (`ROS_DOMAIN_ID=0`, `ROS_LOCALHOST_ONLY=0`) can pick up a
  **remote real KUKA control stack** (`/lbr`, `/lbr_one`, `/lbr_two` nodes) from another machine
  on the network. For a mock dry run this is just noise/confusion risk (check `ros2 node list`
  if things look odd); for an actual `hardware.launch.py` run, make sure you know which physical
  arms you're about to command — use a distinct `ROS_DOMAIN_ID` or physically verify network
  topology first, don't rely on assumption.
- `hardware.launch.py`'s `arms` argument (`both` / `lbr_one` / `lbr_two`) controls which arm(s)
  connect as real FRI hardware; the other is forced to `mode:=mock` so `ros2_control_node`
  doesn't block forever waiting for a robot controller that isn't connected. Confirm this matches
  which physical controllers are actually powered/running their FRI application before launching.
