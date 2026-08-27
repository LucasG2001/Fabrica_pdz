# KUKA hardware/mock executor — gripper actuation — handoff (2026-08-25, rev. 2026-08-27)

## Status: implemented, build/import-verified only — NOT run live

`hardware_plan_executor_node.py` (in
`~/franka_ros2_ws/src/lbr_fri_ros2_stack/lbr_demos/lbr_dual_arm/lbr_dual_arm_pdz_bringup/lbr_dual_arm_pdz_bringup/`)
can actuate `motion.pkl`'s gripper entries instead of logging and skipping them. **This is
opt-in via the `--gripper` flag (argparse, `action='store_true'`, default off).** Without
`--gripper` the node does not touch the grippers at all (no service wait, no startup homing, no
commands) and gripper entries are logged and skipped; every "at startup the node..." / "moves
both real grippers automatically" statement below is conditional on `--gripper` being passed.

**2026-08-27 rewire — binary open/close only.** The original design published a normalized
target to `~/position_command` and polled `~/position` feedback for fine width control. That has
been removed: the executor now calls only `servo_gripper`'s `~/open` / `~/close` `Trigger`
services (namespaced `/left`, `/right` — see `ARM_TO_GRIPPER_NS`). `motion.pkl`'s `open_ratio`
is thresholded via `gripper_open_ratio_to_action()` (`> GRIPPER_OPEN_RATIO_THRESHOLD` = 0.5 →
`~/open`, else `~/close`); `~/close` drives to the torque-limited closed endpoint (force-limited
grip). Each call (startup homing and per-plan-entry alike) goes through `_call_gripper_trigger()`
and blocks on the `Trigger` result, raising the whole plan on a failed/timed-out result
(`GRIPPER_MOTION_TIMEOUT_S`, 30 s). No `Float64`/topic publisher, no `~/position` subscription,
no settle sleeps, no `_wait_for_gripper_position` any more.

See `docs/kuka_hardware_mock_executor_quickstart.md` for the run commands (now a 5-step doc — a
new step 1 launches the grippers before the arm bringup). This file is the design record and the
open items before a live test.

Verified so far: `python3 -m py_compile hardware_plan_executor_node.py` (exit 0) after the
rewire; earlier `colcon build --packages-select lbr_dual_arm_pdz_bringup` (exit 0) and a direct
import. **No round trip against a real or even a running `gripper_controller` node has
happened** — the `~/open` / `~/close` service wiring is untested end-to-end.

## Why this took investigation before implementing

Asked to check for "multiple branches for different types of grippers" before writing code.
There weren't multiple *gripper types* (`~/Workspaces/gripper`'s `main`/`julien`/`new_gripper`
branches are just earlier history of the same open/close-only design) — but there turned out to
be two live, incompatible *implementations* of the same physical hardware, which is a more
important finding for anyone touching this next:

1. **`servo_gripper`** (`~/Workspaces/gripper`, branch `feature/fine-gripper-control`) — the
   package the user's copied README documents. Full interface: `open`/`close`/`stop`/`calibrate`
   services, `~/position_command` (fine, normalized 0.0=open..1.0=closed) and `~/command`
   (binary) topics, `~/position`/`~/raw_position`/`~/travel_counts`/`~/jaw_travel_m`/`~/state`
   diagnostics. Node name `gripper_controller`, launched per-side via `start_dual_grippers.sh` →
   `dual_grippers.launch.py`. **Live namespace is `/left` and `/right`** (confirmed from a real
   `ros2 service list` on 2026-08-27: `/left/gripper_controller/{open,close,stop,calibrate,...}`,
   same for `/right`) — the copied README's "launched as `lbr_one`/`lbr_two`" is stale, the
   launch was changed to side names. The executor's `ARM_TO_GRIPPER_NS` constant maps
   `lbr_one`→`left`, `lbr_two`→`right`. Already built (`install/`/`build/` present) on the
   gripper machine, but **not running** as of this session.
   Verified `open_close_node.py` (what the launch file actually starts) really implements the
   full README interface, including `position_command`/`calibrate` — no doc/code mismatch.
   `gripper_node.py` in the same package is dead code (its console-script entry point in
   `setup.py` actually points at `open_close_node:main`, not at its own `ServoGripperNode`
   class) — ignore it, it's not a hidden third variant.
2. **`servo_gripper_julien`** — a separate package in a **different workspace**
   (`~/Workspaces/servo_test`, not `~/Workspaces/gripper`), node name `gripper_controller_julien`.
   This is what was **actually running** on `lbr_one` at the start of this session (confirmed via
   `ros2 node list`/`service list`/`topic list` over SSH to `s3c@192.170.20.3`) —
   `lbr_two`'s gripper wasn't running at all. Binary `open`/`close`/`stop` plus an extra
   `hold_close` service (continuous closing torque, e.g. for a firm hold without a full stroke
   confirmation) — no `calibrate`, no `position_command`, no fine positioning at all. Confirmed
   by reading `open_close_node.py` in that package directly, not just inferring from the
   `ros2 service list` output.

A calibration file already exists on the gripper machine
(`~/.ros/servo_gripper_calibration.json`) with `stroke_counts` for both USB adapters (both plain-
and `gt2-2mm-20t-64mm-v1`-profile-keyed), so `servo_gripper`'s stroke calibration itself is
already done — only the nodes need to actually be running.

A ros2_control hardware-interface-plugin approach (gripper as a joint inside
`joint_trajectory_controller`, matching how the arms are driven) was considered and ruled out as
out of scope: `dual_arm_controllers.yaml` only claims the 14 arm joints, and writing a
hardware_interface plugin for the ST3215 serial protocol is materially more work than wiring the
existing standalone node in — flagged as the eventual "proper" fix, not attempted here.

**User decision (via AskUserQuestion 2026-08-25)**: originally targeted `servo_gripper`'s fine
`position_command` interface. **Superseded 2026-08-27**: the user asked to rewire to plain
open/close commands, so the executor now uses only `~/open` / `~/close` `Trigger` services (which
both `servo_gripper` and `servo_gripper_julien` expose, though the executor's `/left`,`/right` +
`gripper_controller` node-name assumption matches `servo_gripper`). Fine width control is gone.
The user deferred actually stopping `servo_gripper_julien`/relaunching `servo_gripper` on the
remote machine to a later, explicit test session rather than have a session touch the live
processes.

## Design (see `hardware_plan_executor_node.py`'s module docstring for the authoritative version)

- `gripper_open_ratio_to_action(open_ratio) -> 'open' | 'close'`: thresholds Fabrica's
  `open_ratio` (0=closed, 1=open, from `planning/robot/geometry.py`) at
  `GRIPPER_OPEN_RATIO_THRESHOLD` (0.5) — strictly greater → `'open'`, else `'close'`. No polarity
  flip, no clip; any intermediate width in the plan rounds to a full open or close.
- `_send_gripper_command` → `_call_gripper_trigger(arm, client, action)`: calls the chosen
  `~/open` or `~/close` service and blocks on the `Trigger` result (`spin_until_future_complete`,
  `GRIPPER_MOTION_TIMEOUT_S` = 30 s). Raises the whole plan on `result is None` (timeout) or
  `not result.success` — matching the arm path's stop-the-plan-on-any-unexplained-failure design.
  `servo_gripper`'s open/close service only returns once the servo has reached (or torque-limited
  at) the endpoint, so there's no separate feedback poll or settle sleep. Removed with the
  rewire: the `Float64` `~/position_command` publisher, the `~/position` subscription
  (`_gripper_position_cb`), `_wait_for_gripper_position`, `GRIPPER_POSITION_TOLERANCE`,
  `GRIPPER_SETTLE_BUFFER_S`, and the `time` / `std_msgs.msg.Float64` imports.
- Startup: after the existing arm-side waits (action server, initial `/joint_states`), the node
  now also waits for both sides' `/{left,right}/gripper_controller/open` and `/close` services
  (10s timeout, raises with an explicit message pointing at a wrong namespace / `ARM_TO_GRIPPER_NS`
  or `servo_gripper_julien` as likely causes if missing) and then runs a `~/close` → `~/open`
  cycle per side, blocking on each `Trigger` result. The `~/open` at the end
  establishes the required post-restart multi-turn zero — per the README this is mandatory every
  controller restart even with a saved calibration, since a single-turn encoder angle alone can't
  prove which shaft revolution the screw is on. **This means the node moves both real grippers
  (a full open motion) automatically at every startup, regardless of arm mock/hardware mode.**
- The node deliberately does **not** call `~/calibrate` itself — that requires empty jaws and was
  judged unsafe to automate without the operator physically checking the jaws first; it stays a
  manual pre-step per the README's own "Calibrate and position" section. Not needed on this rig
  right now anyway since a calibration file already exists.
- `package.xml`: `std_srvs` exec_depend (for `Trigger`) stays; the `std_msgs` exec_depend added
  earlier for `Float64` was removed with the rewire (nothing in the package uses it any more).

## Not yet done / before a live test

1. **Stop `servo_gripper_julien` on `lbr_one` and launch `servo_gripper`'s
   `start_dual_grippers.sh` for both arms** on `s3c@192.170.20.3` — deliberately left for the
   user to do in the actual test session, not done here. Both packages will otherwise fight over
   the same serial ports.
2. **First live check should be the gripper wiring in isolation**, before a full plan replay:
   launch just the two `gripper_controller` nodes, confirm `~/open` and `~/close` each return
   success and actually move the jaws — before trusting the executor's own startup homing cycle.
3. **Then a `mode:=mock` dry run of the full 5-step quickstart**, per the existing "never run
   live" caveat that already applied to the arm-only version of this node — the gripper addition
   doesn't change that the arm side itself has also never been run against a live bringup yet.
4. **`GRIPPER_MOTION_TIMEOUT_S` (30 s) and `GRIPPER_OPEN_RATIO_THRESHOLD` (0.5) are first-pass
   values.** Confirm 30 s covers a full 64 mm stroke at the calibration torque, and that 0.5 is
   the right split for this plan's `open_ratio` values (a grasp step planned at e.g. 0.3 open
   still maps to `~/close`, which is probably intended; a release at 0.9 maps to `~/open`).
5. With binary close, whether `motion.pkl`'s `open_ratio` widths were retargeted for this
   gripper's real 64 mm GT2 jaw travel no longer matters for *commanded width* (there is none),
   but the **direction** (grip vs release) per entry still has to be correct after thresholding —
   worth spot-checking the plan's gripper entries against `GRIPPER_OPEN_RATIO_THRESHOLD`.

## Key file locations

- `~/franka_ros2_ws/src/lbr_fri_ros2_stack/lbr_demos/lbr_dual_arm/lbr_dual_arm_pdz_bringup/lbr_dual_arm_pdz_bringup/hardware_plan_executor_node.py`
  — the node (this session's edits).
- `~/Fabrica/docs/kuka_hardware_mock_executor_quickstart.md` — run commands, updated this session.
- `~/Workspaces/gripper/ros2/servo_gripper/README.md` (on `s3c@192.170.20.3`, copied locally to
  `~/README.md` this session) — the target interface's own documentation.
- `~/Workspaces/gripper/ros2/servo_gripper/start_dual_grippers.sh` (remote) — launches both
  `servo_gripper` gripper_controller nodes.
- `~/.ros/servo_gripper_calibration.json` (remote) — existing calibration for both arms.
