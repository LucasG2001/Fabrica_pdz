# KUKA pdz gripper grasp-contact tuning — status (2026-08-25)

## Ask
User report: in the KUKA plumbers_block grasping sim, parts rotate in the pdz gripper and fall
out. Asked to tune contact properties for good grip; later escalated to much higher friction and
finger effort limit after visually inspecting a live run.

## Status: fix applied, xacro-verified, NOT yet visually load-tested

## Diagnosis
`assets/pdz_gripper_description/urdf/pdz_gripper_macro.xacro` defines the pdz gripper's TPU
rubber pad collisions (`left_tpu_pad` / `right_tpu_pad`, on `pdz_gripper_left_finger_link` /
`pdz_gripper_right_finger_link`) with no friction/contact properties at all — no
`<gazebo reference=...>` block anywhere in the file, so the pads used gz-sim's generic default
friction instead of anything representing real rubber-on-part contact. The plumbers_block parts
already have `mu=1.6` (`planning/utils/generate_plumbers_block_gz_assets.py`,
`COLLISION_FRICTION_MU`) to stop them sliding after settling, but the gripper pads holding those
same parts had no matching value — the pad-to-part interface was weaker than the
part-to-fixture interface.

## Fix applied (both in `assets/pdz_gripper_description/urdf/pdz_gripper_macro.xacro`, live path,
not a worktree copy — the untracked-file/worktree-isolation issue from the first pass of this
session was resolved by editing directly in the main checkout, foreground)

1. **Friction** — added, guarded by `${use_pads}` so it never references a collision that doesn't
   exist when pads are disabled:
   ```xml
   <gazebo reference="left_tpu_pad">
     <mu1>30</mu1>
     <mu2>30</mu2>
   </gazebo>
   ```
   (mirrored for `right_tpu_pad`). First applied at `1.8` (just above the parts' `mu=1.6`), then
   raised to `30` on explicit user request after a live visual inspection where a part visibly
   pushed the gripper.

2. **Finger effort/torque limit** — raised on both `pdz_gripper_left_finger_joint` and
   `pdz_gripper_right_finger_joint` from `100.0` to `5000.0`:
   ```xml
   <limit lower="0.0" upper="0.032" effort="5000.0" velocity="0.05"/>
   ```
   This is the `<limit effort=...>` URDF/SDF value dartsim uses as the joint's max generalized
   force — raising it lets the position-tracking finger joint push back harder against a part
   that's pushing on it, rather than yielding. Per user's explicit instruction ("very high"); no
   attempt made to find a more conservative sufficient value.

Both changes verified 2026-08-25 (twice — once at mu=1.8/effort=100→before the escalation, once
at mu=30/effort=5000 after) by re-running
`xacro lbr_dual_arm.xacro model:=iiwa7 gripper:=pdz mode:=gazebo` end-to-end: exit 0, and grep
confirms exactly 4 `<gazebo reference="{left,right}_tpu_pad">` blocks and 4 finger-joint `<limit
effort="...">` occurrences in the expanded URDF (2 pads/joints × 2 arms — the pad collision `name=`
attributes are **not** prefix-scoped in this macro, so both arms' pads/joints share identical
values, which is fine here since both arms should match).

## Live run this session (informational — did not actually stress-test grip)
Launched `ros2 launch lbr_dual_arm_pdz_bringup gazebo.launch.py robot_name:=lbr_dual_arm_pdz
gripper:=pdz world:=plumbers_block` (gravity-off/no-fixture diagnostic scene, per
[[kuka_pdz_plumbers_block_status_20260825]]), all 3 controllers configured+activated cleanly, then
ran `plan_executor_node.py` against it while the user watched. `lbr_two` reached the point of
closing on part 3 and starting to transport it before the user said "it's fine" and asked to stop.
This confirmed the sim loads/runs with the friction change in place and reproduced the earlier
motion-plan validation, but **did not validate grip strength**: gravity is off in this scene (no
force trying to pull/rotate the part out) and `/world/plumbers_block/set_pose` is still
unavailable (open issue #3 in `kuka_pdz_executor_node_handoff.md`), so held parts don't even
visually track the gripper. The user's "gets pushed by parts" comment that triggered the mu=30/
effort=5000 escalation was presumably from watching the gripper closing against a part's collision
geometry during that run, not a controlled load test.

Both the launch and `plan_executor_node.py` were cleanly stopped (`SIGINT` to each parent PID,
verified no orphaned `ign gazebo`/`gzserver`/`gzclient`/`plan_executor_node` processes) before this
doc was finalized. **The mu=30/effort=5000 values have not yet been loaded into a live run** —
only xacro-parse-verified. Next session should relaunch to visually confirm before considering this
closed.

## dartsim contact-property support (confirmed 2026-08-25 from source, not assumed)
Checked the actual physics backend in use here (`libignition-physics5-dartsim`, matching
gz-physics `ign-physics5`'s `dartsim/src/SDFFeatures.cc`): dartsim's SDF surface parsing reads
**only** `<surface><friction><ode>` (`mu`, `mu2`, `slip1`, `slip2`, `fdir1`) and
`<surface><bounce><restitution_coefficient>`, all applied straight to the DART engine. It never
references `min_depth`/`max_vel` (ODE's max-penetration/contact-correction-velocity knobs) or
`velocity_decay` at all — genuinely absent from the parser, not just defaulted, so those are dead
knobs under this engine. `slip1`/`slip2` default to 0 (rigid, no-slip) when unset, already optimal
for grip. Joint-axis `<dynamics><damping>`/`<friction>` IS applied by dartsim but governs the
finger joint's own actuator dynamics, not the pad-part contact — irrelevant to a part rotating
loose in a closed grip. Full detail in memory: [[dartsim_sdf_contact_property_support]].

There is a second, distinct lever that was identified but **not applied**: `gz_ros2_control`'s
`position_proportional_gain` ROS parameter (default `0.1`, confirmed via the `gz_ros2_control`
`humble` branch source) converts a position-controlled joint's tracking error into a corrective
velocity command — i.e. it's the finger joint's positional "stiffness" against an external push,
separate from both contact friction and the URDF effort limit. It is declared once per
`gz_ros2_control` node/plugin instance (not overridable per-joint via the URDF), and this
dual-arm setup uses a single shared plugin instance for both arms' arm joints and gripper joints
(see [[kuka_dual_arm_gazebo_gz_ros2_control]]) — so raising it would also stiffen the arms'
own joint-trajectory tracking, not just the gripper. Not investigated further or applied since the
user's mu/effort-limit request was more targeted; worth revisiting if effort-limit + friction
alone don't fix the push-through behavior, but treat it as a global-blast-radius change requiring
its own check on arm tracking quality, not a drop-in gripper-only fix.

## Next steps
1. Relaunch (`ros2 launch lbr_dual_arm_pdz_bringup gazebo.launch.py robot_name:=lbr_dual_arm_pdz
   gripper:=pdz world:=plumbers_block`) and re-run `plan_executor_node.py` (or a more targeted
   single-grasp test) to visually confirm the mu=30/effort=5000 values actually stop the part
   from being pushed/rotated out — this session stopped before that check happened.
2. Per [[feedback_kuka_gazebo_checkpoints]], checkpoint with a screenshot/confirmation once
   verified, before moving to anything else (e.g. the unrelated `screw_b` ~5cm offset issue).
3. If pushing-through is still observed after the relaunch, consider the
   `position_proportional_gain` lever above — but check its effect on arm tracking first, since
   it's shared across both arms' arm+gripper joints.

## Worktree left behind (from the earlier, background-session part of this effort)
`.claude/worktrees/kuka-gripper-friction` (branch `worktree-kuka-gripper-friction`) — empty of
real changes, safe to remove whenever.
