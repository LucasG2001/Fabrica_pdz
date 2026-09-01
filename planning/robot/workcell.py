import numpy as np


def get_board_dx():
    return 2.5


def get_kuka_mount_block_height():
    # Unlike the other arms here (bolted flush to the tabletop, base z=0), the physical KUKA
    # iiwa7 rig sits on a ~2.5cm riser block, so its base_link origin is actually 2.5cm above
    # the true table surface (world z=0, where ground_col_manager's ground plane and every other
    # arm's base_pos both live). Grasp_Planning's own world model encodes the same real-world gap
    # via DEFAULT_FLOOR_Z_WORLD_M (~3cm, dual_robot_simple_sim.py) -- close to this value but not
    # re-measured/reconciled here, so 2.5cm (the measured block height) is used as-is.
    #
    # Baking this into get_move_arm_pos/get_hold_arm_pos's z (rather than lowering the ground
    # plane) shifts the whole kinematic chain up by the same amount FK produces it from, so every
    # existing z=0-relative check (inverse_kinematics_above_ground's ground_z=0 default,
    # ground_col_manager) automatically gets the right 2.5cm of extra real clearance without
    # having to duplicate the offset at each call site.
    return 2.5


def get_move_arm_pos(arm_type):
    dx = get_board_dx()
    if arm_type == 'xarm7':
        return np.array([14.5 * dx, 10 * dx, 0])
    elif arm_type == 'panda':
        return np.array([18 * dx, 8 * dx, 0])
    elif arm_type == 'kuka':
        # Separation ±42cm (real, was Panda's ±45cm) -- see full derivation + history in
        # get_hold_arm_pos below.
        #
        # UPDATED (2026-08-24): Y changed from 8*dx=20 (Panda-inherited "reach gap" value, tuned
        # 2026-08-20 purely for the native kuka+Y-gripper grasp_gen path's feasibility -- see the
        # now-superseded comment this replaced, preserved in git history) to 10, matching
        # KUKA_BASE_Y in the reference dual-arm-kuka branch's real, measured workcell (July 2026)
        # -- confirmed by an exact numeric replay check: reconstructing a stored plumbers_block
        # pdz grasp's flange pose from its arm_q only matches the grasp's recorded pos/quat when
        # this Y=10 (was off by exactly 10cm in Y at the old value). The native kuka+Y-gripper
        # grasp_gen path is not known to be actively used any more (superseded by the pdz +
        # --use-graspplanning path); if it needs re-tuning for feasibility at this Y, that's a
        # separate, not-yet-revisited concern from this fix.
        return np.array([16.8 * dx, 10, get_kuka_mount_block_height()])
    elif arm_type == 'ur5e':
        return np.array([18 * dx, 10 * dx, 0])
    else:
        raise NotImplementedError


def get_hold_arm_pos(arm_type):
    dx = get_board_dx()
    if arm_type == 'xarm7':
        return np.array([-14.5 * dx, 10 * dx, 0])
    elif arm_type == 'panda':
        return np.array([-18 * dx, 8 * dx, 0])
    elif arm_type == 'kuka':
        # Real-rig-derived, symmetric with get_move_arm_pos.
        #
        # User-provided ground truth (2026-08-20): real dual-KUKA bases sit at (0,-0.42,0) and
        # (0,0.42,0) m in a frame centered between them; table surface z=-0.025m (matches
        # get_kuka_mount_block_height()'s independently-measured 2.5cm exactly -- good
        # cross-check); assembly fixture at ~(0.5,0,0) m, resting on the table, plus Fabrica's
        # usual per-part fixture offset on top. Real-world roles: Grasp_Planning hardcodes
        # holder=(0,-0.42,0), inserter=(0,0.42,0) (grasp_planning/envs/fr3_part_env.py) --
        # matched here to Fabrica's hold/move roles (hold~=holder, move~=inserter, already the
        # established semantic pairing elsewhere in this file's --use-graspplanning code) by
        # putting move (inserter) on the positive side and hold (holder) on the negative side, so
        # no change needed on the Grasp_Planning side.
        #
        # Separation (42cm) is applied as given. Reach gap is NOT the real ~50cm assembly offset,
        # by deliberate choice, not oversight -- three numeric attempts this session (55cm, then
        # a 65cm one caused by an arithmetic slip, then the correct 50cm) were each tested with a
        # controlled A/B (planning/run_grasp_arm_gen.py's check_grasp_feasible(verbose=True) on
        # part 0's full 130-candidate antipodal set, same seed, only arm_pos differing): the
        # Panda-inherited 35cm gap gets 8/130 feasible; every real-gap variant (50/55/65cm) gets
        # 0/130 -- the arm-position-dependent checks (mainly arm-ground collision, which roughly
        # tripled) reject every candidate that used to pass, while raw
        # inverse_kinematics_above_ground() calls on the same targets succeed fine at all these
        # distances (so it's not a basic kinematic-reach wall -- KUKA iiwa7's real reach is
        # ~800mm). Root cause not diagnosed further (open question: genuine rig limitation
        # reaching this low from that far, vs. Fabrica's kuka arm mesh/collision buffer/URDF only
        # ever having been tuned/validated at the shorter Panda-placeholder distance). User
        # decision (2026-08-20): keep 35cm pragmatically since it's what the sim can solve,
        # explicitly NOT a claim that the real assembly sits 35cm away -- revisit if this ever
        # needs reconciling with real hardware behavior.
        #
        # UPDATED (2026-08-24): Y changed from 8*dx=20 to 10 -- see get_move_arm_pos above for
        # the reason (matches the reference branch's real-measured KUKA_BASE_Y, confirmed by an
        # exact numeric replay check against a stored plumbers_block pdz grasp).
        return np.array([-16.8 * dx, 10, get_kuka_mount_block_height()])
    elif arm_type == 'ur5e':
        return np.array([-18 * dx, 10 * dx, 0])
    else:
        raise NotImplementedError


def get_dual_arm_pos(arm_type):
    return get_move_arm_pos(arm_type), get_hold_arm_pos(arm_type)


def get_single_arm_pos(arm_type):
    return get_move_arm_pos(arm_type)


def get_move_arm_euler():
    return np.array([0, 0, -np.pi / 2])


def get_hold_arm_euler():
    return np.array([0, 0, -np.pi / 2])


def get_dual_arm_euler():
    return get_move_arm_euler(), get_hold_arm_euler()


def get_single_arm_euler():
    return get_move_arm_euler()


# SHORT-TERM FIX (2026-08-30): the +/-[.,.,80] / -[.,.,0] z half-extents below are
# Panda-inherited. The taller KUKA iiwa7 sitting on its 2.5cm riser (get_kuka_mount_block_height,
# baked into get_*_arm_pos z) does NOT fit this envelope at its own rest_q: the buffered arm
# collision meshes span z in [+2.3, +82.9]cm vs the box's [+2.5, +82.5], overrunning ~2mm at the
# floor and ~4mm at the ceiling. motion_plan_arm's collision_fn folds that box-shell hit into an
# "arm and ground" collision, so every transport path that starts or ends at rest_q fails. The
# committed pipeline never hit this because it aborts earlier at pickup IK. Widen the KUKA z-band
# (floor to ~ground, ceiling +10cm) to clear it; x/y are unchanged. Proper fix is to re-derive
# get_*_arm_box from the real iiwa7 workspace.
_KUKA_ARM_BOX_DZ = np.array([0.0, 0.0, 10.0])
_KUKA_ARM_BOX_Z0 = 0.0  # floor of the KUKA arm box in world z (below the 2.5cm riser base)


def get_move_arm_box(arm_type):
    arm_pos = get_move_arm_pos(arm_type)
    lower = arm_pos - np.array([100.0, 100.0, 0.0])
    upper = arm_pos + np.array([30.0, 50.0, 80.0])
    if arm_type == 'kuka':
        lower[2] = _KUKA_ARM_BOX_Z0
        upper = upper + _KUKA_ARM_BOX_DZ
    return lower, upper


def get_hold_arm_box(arm_type):
    arm_pos = get_hold_arm_pos(arm_type)
    lower = arm_pos - np.array([30.0, 100.0, 0.0])
    upper = arm_pos + np.array([100.0, 50.0, 80.0])
    if arm_type == 'kuka':
        lower[2] = _KUKA_ARM_BOX_Z0
        upper = upper + _KUKA_ARM_BOX_DZ
    return lower, upper


def get_dual_arm_box(arm_type):
    return get_move_arm_box(arm_type), get_hold_arm_box(arm_type)


def get_single_arm_box(arm_type):
    return get_move_arm_box(arm_type)


def get_assembly_center(arm_type):
    dx = get_board_dx()
    if arm_type == 'xarm7':
        return np.array([0, -6 * dx, 0])
    elif arm_type == 'panda':
        return np.array([0, -6 * dx, 0])
    elif arm_type == 'kuka':
        return np.array([0, -6 * dx, 0])
    elif arm_type == 'ur5e':
        return np.array([0, -6 * dx, 0])
    else:
        raise NotImplementedError


def get_fixture_min_y(arm_type):
    dx = get_board_dx()
    if arm_type == 'xarm7':
        return 6 * dx
    elif arm_type == 'panda':
        return 4 * dx
    elif arm_type == 'kuka':
        # SHORT-TERM FIX (2026-08-30): the +4*dx value is inherited verbatim from Panda, whose
        # arm bases sit at large +y with the assembly at y=-6*dx; there a fixture packed into
        # [+4*dx, +4*dx+bin_y] lands in front of the arms. The KUKA bases are at y=+10 (roughly
        # on the +4*dx line) and yaw -90deg to face -y, so that same packing frame puts the
        # fixture behind/under the bases -> pickup IK is unreachable (parts 2, 3).
        #
        # min_fixture_y is the near (min-y) edge of the packing bin and of the carved fixture
        # box (see generate_pickup_pose / generate_fixture in run_fixture_gen.py), and it enters
        # the global pickup pose as a pure additive y offset. Setting it to -62.7 places the
        # fixture footprint at y in [-62.7, -62.7+bin_y] ~ [-62.7, -42.7], i.e. beyond the
        # assembly (y=-15), in front of both arms -- matching the known-good, dense-insertion-
        # validated layout recorded in logs/plumbers_block_sim/validation.json (fixture
        # footprint [-12.5,-62.7]..[12.5,-42.7]) and the 2026-08-21 motion.pkl solved against
        # it. Proper fix is to decouple the board layout from the assembled-part transform and
        # add an IK-reachability gate; see docs/fixture_pickup_unreachable_handoff.md.
        #
        # -62.7 reproduces validation.json's footprint exactly but leaves the farthest parts
        # (~y=-60) at the very edge of iiwa7 reach -> move-arm pickup IK fails mid-plan. -52
        # pulls the layout ~11cm toward the arms (parts land ~y in [-49, -30], still well beyond
        # the assembly at y=-15); every pickup IK then solves, and run_motion_plan's pickup IK
        # is made collision-aware (inverse_kinematics_collision_free) so the config it returns is
        # actually free of the fixture / neighbouring parts.
        #
        # EXPERIMENT (2026-09-01): -52 -> -60. KUKA bases stay at y=10 (where every grasp in
        # grasps.pkl was solved -- grasp.arm_pos = (±42, 10, 2.5)). -60 puts the fixture
        # footprint at y in [-60, -40], i.e. its far edge ~70 cm from the base and its near
        # edge ~50 cm, keeping the whole fixture inside iiwa7 reach while pushing it as far
        # from the assembly (y=-15) / the other arm as the reach budget allows. Regenerate
        # fixture (--markers aruco) + plan and check for a collision-free motion.pkl.
        #
        # -60: move-arm pickup IK fails at step 2 (part 0, ~67 cm from the base) before any
        #      path planning -- past the reach/collision wall.
        # -55: pickup IK all solves, but move part-3 transport comes back collision:True and
        #      move part-0 switch's start config is in collision with no RRT escape (hang).
        # -52: the shortfix value -- every pickup IK solves, the only residual collision is a
        #      hold-arm regrasp switch, which the plan_path_switch active-part-exclusion fix
        #      in run_motion_plan.py targets directly.
        return -52.0
    elif arm_type == 'ur5e':
        return 6 * dx
    else:
        raise NotImplementedError
