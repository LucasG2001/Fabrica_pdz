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


def get_move_arm_box(arm_type):
    arm_pos = get_move_arm_pos(arm_type)
    return arm_pos - np.array([100.0, 100.0, 0.0]), arm_pos + np.array([30.0, 50.0, 80.0])


def get_hold_arm_box(arm_type):
    arm_pos = get_hold_arm_pos(arm_type)
    return arm_pos - np.array([30.0, 100.0, 0.0]), arm_pos + np.array([100.0, 50.0, 80.0])


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
        return 4 * dx
    elif arm_type == 'ur5e':
        return 6 * dx
    else:
        raise NotImplementedError
