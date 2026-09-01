FINGER_BUFFER = 0.25  # buffer distance for robot finger collision checking (cm)
HAND_KNUCKLE_BUFFER = 1.0  # buffer distance for robot hand knuckle collision checking (cm)
ARM_BUFFER = 1.0  # buffer distance for robot arm collision checking (cm)

RETRACT_OPEN_RATIO = 0.1  # extra open ratio for retract grasp
RETRACT_DELTA_NEAR = 1.0 # incremental distance for retract grasp when near
RETRACT_DELTA_FAR = 9.0 # incremental distance for retract grasp when far away
# 2026-09-01: 5.0 -> 9.0. The switch / transport retract sub-paths back the arm out along the
# flange approach axis (~"up and back" for a downward reach) by this much before the main
# plan; the plumbers_block holder-regrasp switch and the inserter -> rest_q retracts were
# grazing the growing sub-assembly. Bigger back-off before traversing. RETRACT_DELTA_NEAR
# (assembly insertions) unchanged.

CHECK_GRIPPERS_INTERLOCK = False # whether to check grippers interlock during dual-arm planning

OPEN_RATIO_REST = 0.5 # open ratio for resting position
