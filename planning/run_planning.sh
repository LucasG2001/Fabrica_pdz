#!/bin/bash

EXP_NAME=$1
ASSEMBLY=$2
SETUP=${3:-kuka}
ASSEMBLY_DIR=${4:-fabrica}

ARM=""
GRIPPER=""
FT_SENSOR=""
# Nullspace IK regularization pulling the solver toward rest_q (see run_grasp_arm_gen.py
# --ik-regularization, default 1.0). That default was never re-tuned for KUKA's chain: at 1.0 it
# over-constrains the solver and every KUKA grasp candidate fails IK even when the target is
# reachable (verified via planning/utils/debug_grasp_headless.py -- 0/13 geometrically-clean
# plumbers_block candidates converge at 1.0, vs 5/13 at 0.1). 0.1 keeps some regularization
# (smoother joint trajectories) while letting the solver actually reach KUKA targets.
IK_REGULARIZATION=1.0

if [ "$SETUP" == "kuka" ]; then
  ARM="kuka"
  GRIPPER="kuka"
  FT_SENSOR="none"
  IK_REGULARIZATION=0.1
elif [ "$SETUP" == "panda" ]; then
  ARM="panda"
  GRIPPER="panda"
  FT_SENSOR="none"
elif [ "$SETUP" == "panda-robotiq" ]; then
  ARM="panda"
  GRIPPER="robotiq-140"
  FT_SENSOR="none"
elif [ "$SETUP" == "xarm7" ]; then
  ARM="xarm7"
  GRIPPER="robotiq-140"
  FT_SENSOR="move"
elif [ "$SETUP" == "ur5e" ]; then
  ARM="ur5e"
  GRIPPER="robotiq-85"
  FT_SENSOR="none"
else
  echo "Error: Unsupported SETUP value '$SETUP'. Please use 'kuka', 'panda', 'xarm7', or 'ur5e'."
  exit 1
fi

export OMP_NUM_THREADS=1

echo "Running precedence and path planning..."
python planning/run_preced_plan.py --assembly-dir assets/$ASSEMBLY_DIR/$ASSEMBLY --log-dir logs/$EXP_NAME/$ASSEMBLY --num-proc 12 --arm $ARM

echo "Running grasp and arm IK generation..."
python planning/run_grasp_arm_gen.py --assembly-dir assets/$ASSEMBLY_DIR/$ASSEMBLY --log-dir logs/$EXP_NAME/$ASSEMBLY --num-proc 50 --max-n-grasp 100 --arm $ARM --gripper $GRIPPER --ft-sensor $FT_SENSOR --ik-regularization $IK_REGULARIZATION

echo "Running sequence planning..."
python planning/run_seq_plan.py --assembly-dir assets/$ASSEMBLY_DIR/$ASSEMBLY --log-dir logs/$EXP_NAME/$ASSEMBLY --plot

echo "Running sequence optimization..."
python planning/run_seq_opt.py --log-dir logs/$EXP_NAME/$ASSEMBLY --plot

echo "Running fixture generation..."
python planning/run_fixture_gen.py --assembly-dir assets/$ASSEMBLY_DIR/$ASSEMBLY --log-dir logs/$EXP_NAME/$ASSEMBLY --optimized

echo "Running complete motion planning..."
python planning/run_motion_plan.py --assembly-dir assets/$ASSEMBLY_DIR/$ASSEMBLY --log-dir logs/$EXP_NAME/$ASSEMBLY --optimized

echo "Done."
