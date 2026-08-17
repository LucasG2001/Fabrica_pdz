#!/bin/bash

HEADLESS=false
POSITIONAL=()
for arg in "$@"; do
    if [ "$arg" == "--headless" ] || [ "$arg" == "headless" ]; then
        HEADLESS=true
    else
        POSITIONAL+=("$arg")
    fi
done

EXP_NAME=${POSITIONAL[0]}
ASSEMBLY_DIR=${POSITIONAL[1]:-fabrica}

# render_motion_plan_batch.py needs redmax_py (from `pip install ./simulation`), which
# lives in the main $SIM_ENV. render_traj_blender_batch.py needs bpy, which only installs
# on Python 3.13+ and lives in the separate $RENDER_ENV. See README.md > Installation > 4.
SIM_ENV=${SIM_ENV:-fabrica}
RENDER_ENV=${RENDER_ENV:-fabrica-render}

if [ "$HEADLESS" == true ]; then
    xvfb-run -s "-screen 0 1920x1080x24" conda run -n "$SIM_ENV" --no-capture-output python rendering/render_motion_plan_batch.py --assembly-dir assets/$ASSEMBLY_DIR --log-dir logs/$EXP_NAME --num-proc 12
    xvfb-run -s "-screen 0 1920x1080x24" conda run -n "$RENDER_ENV" --no-capture-output python rendering/render_traj_blender_batch.py --assembly-dir assets/$ASSEMBLY_DIR --log-dir logs/$EXP_NAME --record-dir records/blender/$EXP_NAME --num-proc 12 --keep-img --interval 2
else
    conda run -n "$SIM_ENV" --no-capture-output python rendering/render_motion_plan_batch.py --assembly-dir assets/$ASSEMBLY_DIR --log-dir logs/$EXP_NAME --num-proc 12
    conda run -n "$RENDER_ENV" --no-capture-output python rendering/render_traj_blender_batch.py --assembly-dir assets/$ASSEMBLY_DIR --log-dir logs/$EXP_NAME --record-dir records/blender/$EXP_NAME --num-proc 12 --keep-img --interval 2
fi
