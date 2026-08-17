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
ASSEMBLY=${POSITIONAL[1]}
ASSEMBLY_DIR=${POSITIONAL[2]:-fabrica}

# render_motion_plan.py needs redmax_py (from `pip install ./simulation`), which lives in
# the main $SIM_ENV. render_traj_blender.py needs bpy, which only installs on Python
# 3.13+ and lives in the separate $RENDER_ENV. See README.md > Installation > 4.
SIM_ENV=${SIM_ENV:-fabrica}
RENDER_ENV=${RENDER_ENV:-fabrica-render}

if [ "$HEADLESS" == true ]; then
    xvfb-run -s "-screen 0 1920x1080x24" conda run -n "$SIM_ENV" --no-capture-output python rendering/render_motion_plan.py --assembly-dir assets/$ASSEMBLY_DIR/$ASSEMBLY --log-dir logs/$EXP_NAME/$ASSEMBLY
    xvfb-run -s "-screen 0 1920x1080x24" conda run -n "$RENDER_ENV" --no-capture-output python rendering/render_traj_blender.py --assembly-dir assets/$ASSEMBLY_DIR/$ASSEMBLY --log-dir logs/$EXP_NAME/$ASSEMBLY --record-path records/blender/$EXP_NAME/${ASSEMBLY}.mp4 --verbose
else
    conda run -n "$SIM_ENV" --no-capture-output python rendering/render_motion_plan.py --assembly-dir assets/$ASSEMBLY_DIR/$ASSEMBLY --log-dir logs/$EXP_NAME/$ASSEMBLY
    conda run -n "$RENDER_ENV" --no-capture-output python rendering/render_traj_blender.py --assembly-dir assets/$ASSEMBLY_DIR/$ASSEMBLY --log-dir logs/$EXP_NAME/$ASSEMBLY --record-path records/blender/$EXP_NAME/${ASSEMBLY}.mp4 --verbose
fi
