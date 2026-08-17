#!/bin/bash

EXP_NAME=${1-sr_genall}
FRICTION=${2:-1}
CKPT=${3:-""}
GPU=${4:-0}

export CUDA_VISIBLE_DEVICES="$GPU"
export NUM_ENVS=1024

# List of assemblies
ASSEMBLIES=("beam" "car" "cooling_manifold" "duct" "gamepad" "plumbers_block" "stool_circular")

# Log file
LOG_FILE="${EXP_NAME}.log"

# Ensure the log file exists (creates if it doesn't)
touch "$LOG_FILE"

# Loop through each assembly
for ASSEMBLY in "${ASSEMBLIES[@]}"; do
    # Log both to console and file
    echo "Running evaluation for assembly: $ASSEMBLY" | tee -a "$LOG_FILE"

    # Redirect stdout and stderr to the log file
    python train.py task=FabricaFixPlugTaskAssemble \
        task.env.assemblies=["$ASSEMBLY"] \
        task.env.numEnvs=$NUM_ENVS \
        max_iterations=1 \
        headless=True \
        test=True \
        task.env.kuka_friction=$FRICTION \
        checkpoint=$CKPT \
        >> "$LOG_FILE" 2>&1
done
