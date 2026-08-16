EXP_NAME=$1
GPU=${2:-0}
FRICTION=${3:-1}

export CUDA_VISIBLE_DEVICES="$GPU"
export NUM_ENVS=1024
export MAX_ITER=2000

python train.py task=FabricaFixPlugTaskAssemble experiment="$EXP_NAME" task.env.numEnvs=$NUM_ENVS max_iterations=$MAX_ITER headless=True task.env.kuka_friction=$FRICTION
