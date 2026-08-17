# 🏭 Fabrica

<p>
  <strong>Fabrica: Dual-Arm Assembly of General Multi-Part Objects via Integrated Planning and Learning</strong><br>
  <strong>[CoRL 2025, Best Paper Award]</strong>
</p>

<a href="http://fabrica.csail.mit.edu">
  <img src="https://img.shields.io/badge/project-website-green.svg" alt="Project Page"/>
</a> <a href="https://arxiv.org/abs/2506.05168">
  <img src="https://img.shields.io/badge/paper-arXiv-b31b1b.svg" alt="arXiv Paper"/>
</a> <a href="https://opensource.org/licenses/MIT">
  <img src="https://img.shields.io/badge/license-MIT-yellow.svg" alt="License: MIT"/>
</a> <a href="https://x.com/YunshengTian/status/1971853081504878748">
  <img src="https://img.shields.io/badge/twitter-YunshengTian-blue.svg" alt="Twitter"/>
</a> <a href="https://fabrica.csail.mit.edu/static/videos/video_main.mp4">
  <img src="https://img.shields.io/badge/video-overview-orange.svg" alt="Intro Video"/>
</a> <a href="https://www.youtube.com/live/rh2oxU1MCb0?t=21118s">
  <img src="https://img.shields.io/badge/video-talk-orange.svg" alt="Talk Video"/>
</a>

<p>
  <img src="images/teaser.gif" alt="Fabrica teaser" width="640">
</p>

**Fabrica** is an autonomous robotic assembly system capable of planning and executing multi-step contact-rich assembly of general objects without human demonstrations.

## 🔧 Installation

### 1. Clone repository

```bash
git clone --recurse-submodules git@github.com:yunshengtian/Fabrica.git
```

### 2. Create Python environment

```bash
conda env create -f environment.yml
conda activate fabrica
```

or

```bash
sudo apt-get install graphviz graphviz-dev # for linux
brew install graphviz # for mac
pip install -r requirements.txt
```

### 3. Install simulation for planning

```bash
pip install ./simulation
```

> **Known issue:** the `simulation/externals/libigl` and `simulation/externals/pybind11` submodule gitlinks are missing from this repo's history, so `git submodule update --init` finds nothing and the CMake build fails (`add_subdirectory given source "libigl"/"pybind11" which is not an existing directory`). Until this is fixed upstream, work around it by cloning them manually:
>
> ```bash
> cd simulation/externals
> git clone --recursive https://github.com/pybind/pybind11 pybind11
> git clone --recursive https://github.com/libigl/libigl libigl
> cd libigl && git checkout v2.4.0 && cd ..
> ```
>
> The `libigl v2.4.0` pin matters: newer libigl versions use `Eigen::all`/`.reshaped()`, which require Eigen >= 3.4, but this repo bundles Eigen 3.3.4 directly under `simulation/externals/eigen`, so newer libigl fails to compile against it. We will fix this properly later (either commit real submodule gitlinks or bump the bundled Eigen).

To test if the installation steps are successful, run:

```bash
python simulation/test/test_simple_sim.py --model box/box_stack
```

[Here](https://github.com/yunshengtian/Assemble-Them-All?tab=readme-ov-file#simulation-viewer) are some tips on interacting with the simulation viewer. 

To visualize the simulation of our beam assembly in the benchmark, run:

```bash
python simulation/test/test_multi_sim.py --dir fabrica --id beam
```

### 4. Install renderer (optional)

The renderer depends on `bpy` (Blender-as-a-Python-module) and `mathutils`, whose PyPI wheels only support **Python 3.13+** — not the Python 3.10 used by the main `fabrica` env (`environment.yml`). Install it into its own conda environment instead of the main one:

```bash
conda create -n fabrica-render python=3.13
conda activate fabrica-render
pip install ./rendering
conda deactivate
```

Note: `rendering/run_rendering_blender.sh` and `rendering/run_rendering_blender_batch.sh` need **two** environments to run — `fabrica` (for `render_motion_plan*.py`, which needs `redmax_py` from step 3) and `fabrica-render` (for `render_traj_blender*.py`, which needs `bpy`). The scripts switch between them automatically via `conda run`, so just make sure both envs exist under those names (or override with `SIM_ENV=... RENDER_ENV=... bash rendering/run_rendering_blender.sh ...`); you don't need to activate anything yourself before running them. `run_rendering_opengl.sh` only needs the `fabrica` env.

### 5. Install Isaac Gym for learning

Note: It's recommended to install this step in a separate conda environment to avoid conflicts with other packages. For example:

```bash
conda create -n isaacgym python=3.8
conda activate isaacgym
```

> **Known issue:** Isaac Gym Preview 4 only ships precompiled Python bindings for 3.6/3.7/3.8 (`gym_36.so`/`gym_37.so`/`gym_38.so` under `_bindings/linux-x86_64/`) and its `setup.py` enforces `python_requires='>=3.6,<3.9'`. An env created with a newer Python (3.9+) will fail to `import isaacgym` — use 3.8.

Download the Isaac Gym Preview 4 release from the [website](https://developer.nvidia.com/isaac-gym), then follow the installation instructions in the documentation. Once Isaac Gym is installed, run:

```bash
pip install ./learning
```

> **Known issues after installing:**
> - On Ubuntu 22.04+, `import isaacgym` may fail with `ImportError: libpython3.8.so.1.0: cannot open shared object file`, even though that file exists under the conda env's own `lib/` — it's just not on the loader's search path. Fix by adding it to `LD_LIBRARY_PATH` when the env is active, e.g. via conda activate hooks:
>   ```bash
>   mkdir -p "$CONDA_PREFIX/etc/conda/activate.d" "$CONDA_PREFIX/etc/conda/deactivate.d"
>   echo 'export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"' > "$CONDA_PREFIX/etc/conda/activate.d/isaacgym_ld_library_path.sh"
>   ```
> - `pip install ./learning` pulls in `urdfpy==0.0.22`, which pins `networkx==2.2` — too old for the `numpy>=1.24` that `torch` pulls in (`AttributeError: module 'numpy' has no attribute 'int'`). Fix with `pip install networkx==2.6.3` after the install (pip's resolver will warn about the urdfpy conflict; `urdfpy` still works fine with the newer networkx).

### 6. Install kukapy for real-robot experiments

This fork's `real_robot/` scripts target a dual-arm **KUKA LBR iiwa7** rig via
[kukapy](../franka_ros2_ws/src/kukapy) — a frankapy-API-compatible ROS2
facade — instead of the original Franka Emika Panda + frankapy setup
(upstream Fabrica's [frankapy](https://github.com/iamlab-cmu/frankapy)
instructions no longer apply here; see `real_robot/robot_interface.py`'s
module docstring for the full porting notes).

kukapy lives in a separate ROS2 workspace (`franka_ros2_ws`), not this repo.
Build it there, then **source that workspace before running any
`real_robot/*.py` script** so `import kukapy` resolves:

```bash
cd ~/franka_ros2_ws
colcon build --packages-select kukapy --symlink-install
source install/setup.bash
```

Also make sure the dual-arm rig is brought up with the torque-mode
controller set kukapy needs:

```bash
ros2 launch lbr_dual_arm_bringup cartesian_impedance.launch.py
```

See `franka_ros2_ws/src/kukapy/README.md` for the full frankapy → kukapy
method-mapping table, frame conventions, and known gaps (in particular:
`get_jacobian()` is unimplemented, and `reset_joints()`'s home
configuration is not verified safe for this physical rig — read that
before running unattended).

## 💻 Experiments

### 1. Planning multi-part assembly processes

Planning consists of 6 stages: precedence planning, grasp planning, sequence planning, sequence optimization, fixture generation, and arm motion planning. We provide separate scripts for each one under `planning/` directory, but also a single bash script that automates the whole pipeline:

```bash
bash ./planning/run_planning.sh EXP_NAME ASSEMBLY_NAME
```

For parallel batch planning over multiple assemblies, run:

```bash
bash ./planning/run_planning_batch.sh EXP_NAME
```

Note: To modify the robot setup, refer to `planning/robot/workcell.py` and `planning/robot/geometry.py` to add your custom settings. 

### 2. Learning two-part assembly policies

Prepare the assets from planning output first:

```bash
bash ./learning/preprocessing/prepare_isaac.sh EXP_NAME
```

Next, enter `learning/isaacgymenvs`.

Train a specialist policy for a given assembly (e.g., beam):
```bash
python train.py task=FabricaTaskAssemble task.env.assemblies=["beam"]
```

Train a generalist policy for all assemblies:
```bash
python train.py task=FabricaTaskAssemble
```

Note: The first time you run these examples, it may take a long time for Gym to generate signed distance field representations (SDFs) for the assets. However, these SDFs will then be cached (i.e., the next time you run the same examples, it will load the previously generated SDFs).

Other useful command line arguments:
  - To run the examples **without rendering**, add: `headless=True`
  - To resume training from a specific **checkpoint**, add: `checkpoint=[path to checkpoint]`
  - To **test** a trained policy, add: `checkpoint=[path to trained policy checkpoint] test=True`
  - To change the number of parallelized environments, add: `task.env.numEnvs=[number of environments]` 
  - To set a random seed for RL training, add: `seed=-1`, to set a specific seed, add `seed=[seed number]`
  - To set maximum number of iterations for RL training, add: `max_iterations=[number of max RL training iterations]`
  - To test a policy, add: `test=True task.env.if_eval=True checkpoint=[path to trained policy checkpoint]`

### 3. Running real-robot experiments with integrated planning and learning

To prepare all parts and fixtures for 3D printing, run:

```bash
python real_robot/prepare_parts_printing.py --assembly-dir assets/fabrica --printing-dir printing
python real_robot/prepare_fixture_printing.py --log-dir logs/EXP_NAME --printing-dir printing
```

The exported STL files will be saved under `printing/ASSEMBLY_NAME/`.

Accurately calibrating the robot setup is crucial to the success of real-robot experiments. Please adjust the hardcoded transform in `pose_world_to_robot_base()` (`real_robot/robot_interface.py`) accordingly and ensure that your real-robot setup aligns well with it. (Upstream Fabrica pins this at [a specific line](https://github.com/yunshengtian/Fabrica/blob/main/real_robot/robot_interface.py#L368); this fork's copy of the file has shifted since the kukapy port, so go by the method name instead.)

To run the real-robot experiments on the dual-arm KUKA LBR rig (see "Install kukapy for real-robot experiments" above — the `franka_ros2_ws` workspace must be sourced and `cartesian_impedance.launch.py` running first), run:

```bash
python real_robot/run.py --residual --fn logs/EXP_NAME/ASSEMBLY_NAME/motion.pkl --checkpoint-path CHECKPOINT_PATH
```

`CHECKPOINT_PATH` is the path to the trained policy checkpoint.

To use VLMs for error recovery, add `--vlm` and `--video-dir VIDEO_DIR` arguments to the command, where `VIDEO_DIR` is the path to store temporary videos for VLMs to analyze.

## 🎥 Rendering

To render the assembly process after planning, we have two renderers: Blender and OpenGL. Blender is of higher quality but slower, while OpenGL is faster but lower quality.

```bash
bash rendering/run_rendering_blender.sh EXP_NAME ASSEMBLY_NAME
bash rendering/run_rendering_opengl.sh EXP_NAME ASSEMBLY_NAME
```

Note: the Blender scripts internally use both the `fabrica` and `fabrica-render` conda environments (see [Installation > 4](#4-install-renderer-optional)) — run them from any environment, they switch envs themselves.

To render in headless mode, add `--headless` argument to the command.

To render multiple assemblies in batch, run:

```bash
bash rendering/run_rendering_blender_batch.sh EXP_NAME
bash rendering/run_rendering_opengl_batch.sh EXP_NAME
```

## 📧 Contact

Please feel free to contact yunsheng@csail.mit.edu or create a GitHub issue for any questions. Due to limited maintenance bandwidth, we do not anticipate significant changes or feature enhancements to this repository; however, we hope it will serve as a useful reference and are happy to engage in discussion.

## 📚 Citation

If you find our paper, code or dataset is useful, please consider citing:

```bibtex
@inproceedings{tian2025fabrica,
  title={Fabrica: Dual-Arm Assembly of General Multi-Part Objects via Integrated Planning and Learning},
  author={Yunsheng Tian and Joshua Jacob and Yijiang Huang and Jialiang Zhao and Edward Li Gu and Pingchuan Ma and Annan Zhang and Farhad Javid and Branden Romero and Sachin Chitta and Shinjiro Sueda and Hui Li and Wojciech Matusik},
  booktitle={9th Annual Conference on Robot Learning},
  year={2025},
  url={https://openreview.net/forum?id=aSUNzvEJIf}
}
```
