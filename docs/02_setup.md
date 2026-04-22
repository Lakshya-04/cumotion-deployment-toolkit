# 02 — Setup

Install cuMotion, dependencies, and the companion tooling in this repo.

## System requirements

| Requirement | Tested versions |
|---|---|
| OS | Ubuntu 22.04 (for ROS 2 Humble), Ubuntu 24.04 (ROS 2 Jazzy, untested here but should work) |
| GPU | CUDA-capable. RTX 3050 minimum; Jetson Orin preferred for deployment |
| CUDA | 11.8+ (cuRobo requires it) |
| Python | 3.10+ |
| ROS 2 | Humble (production tested). Jazzy should work but not verified. |
| MoveIt 2 | Matching ROS 2 distribution |

## Install ROS 2 Humble + MoveIt 2

Follow the official guides:
- ROS 2 Humble: https://docs.ros.org/en/humble/Installation.html
- MoveIt 2 Humble: https://moveit.ros.org/install-moveit2/binary/

Quick check:
```bash
ros2 topic list    # should not error
ros2 pkg prefix moveit_ros_move_group   # should return a path
```

## Install cuRobo (cuMotion backend)

cuRobo is the GPU library powering cuMotion. Install via pip with CUDA support:

```bash
# Recommended: clean venv
python3 -m venv ~/curobo-venv
source ~/curobo-venv/bin/activate

# Install torch matching your CUDA version (CUDA 11.8 shown)
pip install torch --index-url https://download.pytorch.org/whl/cu118

# Install cuRobo
pip install curobo
```

Verify:
```bash
python -c "from curobo.wrap.reacher.motion_gen import MotionGen; print('ok')"
```

## Install isaac_ros_cumotion

The ROS 2 wrapper. From source (recommended for patching):

```bash
cd ~/ros2_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select isaac_ros_cumotion isaac_ros_cumotion_moveit
source install/setup.bash
```

Verify:
```bash
ros2 pkg executables isaac_ros_cumotion
# expect: cumotion_planner_node
```

## Apply the planner patches

Out-of-the-box `cumotion_planner_node` has the warmup and `maximum_trajectory_dt` issues documented in [08_debugging.md](08_debugging.md). Apply patches:

```bash
python3 cumotion-deployment-toolkit/templates/scripts/cumotion_planner_patches.py \
    ~/ros2_ws/install/isaac_ros_cumotion/lib/python3.10/site-packages/isaac_ros_cumotion/cumotion_planner.py
```

(or adjust path to wherever your build put `cumotion_planner.py`).

The script is idempotent — safe to re-run after package updates.

## Install FOAM / VAMP for sphere generation

FOAM is the gold-standard sphere packing. VAMP wraps it.

```bash
pip install git+https://github.com/CoMMALab/foam
# or
pip install vamp   # if available on PyPI; otherwise build from source
```

If either won't install (build system issues, etc.), the toolkit falls back to k-means — no capability is gated behind FOAM. See [04_sphere_fitting.md](04_sphere_fitting.md).

## Install this toolkit

```bash
git clone https://github.com/Lakshya-04/cumotion-deployment-toolkit.git ~/cumotion-deployment-toolkit
cd ~/cumotion-deployment-toolkit
pip install -r tools/requirements.txt   # (if present; else install trimesh, numpy, sklearn, pyyaml manually)
```

No ROS package structure — the scripts are standalone.

## Optional: nvblox for environment collision

If your cuMotion deployment needs live environment collision (vs pure self-collision), install nvblox. Usually a separate pipeline — see the companion [`nvblox-pipeline-toolkit`](https://github.com/Lakshya-04/nvblox-pipeline-toolkit) repo (coming).

## Test with the UR10e example

```bash
# 1. Install UR description (if not already)
sudo apt install ros-humble-ur-description ros-humble-ur-moveit-config

# 2. Launch MoveIt with UR10e
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10e launch_rviz:=true

# 3. In a second terminal, launch the cumotion node
ros2 launch $(pwd)/examples/ur10e/launch/fallback.launch.py

# 4. Run the example plan
python3 examples/ur10e/scripts/simple_plan_example.py
```

If the plan succeeds you're good. If it fails at warmup, check [08_debugging.md](08_debugging.md#timed_out--warmup).
