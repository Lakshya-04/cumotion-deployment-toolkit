# Quickstart Tutorial

Get from zero to a planned trajectory in ~15 minutes, assuming you have ROS 2 Humble + MoveIt 2 already installed.

## Step 1 — Clone with submodules

```bash
git clone --recursive https://github.com/Lakshya-04/cumotion-deployment-toolkit.git
cd cumotion-deployment-toolkit
```

The `--recursive` pulls both submodules:
- `isaac_ros_cumotion/` — NVIDIA's cuMotion with our 4 production patches applied on top of `release-3.2`
- `nvblox-pipeline/` — companion nvblox toolkit (optional, for environment collision)

## Step 2 — Install toolkit dependencies

```bash
pip install -r requirements.txt
# Optional (best sphere quality):
pip install git+https://github.com/CoMMALab/foam
```

## Step 3 — Build `isaac_ros_cumotion` (patched fork)

The submodule is a standard ROS 2 colcon workspace. Build it alongside your other ROS 2 packages:

```bash
# Assuming you have ~/ros2_ws as your workspace
cp -r isaac_ros_cumotion ~/ros2_ws/src/isaac_ros_cumotion
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
    --packages-select isaac_ros_cumotion isaac_ros_cumotion_moveit \
                      isaac_ros_cumotion_python_utils isaac_ros_cumotion_interfaces \
                      isaac_ros_cumotion_robot_description
source install/setup.bash
```

(Or if you already have upstream `isaac_ros_cumotion` built, apply patches via `templates/scripts/cumotion_planner_patches.py` instead.)

## Step 4 — Install a robot

The UR10e example uses the public Universal Robots packages:

```bash
sudo apt install ros-humble-ur-description ros-humble-ur-moveit-config
```

## Step 5 — Run MoveIt + cuMotion

Terminal 1 — MoveIt with the UR10e:
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10e launch_rviz:=true
```

Terminal 2 — cuMotion node pointing at our example XRDF:
```bash
ros2 launch $PWD/examples/ur10e/launch/fallback.launch.py
```

Terminal 3 — verify the XRDF sphere placement:
```bash
python3 tools/xrdf_sphere_publisher.py \
    --ros-args -p xrdf_path:=$PWD/examples/ur10e/xrdf/ur10e.xrdf
```

In RViz, add a `MarkerArray` display on `/xrdf_spheres`. You should see green spheres following the UR10e's links. Move joints via the `joint_state_publisher_gui` to confirm they track.

## Step 6 — Plan something

```bash
python3 tools/send_joint_goal.py \
    --planning-group manipulator \
    --joint-names shoulder_pan_joint shoulder_lift_joint elbow_joint \
                  wrist_1_joint wrist_2_joint wrist_3_joint \
    --start 0 -1.57 1.57 0 1.57 0 \
    --goal  1.0 -1.2 1.8 -0.5 1.2 0.5 \
    --replay
```

Expected: `PLAN SUCCESS — N waypoints`. RViz will show the arm moving along the trajectory.

If it fails, see [`docs/08_debugging.md`](docs/08_debugging.md).

## Step 7 — Benchmark on a dataset

```bash
python3 tools/validate_dataset.py \
    --dataset examples/ur10e/pairs_example.json \
    --out-dir results/ \
    --planning-group manipulator \
    --arm-joints shoulder_pan_joint shoulder_lift_joint elbow_joint \
                  wrist_1_joint wrist_2_joint wrist_3_joint \
    --pipeline cumotion --save-trajectories
```

Output: `results/run_cumotion_<timestamp>.json` with per-pair success, planning time, and trajectories.

## Step 8 — FCL-validate the successes

```bash
python3 tools/fcl_validate_trajectories.py \
    --run results/run_cumotion_*.json \
    --planning-group manipulator
```

This post-validates the saved trajectories with FCL mesh collision. If any come back DIRTY, the XRDF is under-approximating — iterate sphere tightness (see [`docs/04_sphere_fitting.md`](docs/04_sphere_fitting.md)).

## Step 9 — Replay later

```bash
python3 tools/replay_trajectories.py --run results/run_cumotion_*.json --loops 2
```

No re-planning needed. The saved JSON has everything.

## Next steps

- Read [`docs/06_ompl_fallback.md`](docs/06_ompl_fallback.md) to add the OMPL safety net
- Read [`docs/10_lessons.md`](docs/10_lessons.md) for the production-deployment gotchas
- Adapt [`examples/ur10e/`](examples/ur10e/) to your own robot — replace URDF, XRDF, joint names
- For environment (world) collision, see [`nvblox-pipeline/`](nvblox-pipeline/) submodule
