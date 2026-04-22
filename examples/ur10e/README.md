# UR10e Example

A minimal cuMotion + OMPL fallback setup for a plain Universal Robots UR10e (6-DOF, no tool, no base).

Starting point for adapting to your own robot. Use as a template.

## Dependencies

- ROS 2 Humble
- MoveIt 2 Humble
- `isaac_ros_cumotion` and dependencies (see [../../docs/02_setup.md](../../docs/02_setup.md))
- `ur_description` (public Universal Robots URDF)
  ```bash
  sudo apt install ros-humble-ur-description ros-humble-ur-moveit-config
  ```

## Files in this example

```
ur10e/
├── README.md                      (this file)
├── xrdf/
│   └── ur10e.xrdf                 (collision spheres for the 6 arm links)
├── config/
│   ├── cumotion_params.yaml       (cuMotion tuning for UR10e)
│   ├── ompl_planning.yaml         (OMPL fallback config)
│   └── moveit_controllers.yaml
├── launch/
│   ├── cumotion.launch.py         (cuMotion only)
│   ├── fallback.launch.py         (cuMotion + OMPL)
│   └── validation.launch.py       (benchmark runner)
└── scripts/
    └── simple_plan_example.py     (minimal Python client using the dispatcher)
```

## Quick start

### Visualize first (no planning, no cuMotion needed)

Verify XRDF spheres align with URDF meshes:

```bash
ros2 launch $PWD/launch/visualize.launch.py
```

Opens RViz with:
- UR10e robot model (from public `ur_description`)
- XRDF collision spheres as `MarkerArray` on `/xrdf_spheres`
- Joint sliders GUI — drive the arm through its range, confirm spheres track

### Plan with cuMotion + OMPL

1. Build: `colcon build --packages-select <your_pkg_wrapping_this_example>`
2. Launch: `ros2 launch <your_pkg> fallback.launch.py`
3. Test plan from Python:
   ```bash
   ros2 run <your_pkg> simple_plan_example
   ```

## XRDF notes

The `xrdf/ur10e.xrdf` in this example uses pre-fit spheres for the 6 UR10e arm links, generated with VAMP/FOAM from the UR10e collision meshes. Radii are tight enough to succeed on most random goal pairs but loose enough to cover the actual mesh.

If you need tighter spheres for your environment:
```bash
python ../../tools/tighten_xrdf_from_urdf.py \
    --urdf /opt/ros/humble/share/ur_description/urdf/ur10e.urdf \
    --xrdf-in xrdf/ur10e.xrdf \
    --xrdf-out xrdf/ur10e_tight.xrdf \
    --package-root ur_description=/opt/ros/humble/share/ur_description
```

## Adapting to your own robot

1. Replace the URDF with your robot's
2. Generate a new XRDF (see [`docs/04_sphere_fitting.md`](../../docs/04_sphere_fitting.md))
3. Update `config/cumotion_params.yaml` (especially `tool_frame` and `num_trajopt_time_steps`)
4. Update the launch file paths
5. Update `simple_plan_example.py` with your planning group name and joint names

## Expected performance

On an RTX 3050 with this unmodified setup:
- cuMotion plan time: 0.2-0.8s per call
- OMPL (RRTConnect) plan time: 0.1-0.5s per call
- Dispatcher success rate on random pinocchio-validated joint-space pairs: ~80-90% combined

These numbers drop if you add a complex tool at the flange (see examples that include end-effectors).
