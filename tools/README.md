# Tools

Standalone Python scripts. No ROS package required for build — just install deps and run.

## Dependencies

```bash
pip install numpy trimesh scikit-learn pyyaml
# Optional (better sphere quality):
pip install git+https://github.com/CoMMALab/foam
pip install vamp
# Required for ROS-based tools (validate_dataset, fcl_validate, replay):
# ROS 2 Humble with rclpy available
```

## What each tool does

| Tool | Purpose | Needs ROS running? |
|---|---|---|
| `generate_spheres_from_mesh.py` | Create spheres for a single mesh (FOAM/VAMP/k-means) | No |
| `tighten_xrdf_from_urdf.py` | Bulk refit every link's spheres in an XRDF against URDF meshes | No |
| `validate_dataset.py` | Run cuMotion/OMPL over a pair dataset, save results + trajectories | Yes (move_group + cumotion) |
| `fcl_validate_trajectories.py` | Post-validate saved trajectories against FCL mesh collision | Yes (move_group) |
| `replay_trajectories.py` | Publish saved trajectories on `/joint_states` for RViz replay | No (RViz optional) |
| `sphere_blocker_analysis.py` | Find which XRDF spheres block the most pairs | No |

## Typical workflow

```bash
# 1. Generate initial XRDF spheres (if you don't have one)
python generate_spheres_from_mesh.py --mesh robot_arm.stl --method foam --n 40 \
    --out spheres_arm.json

# 2. Bulk refit an existing XRDF against URDF
python tighten_xrdf_from_urdf.py \
    --urdf robot.urdf --xrdf-in robot.xrdf --xrdf-out robot_tight.xrdf \
    --package-root my_robot=/path/to/my_robot_pkg

# 3. Launch your MoveIt + cumotion stack, then benchmark
python validate_dataset.py --dataset pairs.json --out-dir results/ \
    --planning-group manipulator --arm-joints <list> \
    --pipeline cumotion --save-trajectories

# 4. Post-validate with FCL mesh check
python fcl_validate_trajectories.py --run results/run_cumotion_*.json \
    --planning-group manipulator

# 5. Analyze blockers (helps decide what to shrink)
python sphere_blocker_analysis.py \
    --dataset pairs.json --xrdf robot.xrdf --urdf robot.urdf \
    --joint-names <joint names> --top 20

# 6. Replay for visual inspection
python replay_trajectories.py --run results/run_cumotion_*.json --loops 2
```

## Input/output formats

All ROS-aware tools use the same dataset JSON format:

```json
{
  "pairs": [
    {"id": "pair_000000", "start": [j1, ..., jN], "goal": [j1, ..., jN]},
    ...
  ]
}
```

All save trajectory info as:

```json
{
  "per_pair": [
    {
      "id": "pair_000000",
      "success": true,
      "n_pts": 32,
      "time_s": 0.43,
      "joint_names": [...],
      "trajectory": [
        {"t": 0.0, "positions": [...]},
        ...
      ]
    }
  ]
}
```
