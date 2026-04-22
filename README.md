# cuMotion Deployment Toolkit

A practical guide + tooling for deploying NVIDIA Isaac cuMotion on real robots.

## What is this

cuMotion is fast (sub-second joint-space plans on Jetson Orin) but requires careful setup — sphere-based self-collision, no URDF mimic-joint support, MoveIt plugin quirks, GPU memory fragility. This repo captures the non-obvious lessons so you can skip the trial-and-error.

## When to use cuMotion

- **Joint-space planning** on 6-7 DOF manipulators where you need sub-second replans
- **Self-collision + environment collision** via nvblox ESDF or cuboid/mesh primitives
- **GPU-accelerated optimization** (not sampling) — trajectories are smooth minimum-jerk

## When NOT to use cuMotion alone

- Problems where sampling helps (threading the arm through narrow slots between obstacles)
- Tight mesh corridors — sphere approximation blocks ~30-40mm before real mesh boundaries
- Production deployments where any single failure is unacceptable

## The production pattern: cuMotion + OMPL fallback

The single most important takeaway. Neither planner alone is enough. cuMotion is your fast path; OMPL is the safety net:

```
try cuMotion (N attempts)
├── if COLLISION_CHECKING_UNAVAILABLE → fallback to OMPL
├── if START_STATE_IN_COLLISION (sphere false positive) → fallback to OMPL
├── if move_group SIGSEGV → permanently switch to OMPL for session
└── if all N fail (TRAJOPT_FAIL/PLANNING_FAILED) → OMPL last-resort with more time
```

See [`docs/06_ompl_fallback.md`](docs/06_ompl_fallback.md) for the full dispatch pattern with code.

## Guide index

| | |
|---|---|
| [01 Concepts](docs/01_concepts.md) | Sphere model, XRDF, cuRobo vs MoveIt, limitations |
| [02 Setup](docs/02_setup.md) | Install `isaac_ros_cumotion`, CUDA deps, ROS2 Humble |
| [03 XRDF authoring](docs/03_xrdf_authoring.md) | Ignore list, sphere generation, mesh_link overrides |
| [04 Sphere fitting](docs/04_sphere_fitting.md) | Sphere generation, frame transforms, mimic joint workarounds |
| [05 MoveIt integration](docs/05_moveit_integration.md) | Plugin vs direct `/cumotion/move_group`, timeout bypass |
| [06 OMPL fallback](docs/06_ompl_fallback.md) | Hybrid dispatch pattern (code + rationale) |
| [07 Validation](docs/07_validation.md) | Dataset benchmarking, FCL post-check, blocker diagnosis |
| [08 Debugging](docs/08_debugging.md) | TRAJOPT_FAIL, DT_EXCEPTION, start-collision, SIGSEGV |
| [09 Performance tuning](docs/09_performance.md) | GPU-specific tuning (3050 vs Orin), seeds, timesteps |
| [10 Lessons learned](docs/10_lessons.md) | Curated takeaways |

## Tools

| Script | Purpose |
|---|---|
| `tools/validate_dataset.py` | Run cuMotion/OMPL over a pair dataset, record results |
| `tools/fcl_validate_trajectories.py` | FCL mesh-level post-validate saved trajectories |
| `tools/replay_trajectories.py` | Visualize saved trajectories in RViz |
| `tools/sphere_blocker_analysis.py` | Identify specific spheres blocking specific pairs |
| `tools/generate_spheres_from_mesh.py` | Sphere generation with correct frame handling |
| `tools/compare_planners.py` | Side-by-side cuMotion vs OMPL benchmark |

## Templates

Ready-to-adapt launch files and config:

- `templates/launch/cumotion_only.launch.py`
- `templates/launch/cumotion_with_ompl_fallback.launch.py`
- `templates/launch/validation.launch.py`
- `templates/config/cumotion_params.yaml` (annotated with tuning notes)
- `templates/config/ompl_planning.yaml`
- `templates/config/xrdf_template.xrdf`
- `templates/scripts/cumotion_planner_patches.py`

## Examples

- `examples/ur10e/` — Plain UR10e example (Universal Robots arm, publicly available URDF)

## License

MIT.
