# 01 — Concepts

Quick mental model before you dive into code.

## What cuMotion is

- An NVIDIA library (`isaac_ros_cumotion`, wrapping `cuRobo`) that performs GPU-accelerated motion planning
- Supports joint-space goals (via `plan_single_js`) and pose goals (via `plan_single`)
- Output: smooth minimum-jerk trajectories (joint positions + time)
- Plans in sub-second on Jetson Orin / desktop GPUs
- Integrates with MoveIt 2 via a plugin (`isaac_ros_cumotion_moveit`)

## cuMotion's core abstractions

| Concept | Purpose |
|---|---|
| **XRDF** | cuRobo's robot description file. YAML. Specifies cspace (actuated joints), sphere collision geometry per link, and self-collision ignore pairs. |
| **Sphere model** | Every link's collision geometry is a set of spheres. cuRobo runs hundreds of sphere-sphere overlap tests per evaluation. Fast but approximate. |
| **MotionGen** | Top-level planner object. Wraps graph search + TrajOpt. Compiled once at startup. |
| **plan_single_js** | Joint-space goal planning. Input: joint state + goal joint state. Output: joint trajectory. |
| **plan_single** | Pose goal planning (target end-effector pose, find IK + plan). |
| **ESDF** | Euclidean Signed Distance Field for world (environment) collision. Populated from nvblox, cuboid primitives, or attached mesh objects. |

## How cuMotion plans

1. **Graph search** (fast, ~50ms): samples points in cspace, connects valid ones into a roadmap, finds a seed trajectory
2. **TrajOpt** (slow, 200ms-5s): gradient-based optimization of seed trajectory. Minimizes jerk while enforcing collision + joint limits
3. **Time parameterization** (internal, optional): assigns timings to waypoints
4. Returns a dense trajectory ready for execution

## Relation to MoveIt

MoveIt 2 is the ROS 2 motion planning framework. cuMotion hooks in two ways:

- **As a MoveIt plugin** (`isaac_ros_cumotion_moveit/CumotionPlanner`): invoked by move_group when your `planning_pipeline` is set to `isaac_ros_cumotion`. Passes requests to the cumotion action server.
- **Direct action server** (`/cumotion/move_group`): bypasses the plugin entirely. Useful when the plugin's hardcoded 5s timeout blocks lazy compilation.

See [05_moveit_integration.md](05_moveit_integration.md) for when to use each.

## Limitations you must know

| Limitation | Consequence |
|---|---|
| Sphere collision only | 30-40mm false-positive zone around flat mesh faces. Corridors narrower than that fail. |
| No URDF mimic joint support | Mimic joints are treated as fixed at joint=0. Workarounds in [04_sphere_fitting.md](04_sphere_fitting.md#mimic-joint-limitation). |
| Reads XRDF as ASCII | Non-ASCII chars (em-dash, arrows) crash the load. |
| `plan_single_js` CUDA graph compiles lazily | First call takes ~25s. See [08_debugging.md](08_debugging.md#timed_out--warmup) for the warmup patch. |
| MoveIt2 Humble FCL thread race | `move_group` SIGSEGVs under concurrent planning. Switch to Bullet collision detector. |

## Relation to OMPL

OMPL (Open Motion Planning Library) is a general-purpose sampling-based planner library built into MoveIt 2. It's slow but can solve problems cuMotion can't (tight mesh corridors, dense obstacles).

The production pattern is **cuMotion primary, OMPL fallback**. See [06_ompl_fallback.md](06_ompl_fallback.md).

## What a cuMotion-ready robot looks like

- URDF with velocity + acceleration limits per joint (for time parameterization)
- SRDF with planning group + disable_collisions (for OMPL + FCL post-check)
- XRDF with sphere geometry + ignore list (for cuMotion)
- MoveIt config with both `isaac_ros_cumotion` and `ompl` pipelines loaded
- `collision_detector: Bullet` (MoveIt2 Humble) to avoid FCL SIGSEGVs
