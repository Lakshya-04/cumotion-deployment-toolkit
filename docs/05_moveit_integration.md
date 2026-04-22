# 05 — MoveIt Integration

Two ways to invoke cuMotion from MoveIt 2. Pick based on your deployment.

## Option A: MoveIt plugin (recommended for production)

cuMotion ships a MoveIt planning plugin (`isaac_ros_cumotion_moveit/CumotionPlanner`). When `move_group` receives a planning request with `pipeline_id: "isaac_ros_cumotion"`, the plugin forwards it to the `cumotion_planner_node` action server and returns the result through the standard MoveIt pipeline.

Benefits:
- Standard MoveIt API (clients just call `/move_action`)
- Request adapters run on the result (`AddTimeOptimalParameterization`, etc.) — important for controller-friendly timing
- FCL post-validation is built-in to MoveIt

Drawbacks:
- The plugin has a **hardcoded action client timeout** (~5s) in `libisaac_ros_cumotion_moveit.so`. First plan often exceeds this due to lazy CUDA graph compilation → `TIMED_OUT` error.

Fix: apply the warmup patch (see [`templates/scripts/cumotion_planner_patches.py`](../templates/scripts/cumotion_planner_patches.py)) which pre-compiles the CUDA graphs during node startup. After that, plans complete in <1s and fit comfortably in the 5s budget.

### Config

`isaac_ros_cumotion_planning.yaml`:
```yaml
planning_plugin: isaac_ros_cumotion_moveit/CumotionPlanner
request_adapters: >-
  default_planner_request_adapters/FixWorkspaceBounds
  default_planner_request_adapters/FixStartStateBounds
  default_planner_request_adapters/FixStartStatePathConstraints
  default_planner_request_adapters/AddTimeOptimalParameterization
start_state_max_bounds_error: 0.1
num_steps: 32
planner_action_topic: "/cumotion/motion_plan"
```

Load in your MoveIt config:
```python
.planning_pipelines(
    pipelines=["isaac_ros_cumotion", "ompl"],
    default_planning_pipeline="isaac_ros_cumotion",
)
```

Client usage:
```python
req.pipeline_id = "isaac_ros_cumotion"
# goes via /move_action → plugin → cumotion_planner_node
```

## Option B: Direct cumotion action client

Bypass the MoveIt plugin entirely — send your request directly to `/cumotion/move_group` (the action topic the cumotion node exposes natively).

Benefits:
- No plugin timeout to work around
- Full control over action goal / result handling
- Useful for benchmarking and dataset validation runs where you want raw cumotion behavior without MoveIt's overhead

Drawbacks:
- **No request adapters run**. You don't get `AddTimeOptimalParameterization`, so trajectory timing is cuMotion's internal (can drift, may hit DT_EXCEPTION).
- No FCL post-validation — you must run it yourself (see `tools/fcl_validate_trajectories.py`).
- You lose MoveIt's pipeline conveniences (workspace bounds fix, etc.)

### When to use

- Dataset benchmarking (we used it for v2 validation runs)
- Debugging cuMotion's raw behavior
- Custom controllers that don't need MoveIt's time parameterization

### Client usage

```python
from moveit_msgs.action import MoveGroup
client = ActionClient(node, MoveGroup, "/cumotion/move_group")
# Build MotionPlanRequest as usual, but no plugin will translate pipeline_id.
```

The goal message format is the same (MoveGroup action), just routed differently.

## Which pipeline should I use?

| Use case | Pipeline | Action topic |
|---|---|---|
| Production robot with controllers expecting TOTG timing | Option A (plugin via `/move_action`) | `/move_action` |
| Offline dataset benchmarking | Option B (direct) | `/cumotion/move_group` |
| Interactive development / RViz motion planning panel | Option A | `/move_action` |
| Custom C++ node that does its own time parameterization | Either | either |

**Default: Option A with the warmup patch applied.**

## MoveIt2 Humble FCL SIGSEGV

Regardless of which option you pick, MoveIt2 Humble has a known thread-safety bug in FCL. Under load (many parallel collision checks), `fcl::BroadPhaseCollisionManager::registerObjects` races and crashes `move_group`.

Fix: switch to the Bullet collision detector.

```yaml
# In your MoveIt config (moveit_controllers.yaml or equivalent)
collision_detector: "Bullet"
```

Or in code when building MoveItConfig:
```python
.move_group_capabilities(capabilities=["..."])  # standard MoveIt config
```
(Bullet is opt-in at runtime; some configs use `planning_scene/collision_detector` ROS param.)

If you can't switch detectors, add crash detection to your dispatcher (see [06_ompl_fallback.md](06_ompl_fallback.md)) so you catch the crash and fall back to OMPL on the retry.

## Configuring both pipelines in parallel

Your MoveIt launch should load both `isaac_ros_cumotion` and `ompl`:

```python
moveit_config = (
    MoveItConfigsBuilder(...)
    .planning_pipelines(
        pipelines=["isaac_ros_cumotion", "ompl"],
        default_planning_pipeline="isaac_ros_cumotion",
    )
    .to_moveit_configs()
)
```

This gives clients the freedom to switch `pipeline_id` per request — critical for the OMPL fallback pattern in [06_ompl_fallback.md](06_ompl_fallback.md).

See [`templates/launch/cumotion_with_ompl_fallback.launch.py`](../templates/launch/cumotion_with_ompl_fallback.launch.py) for a reference launch.
