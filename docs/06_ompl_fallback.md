# 06 — OMPL Fallback Strategy

The single most important pattern for production cuMotion deployments. Neither planner alone is reliable enough — you need both.

## Why fallback is required

cuMotion fails in specific ways that OMPL doesn't:

| Failure mode | cuMotion outcome | OMPL outcome |
|---|---|---|
| Start state in cuRobo sphere collision (mesh-valid) | `START_STATE_IN_COLLISION` | Plans normally |
| ESDF not yet populated | `COLLISION_CHECKING_UNAVAILABLE` | Plans against SRDF only |
| Narrow mesh corridor | `TRAJOPT_FAIL` (sphere over-approx) | Samples a valid path |
| `move_group` SIGSEGV (FCL thread race) | Process crashes | Unaffected |
| Transient GPU memory fragmentation | `PLANNING_FAILED` | Unaffected |

And OMPL fails where cuMotion wins:

| Problem | OMPL | cuMotion |
|---|---|---|
| Fast replans (<1s) | Rarely | Usually |
| Smooth, jerk-optimal trajectories | Needs shortcut/smoother | Native |
| Real-time deterministic performance | No | Yes |

The production sweet spot: **cuMotion as primary, OMPL as fallback on specific triggers**.

## Dispatch pattern

```
for attempt in range(max_attempts):
    active_pipeline = "ompl" if ompl_fallback else primary_pipeline

    if active_pipeline == "isaac_ros_cumotion":
        success = plan_with_cumotion(goal)
        if not success:
            err = last_cumotion_error_code()

            # Immediate fallback triggers: problems OMPL can fix
            if err in (COLLISION_CHECKING_UNAVAILABLE, START_STATE_IN_COLLISION):
                ompl_fallback = True
                success = plan_with_ompl(goal, time_regular)

            # Permanent fallback: crash protection
            elif move_group_crashed():
                # Retrying cuMotion would crash move_group again (FCL race).
                # Switch permanently to OMPL for the rest of this request.
                ompl_fallback = True
                success = plan_with_ompl(goal, time_increased)
    else:
        # In fallback mode — OMPL handles the rest
        planning_time = time_regular if attempt == 0 else time_increased
        success = plan_with_ompl(goal, planning_time)

    if success and post_validate_trajectory(trajectory):
        return SUCCESS

# Last resort: cuMotion exhausted all attempts without fallback trigger
# (e.g. just repeated TRAJOPT_FAIL/PLANNING_FAILED).
if primary_pipeline == "isaac_ros_cumotion" and not ompl_fallback:
    if plan_with_ompl(goal, time_increased) and post_validate_trajectory(trajectory):
        return SUCCESS

return FAILURE
```

## Key parameters

| Param | Typical | Purpose |
|---|---|---|
| `planning_pipeline` | `"isaac_ros_cumotion"` | Primary |
| `planning_attempts` | 3-5 | cuMotion retries (GPU transient failures) |
| `planning_time_regular` | 2.0s | OMPL first attempt time budget |
| `planning_time_increased` | 5.0s | OMPL fallback/retry time budget |

## Trigger conditions in detail

### `COLLISION_CHECKING_UNAVAILABLE` — immediate fallback

cuMotion is waiting for ESDF (nvblox) to populate. Early in a mission, depth sensors haven't produced enough data yet. OMPL doesn't need the ESDF — it plans against the SRDF-defined collision geometry, which is present from start.

### `START_STATE_IN_COLLISION` — immediate fallback

cuRobo's sphere model flagged the start as self-colliding. Often a false positive from sphere over-approximation (spheres block at ~30-40mm beyond true mesh boundary). FCL (mesh) inside OMPL may say the state is fine.

If FCL also says it's colliding, OMPL will fail too and you've lost nothing.

### `move_group` SIGSEGV — permanent switch

MoveIt2 Humble has a known FCL thread-safety bug: concurrent `registerObjects` calls from MoveIt's multi-threaded executor cause SIGSEGV. cuMotion triggers this because it pushes many collision objects rapidly.

Mitigations:
1. **Switch to Bullet collision detector** in MoveIt (`collision_detector: "Bullet"` in the SRDF/config) — Bullet is thread-safe where FCL isn't.
2. **Detect the crash** at the dispatch layer — watch for the move_group node crashing and respawning. Once detected, retrying cuMotion will crash it again. Switch to OMPL permanently for this planning request.

Crash detection pattern:
```python
def move_group_crashed(self):
    # Snapshot pid at start of cumotion call, compare after.
    # If pid changed, move_group respawned (crash).
    return self._cumotion_start_pid != get_move_group_pid()
```

### Last-resort OMPL after cuMotion exhausts

Covers generic failures that don't fit a specific trigger:
- `TRAJOPT_FAIL` repeatedly (sphere corridor too tight for smooth trajectory)
- `PLANNING_FAILED` (graph search couldn't find seed)
- `TIMED_OUT`

These are often solvable by OMPL's sampling (it just needs more time).

## Time parameterization: use MoveIt's TOTG, not cuMotion's internal

Both cuMotion and OMPL produce geometrically valid trajectories, but their timing differs:

- **cuMotion** — internal minimum-jerk time parameterization. Fast but can drift near velocity/acceleration limits, especially with custom `maximum_trajectory_dt`. Can produce `DT_EXCEPTION` on large motions.
- **OMPL** — no time parameterization at all. You MUST re-parameterize before execution.

**Use MoveIt's `AddTimeOptimalParameterization` (TOTG) adapter for both.** Add it to both pipeline configs:

```yaml
# isaac_ros_cumotion_planning.yaml
request_adapters: >-
  default_planner_request_adapters/FixWorkspaceBounds
  default_planner_request_adapters/FixStartStateBounds
  default_planner_request_adapters/FixStartStatePathConstraints
  default_planner_request_adapters/AddTimeOptimalParameterization
```

Benefits:
- Joint velocity/acceleration limits strictly respected (from URDF)
- Monotonic, controller-friendly timing
- Consistent trajectory timing between cuMotion and OMPL paths — downstream execution code doesn't need to distinguish
- Fixes timing drift that triggers `DT_EXCEPTION` without needing to re-tune `maximum_trajectory_dt`

See `templates/config/isaac_ros_cumotion_planning.yaml` for the full config.

## Post-validation is mandatory

Every trajectory — from cuMotion OR OMPL — must pass FCL mesh-level post-validation before execution. cuMotion's sphere check can miss real mesh collisions; OMPL's internal check can miss edge cases from smoothing/scaling.

```python
def post_validate_trajectory(traj):
    psm = planning_scene_monitor.snapshot()  # freeze scene once
    for waypoint in traj.points:
        state = apply_waypoint_to_robot_state(waypoint)
        if psm.is_state_colliding(state):
            return False
    return True
```

**Important:** snapshot the planning scene ONCE before iterating waypoints. If you read it live, a mid-validation update can cause a partial scene check and false positives.

## Implementation

See `templates/launch/cumotion_with_ompl_fallback.launch.py` and the dispatcher class in `templates/scripts/planner_dispatcher.py`.

## Measured impact

On one deployment (random goal pairs dataset):
- cuMotion only: ~15% plan success
- OMPL only (same pairs, RRTConnect): ~10% clean
- cuMotion + OMPL fallback: ~40% combined

Your numbers will vary by robot geometry, but the pattern is consistent: the combined rate substantially exceeds either planner alone because their failure modes are orthogonal.
