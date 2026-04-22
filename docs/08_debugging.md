# 08 — Debugging Guide

Every failure mode you'll hit deploying cuMotion, what it means, and how to fix it.

## Quick triage

Always start with the error code from `motion_gen_result.status` (cuRobo) or `MoveItErrorCodes` (MoveIt):

| Symptom | Likely cause | Go to |
|---|---|---|
| `TRAJOPT_FAIL` | Sphere corridor too tight, optimizer can't smooth | [TRAJOPT_FAIL](#trajopt_fail) |
| `DT_EXCEPTION` | Trajectory timing constraint violated | [DT_EXCEPTION](#dt_exception) |
| `START_STATE_IN_COLLISION` (-10) | cuRobo thinks start is in self-collision | [START_STATE_IN_COLLISION](#start_state_in_collision) |
| `INVALID_MOTION_PLAN` (-2) | FCL post-check rejected cuRobo trajectory | [INVALID_MOTION_PLAN](#invalid_motion_plan) |
| `COLLISION_CHECKING_UNAVAILABLE` | ESDF not ready | [ESDF timing](#esdf-not-available) |
| `TIMED_OUT` (-6) | JS warmup or GPU contention | [Warmup timeout](#timed_out--warmup) |
| `PLANNING_FAILED` / code=0 | Generic — usually TRAJOPT_FAIL | See TRAJOPT_FAIL |
| move_group crashed mid-plan | FCL thread race | [move_group SIGSEGV](#move_group-sigsegv) |
| `UnicodeDecodeError` reading XRDF | Non-ASCII character | [XRDF ASCII](#xrdf-ascii) |
| `ParameterAlreadyDeclaredException` | Duplicated param (common after restarts) | [Duplicated patches](#duplicated-patches) |
| `Planner is busy` | `planner_busy` flag not cleared | [planner_busy leak](#planner_busy-leak) |

---

## TRAJOPT_FAIL

cuRobo's trajectory optimizer couldn't produce a smooth collision-free trajectory. Graph search often succeeds (discrete waypoints exist) but the continuous smoothed path can't fit through the sphere-model corridor.

**Diagnose:**
```bash
python tools/sphere_blocker_analysis.py --dataset pairs.json --xrdf full_robot.xrdf
```
This linearly interpolates every pair and reports which specific sphere pairs block the most plans.

**Fixes (in order):**

1. **Shrink the blocker spheres** — often a single oversized sphere (>120mm) on a mount or cable clip blocks many pairs. Reduce to the actual mesh bounding sphere.
2. **Increase timesteps**: `num_trajopt_time_steps: 48` (or 64) — finer resolution helps navigate tight corridors.
3. **More graph seeds**: `num_graph_seeds: 64`, `num_trajopt_seeds: 32` — more diverse starting paths.
4. **More TrajOpt finetune iterations**: `trajopt_finetune_iters: 400` — more optimizer effort.
5. **Fall back to OMPL** (see [06_ompl_fallback.md](06_ompl_fallback.md)).

**What does NOT help:**
- Longer timeout alone. On tight-corridor pairs, a 60s budget won't find a path a 10s budget missed — the problem is geometric, not time.

---

## DT_EXCEPTION

cuRobo's optimal trajectory timestep exceeded `maximum_trajectory_dt`. The log message: `"Optimized dt is above maximum_trajectory_dt, consider increasing max_trajectory_dt"`.

**Root cause:** By default cuRobo caps `maximum_trajectory_dt` at 0.15s. With 32 timesteps, max total trajectory time = 4.8s. Large-motion pairs (several joints each moving ~3 rad at 3 rad/s max velocity) need ≥4.8s and hit this limit.

**Fix:** Expose `maximum_trajectory_dt` as a ROS parameter (not exposed by default in `isaac_ros_cumotion` node) and set to 0.5 or higher:

```python
self.declare_parameter('maximum_trajectory_dt', 0.5)
# ...
motion_gen_config = MotionGenConfig.load_from_robot_config(
    ...,
    maximum_trajectory_dt=self.__maximum_trajectory_dt,
)
```

See `templates/scripts/cumotion_planner_patches.py` for the full patch. With 0.5 × 48 steps = 24s budget, large motions succeed.

### Alternative: use MoveIt's TOTG adapter

Instead of bumping `maximum_trajectory_dt`, add `AddTimeOptimalParameterization` to the cumotion planning pipeline's `request_adapters`. MoveIt re-parameterizes cuMotion's output using joint velocity/acceleration limits, sidestepping the internal timing logic entirely.

```yaml
# isaac_ros_cumotion_planning.yaml
request_adapters: >-
  default_planner_request_adapters/FixWorkspaceBounds
  default_planner_request_adapters/FixStartStateBounds
  default_planner_request_adapters/FixStartStatePathConstraints
  default_planner_request_adapters/AddTimeOptimalParameterization
```

Using both (patched `maximum_trajectory_dt` AND TOTG) is also fine and gives the most predictable results. See [`templates/config/isaac_ros_cumotion_planning.yaml`](../templates/config/isaac_ros_cumotion_planning.yaml).

---

## START_STATE_IN_COLLISION

cuRobo's sphere model says the start configuration is in self-collision. Several sub-cases:

### Sub-case A: sphere over-approximation

Run blocker diagnosis to see which sphere pair overlaps. If the overlap is only a few mm and FCL/mesh says the state is fine:
- The sphere model is over-conservative. Either shrink the offending sphere or add the pair to the ignore list if the pair is physically non-colliding.

### Sub-case B: wrong sphere transform

If spheres are floating in empty space or poking through other links in RViz, the transform from mesh frame to link frame is wrong. See [04_sphere_fitting.md](04_sphere_fitting.md#critical-gotcha-sphere-frame-of-reference).

### Sub-case C: genuinely invalid start

Dataset error — the start config truly collides. Filter the dataset through FCL validation before benchmarking.

---

## INVALID_MOTION_PLAN

cuRobo returned a trajectory, but MoveIt's FCL post-check rejected it. Two interpretations:

- **Good:** Your collision model is MORE conservative than mesh reality — this is a false positive you can bypass with `bypass_fcl_check=True` if you trust the sphere model.
- **Bad:** Your collision model is LESS conservative than mesh — cuRobo is planning through a region that actually has mesh collision (e.g. XRDF ignore list has a pair that shouldn't be ignored).

**Diagnose:** FCL's contact info tells you which link pair. Search the XRDF ignore list:
- If the pair is there → remove it (ignore list was wrong)
- If not → the sphere for that link is too small to cover the mesh

---

## ESDF not available

`COLLISION_CHECKING_UNAVAILABLE` — cuRobo is waiting for ESDF updates from nvblox (or other world collision source).

**Causes:**
- Depth sensors haven't published enough frames yet (early in mission)
- nvblox node crashed or not running
- The ESDF topic name doesn't match cuRobo's subscription

**Fix:** Fall back to OMPL (see fallback pattern). Long-term, ensure your system waits for ESDF readiness before enabling cuMotion.

---

## TIMED_OUT — warmup

First cuMotion plan after node startup hangs for 20-30s, times out at the action client's 5s budget. Subsequent plans are fast.

**Root cause:** cuRobo lazily compiles JS (joint-space) TrajOpt CUDA graphs on first `plan_single_js` call. `motion_gen.warmup()` only compiles pose-based graphs, not JS.

**Fix in `cumotion_planner.py`**:

```python
def warmup(self):
    self.motion_gen.warmup(enable_graph=True)
    # Pre-compile JS TrajOpt CUDA graphs
    try:
        jnames = self.motion_gen.kinematics.joint_names
        dummy_q = torch.zeros(1, len(jnames), dtype=torch.float32, device='cuda:0')
        from curobo.types.robot import JointState
        s = self.motion_gen.get_active_js(JointState.from_position(position=dummy_q, joint_names=jnames))
        g = self.motion_gen.get_active_js(JointState.from_position(position=dummy_q+0.01, joint_names=jnames))
        self.motion_gen.reset(reset_seed=False)
        self.motion_gen.plan_single_js(s, g, MotionGenPlanConfig(
            max_attempts=self.__max_attempts, enable_graph_attempt=1,
            timeout=60.0, check_start_validity=False,
        ))
    except Exception as exc:
        self.get_logger().warn(f'JS warmup failed: {exc}')
```

See `templates/scripts/cumotion_planner_patches.py`.

---

## move_group SIGSEGV

MoveIt2 Humble has a known FCL bug: concurrent `registerObjects` calls from MoveIt's multi-threaded executor cause a segfault. cuMotion triggers it by pushing many collision objects rapidly.

**Symptoms:**
- `move_group` node dies mid-planning
- Core dump shows `fcl::BroadPhaseCollisionManager`
- Logs show `registerObjects` in backtrace

**Fix:** Switch to Bullet collision detector in MoveIt config:
```yaml
# moveit_controllers.yaml or equivalent
collision_detector: "Bullet"
```

Bullet is thread-safe where FCL isn't in this version.

Also add crash detection to your planner dispatcher (see [06_ompl_fallback.md](06_ompl_fallback.md)).

---

## XRDF ASCII

`UnicodeDecodeError: 'ascii' codec can't decode byte 0xe2` while loading XRDF.

**Cause:** cuRobo reads XRDF as ASCII. Any UTF-8 character (em-dash `—`, ≥, ±, right-arrow `→`) crashes the load.

**Fix:** Replace non-ASCII with ASCII equivalents (`-`, `>=`, `+/-`, `->`). Verify:
```bash
python -c "open('full_robot.xrdf', 'rb').read().decode('ascii')"
```

---

## Duplicated patches

After patching `cumotion_planner.py` in a container/venv that's restored from an image, the patch may be applied multiple times. Result:

```
ParameterAlreadyDeclaredException: Parameter 'maximum_trajectory_dt' has already been declared
```

**Fix:** Use a dedup script to keep exactly one declaration. See `templates/scripts/cumotion_planner_patches.py` which is idempotent (checks before inserting).

---

## planner_busy leak

`Planner is busy` error on every plan after the first one fails. The `planner_busy` flag in the cumotion action server never got cleared because an exception in `plan_single_js` bypassed the clear path.

**Fix:** Wrap `plan_single_js` in try-finally:

```python
try:
    motion_gen_result = self.motion_gen.plan_single_js(start_state, goal_state, config)
except Exception as exc:
    self.get_logger().error(f'plan_single_js raised: {exc}')
    result = MoveGroup.Result()
    result.error_code.val = MoveItErrorCodes.FAILURE
    goal_handle.succeed()
    return result
finally:
    with self.lock:
        self.planner_busy = False
```

Patch included in `templates/scripts/cumotion_planner_patches.py`.

---

## GPU out of memory

`CUDA out of memory` during warmup, especially on smaller GPUs (RTX 3050, 4 GiB).

**Causes:**
1. Stale processes holding memory (common after crashed launches)
2. RViz + cuMotion + nvblox all on the same GPU
3. Too many trajopt seeds allocated

**Fixes:**
1. `kill -9` any orphan `python3 /opt/ros/.../cumotion_planner_node` processes. Check `nvidia-smi --query-compute-apps=pid --format=csv,noheader`.
2. Run RViz on a separate display or reduce its memory load.
3. Lower `num_trajopt_seeds` and `num_graph_seeds`.
4. `export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True` to reduce fragmentation.

---

## Still stuck?

Run `tools/validate_dataset.py` on your pairs dataset with `batch_mode:=true`. It saves per-pair results with blocker diagnosis, making it easy to see patterns across failures rather than debugging one at a time.
