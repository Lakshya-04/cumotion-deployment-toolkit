# 10 — Lessons Learned

The non-obvious takeaways from deploying cuMotion on real hardware. What the documentation doesn't tell you.

## Planner selection is binary + fallback

You cannot pick one planner for production. cuMotion is fast but fragile; OMPL is slow but robust. The combination beats either alone by 20-30 percentage points on success rate.

**Use cuMotion as primary, OMPL as fallback.** Don't try to "choose" — run both.

## Sphere quality dominates every other tuning decision

Changing graph/trajopt seeds moves success rates by 2-5%. Changing sphere model quality moves them by 20-30%. If your numbers are below expectations, fix spheres first.

Specifically:
- A single oversized sphere (>150mm on a blocker pair) can reduce success rate by 20%
- A 30mm over-approximation near a column creates a "forbidden zone" where optimizer trajectories can't fit

## The #1 debug technique: sphere blocker analysis

When TRAJOPT_FAIL happens, don't increase the timeout. Instead run `sphere_blocker_analysis.py` to see WHICH specific spheres are blocking the most pairs. One oversized sphere can be responsible for half your failures.

## Frame transforms for spheres are the single most common bug

When porting spheres generated from a mesh into an XRDF, the transform chain is:
```
mesh-local → link that owns the mesh collision (apply visual origin)
           → target link (apply inverse of joint transform, if different)
```

Miss either step and your spheres appear shifted by several centimeters, often causing false start-state collisions on every configuration. Always verify in RViz before trusting.

## Validation must happen at three levels

1. **cuMotion return code** — does it say SUCCESS? (necessary)
2. **Trajectory non-empty** — does it have waypoints? (MoveIt can return SUCCESS with 0 points if busy)
3. **FCL post-validation** — are the waypoints actually mesh-collision-free? (the ground truth)

Level 1 alone lies. Levels 1+2+3 is the real picture.

## `maximum_trajectory_dt` is undocumented but critical

cuRobo's default (0.15s) caps total trajectory time at `num_trajopt_time_steps × 0.15`. For 32 steps that's 4.8s — too short for many large arm motions. Result: `DT_EXCEPTION` on otherwise valid plans.

Fix by exposing it as a ROS parameter and setting to 0.5. Trivial patch, huge impact.

## MoveIt's TOTG is non-negotiable for production

cuMotion's internal time parameterization drifts near velocity/acceleration limits. Controllers expect smooth, strictly-monotonic, limit-respecting trajectories. MoveIt's `AddTimeOptimalParameterization` adapter re-parameterizes cuMotion's output to match. Don't skip it.

## MoveIt 2 Humble has an FCL SIGSEGV you'll hit

Concurrent planning + MoveIt's multi-threaded executor → `fcl::registerObjects` race → `move_group` crashes. Switch to Bullet collision detector. If you can't, add crash detection in your dispatcher and permanently switch to OMPL for the rest of that planning request.

## The dataset probably doesn't match your robot

Random pair datasets are usually generated with the arm-only URDF (via pinocchio etc.). Adding a tool, AMR base, or environment reveals that 30-50% of pairs are mesh-infeasible at start or goal. Your "real" achievable rate is always lower than the dataset's nominal size suggests.

**Build a filtered subset** that's FCL-valid at start and goal before benchmarking seriously.

## GPU memory fragments; restart daily

After 6-12 hours of continuous operation, cuMotion plan times drift upward and OOM errors start appearing. Cause: CUDA memory fragmentation from the churn of collision object cache.

Workaround: periodic restart of `cumotion_planner_node` (30s warmup cost, cheap). Long-term: NVIDIA may fix; in the meantime, build it into your uptime budget.

## Stale processes are a constant threat

Failed launches leave orphan processes consuming GPU memory. Symptoms: plans hang, OOM in warmup, GPU listed as "in use" with no apparent owner.

```bash
# Check
nvidia-smi --query-compute-apps=pid,used_memory --format=csv,noheader

# Kill
kill -9 $(nvidia-smi --query-compute-apps=pid --format=csv,noheader)
```

Better: restart the container / host periodically. ROS 2 cleanup is not reliable under crashes.

## XRDF ignore list needs iteration, not one-shot

Starting from SRDF `disable_collisions reason="Never|Adjacent"` gets you 80% of the way. The other 20% reveals itself through:
- FCL post-validation flagging trajectories as dirty → you missed adding a real collision pair back
- `sphere_blocker_analysis.py` showing a pair blocking many pairs when it shouldn't → add to ignore if verified non-colliding

Budget a day of iteration to get the ignore list right on a new robot.

## ASCII-only discipline

cuRobo reads XRDF as ASCII. One em-dash in a comment crashes the loader with `UnicodeDecodeError`. Common in AI-generated comments.

Quick check: `python -c "open('robot.xrdf', 'rb').read().decode('ascii')"`. Add it to CI if you have one.

## Plan_single_js vs plan_single for joint goals

Out-of-the-box `cumotion_planner_node` uses `plan_single` (pose-based) for everything. If your goal is joint-space, this fails with unhelpful errors — because cuMotion is trying to solve IK for the pose of your goal-configured end-effector and optimize back.

Use `plan_single_js` for joint-space goals. Requires patching the planner (see `templates/scripts/cumotion_planner_patches.py`).

## cuMotion doesn't support URDF mimic joints

Common in telescoping lifts, grippers, parallel-jaw actuators. cuRobo treats them as fixed at joint=0. For telescoping structures where the mimic-joint link overlaps other geometry along its full travel, the effect is limited. For independent mimic chains (e.g. a parallel gripper), you need one of:

1. Accept the wrong pose and model collision geometry conservatively
2. Add as actuated joint with a linear constraint (cuRobo supports in cspace)
3. URDF surgery to collapse the mimic chain

## Small GPUs limit seed budget

RTX 3050 (4 GB) maxes out around 48 graph + 32 trajopt seeds. Jetson Orin comfortably handles 128 + 64. The seeds are where cuMotion gets its success rate — less GPU = lower ceiling.

Plan for Orin in production from day 1.

## Warmup isn't one-shot

`motion_gen.warmup(enable_graph=True)` compiles Pose-based CUDA graphs. It does NOT compile JS (joint-space) graphs. First `plan_single_js` call triggers a separate ~25s compilation.

With the MoveIt plugin's 5s action-client timeout, your first joint-space plan always fails with TIMED_OUT unless you explicitly pre-compile JS graphs in your warmup. The patch is in `templates/scripts/cumotion_planner_patches.py`.

## Test on real pairs, not synthetic ones

Success rate on random joint pairs ≠ success rate on your application's actual goal distribution. If your robot always goes from A to B and back, benchmark on {A, B} — not on 10k random pairs.

## Keep a known-good dataset

A small set (50-100 pairs) that BOTH cuMotion AND OMPL solve cleanly. Use as a regression test before any XRDF or config change. Catches 90% of deployment regressions with minimal runtime cost.
