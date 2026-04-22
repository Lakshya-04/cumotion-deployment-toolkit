# 07 — Validation

How to benchmark your cuMotion setup and catch issues before they hit production.

## Why you need a validation workflow

cuMotion's sphere model is approximate. Until you measure on realistic goal pairs, you don't know:

1. What success rate is realistically achievable on your robot
2. Which specific sphere pairs are costing you the most
3. Whether your "successful" plans are actually collision-free at mesh level
4. Whether the dataset you're validating against is even reachable given your tool geometry

All solvable with three tools from this repo.

## Workflow overview

```
1. Build a pair dataset (start/goal joint configs)
2. Run validate_dataset.py         → per-pair results
3. Run fcl_validate_trajectories.py → mesh-level ground truth
4. Run sphere_blocker_analysis.py  → which spheres block which pairs
5. Fix (shrink, add ignore, regenerate)
6. Loop to step 2
```

## 1. Build a pair dataset

Generate random reachable joint-space pairs, filter via kinematic validation (pinocchio, pybullet, etc.) so only collision-free configurations remain. Dataset format:

```json
{
  "pairs": [
    {"id": "pair_000000", "start": [j1, j2, j3, j4, j5, j6], "goal": [...]},
    {"id": "pair_000001", "start": [...], "goal": [...]},
    ...
  ]
}
```

A 1k-10k pair dataset sampled uniformly over joint limits is a good start.

**Gotcha**: if your generator uses only the arm URDF (no tool), pairs will include tool-infeasible configurations. A significant fraction may fail FCL start/goal validation once the tool is added. This is normal — filter before benchmarking a "real" rate.

## 2. Run the dataset

```bash
python tools/validate_dataset.py \
    --dataset pairs.json \
    --out-dir results/ \
    --planning-group manipulator \
    --arm-joints shoulder_pan_joint shoulder_lift_joint elbow_joint \
                 wrist_1_joint wrist_2_joint wrist_3_joint \
    --pipeline cumotion \
    --save-trajectories
```

Writes `results/run_cumotion_<timestamp>.json` with per-pair:
- `success`: whether the planner returned a non-empty trajectory
- `n_pts`: number of waypoints
- `time_s`: planning time
- `trajectory`: waypoints (if `--save-trajectories`)

Repeat with `--pipeline ompl` and `--pipeline fallback` for comparison.

## 3. FCL-validate the successes

Success in cuMotion doesn't mean mesh-clean. Run:

```bash
python tools/fcl_validate_trajectories.py \
    --run results/run_cumotion_<ts>.json \
    --planning-group manipulator
```

Output: per-trajectory CLEAN/DIRTY + aggregated collision pair counts.

**What to look for:**

- High % dirty → sphere model is missing collisions. Check XRDF ignore list: any `disable_collisions` entries that are wrong?
- Specific pair repeated across many dirty trajectories → that link pair needs a sphere that covers the actual mesh region (you're under-approximating).

## 4. Identify blocker spheres

For the pairs that cuMotion couldn't plan (TRAJOPT_FAIL), find out which spheres are in the way:

```bash
python tools/sphere_blocker_analysis.py \
    --dataset pairs.json \
    --xrdf robot.xrdf \
    --urdf robot.urdf \
    --joint-names shoulder_pan_joint shoulder_lift_joint elbow_joint \
                  wrist_1_joint wrist_2_joint wrist_3_joint \
    --max-pairs 200 \
    --top 20
```

Output: ranked list of (link_a sphere_i, link_b sphere_j, blocker count, radii).

**Reading the output:**

- A sphere with very large radius (>120mm) at the top → candidate for shrinking or splitting
- A pair of small spheres blocking many pairs → likely sphere over-approximation on thin geometry, not a real collision; consider adding to ignore list after FCL verifies

## 5. Fix and re-measure

Based on the blocker analysis, apply one of:

- **Shrink spheres**: edit the XRDF or use `tools/tighten_xrdf_from_urdf.py` to Voronoi-tighten against meshes
- **Split spheres**: replace one large sphere with two smaller ones covering the same region
- **Add ignore**: only after verifying with FCL that the pair is geometrically non-colliding
- **Regenerate**: `tools/tighten_xrdf_from_urdf.py --regenerate --method foam` for a fresh fit

Re-run validate_dataset.py. Track the numbers in a simple CSV/RUNS.md to see progress.

## What numbers are "good"?

Context-dependent, but rough guidelines:

- Random reachable joint pairs on a 6-DOF arm (no tool, no obstacles): **70-90%** cuMotion success
- 6-DOF arm with a big end-effector tool: **30-60%** (tool geometry adds corridors)
- 7-DOF (arm + lift/rail): subtract 10-20% from above
- With OMPL fallback: add 15-25% to whatever cuMotion alone gets

If you're far below these ranges, something's wrong with the XRDF or the dataset.

## Replay for visual inspection

After validation, spot-check results:

```bash
python tools/replay_trajectories.py --run results/run_cumotion_<ts>.json \
    --pair-ids pair_000042 pair_000119 \
    --loops 3
```

Animates in `/joint_states` — RViz (with MoveIt or robot_state_publisher) will show the motion. Useful for catching "success but the path is awful" cases that pure success rate misses.

## Build a "known-good" subset

Keep a small (50-100 pair) subset of pairs that ALL planners (cuMotion AND OMPL) solve cleanly. Use this as a regression test — run before any XRDF change to catch unintended breakage.
