# 09 — Performance Tuning

How to make cuMotion fast (or at least not slow) on your hardware.

## Your time budget is GPU-bound

cuMotion's bottleneck is the GPU collision checking kernel, not CPU. Planning time scales with:
- Number of collision spheres (quadratic pairwise cost)
- Number of parallel seeds (TrajOpt + graph)
- Number of waypoints per trajectory

## Hardware-specific tuning

### Jetson Orin (deployment target)

- `num_graph_seeds: 64-128` — Orin handles this well
- `num_trajopt_seeds: 64` — 2x what you'd do on RTX 3050
- `num_trajopt_time_steps: 32-48` — stick with 32 unless your corridors are tight
- Expected plan time: 0.2-0.8s

### RTX 3050 (4 GB, budget desktop)

- `num_graph_seeds: 48` — more is GPU-memory-fragmentation risk
- `num_trajopt_seeds: 32`
- `num_trajopt_time_steps: 32`
- Expected plan time: 0.5-2.0s
- Watch out for CUDA OOM on multi-process setups (RViz + cuMotion + nvblox all share the GPU)

### RTX 3060+ / 4070+ / workstation GPUs

- `num_graph_seeds: 96-128`
- `num_trajopt_seeds: 64-96`
- `num_trajopt_time_steps: 48`
- Expected plan time: 0.1-0.5s

### Orin Nano (tight deployments)

- `num_graph_seeds: 32`
- `num_trajopt_seeds: 16-24`
- Keep sphere count low (<100 total)
- Expected plan time: 0.5-1.5s

## Seed budget: graph vs trajopt

**Graph seeds are cheap** (~0.05s each). More = better roadmap coverage = more likely to find a feasible seed path. Go high: 48-128.

**TrajOpt seeds are expensive** (scales linearly with the parallel planning time). More seeds = higher chance one converges. But once you have enough, diminishing returns.

Rule of thumb: `num_graph_seeds: 2 × num_trajopt_seeds`.

## Timesteps: the trajectory resolution knob

`num_trajopt_time_steps` controls how many waypoints TrajOpt optimizes over. Tradeoff:

| Timesteps | Effect |
|---|---|
| 16 | Fast but chunky; may miss feasibility in tight corridors |
| 32 (default) | Good for most 6-DOF arms |
| 48 | Better for 7-DOF or very long motions |
| 64 | Noticeably slower; diminishing returns |

Also interacts with `maximum_trajectory_dt`:
- Total max trajectory duration = `num_trajopt_time_steps × maximum_trajectory_dt`
- Default: 32 × 0.15 = 4.8s — often not enough for 7-DOF long motions
- With patched planner: 32 × 0.5 = 16s — plenty

## `trajopt_finetune_iters`

After the initial TrajOpt optimization, cuMotion runs a finetune stage for smoothness. Default 200. Impact:

- 100: fast, trajectories can be jerky
- 200 (default): fine for most
- 400+: smoother but slower; use for 7-DOF or when trajectories look "obsessive"

## `include_trajopt_retract_seed`

When `true` (cuRobo default), one TrajOpt seed is forced through the retract (home) configuration. Pros: always have a "go home first" fallback. Cons: produces long detour trajectories on large motions ("why did my arm swing through home?").

**Set to `false`** for clean direct paths. The graph seeds provide enough diversity.

## Timeout: stop chasing compute

`curobo_timeout` caps total per-plan wall-clock time.

- <3s: successes stay under this, failures give up quickly → high throughput for known-easy problems
- 10s: reasonable for mixed workloads
- 30-60s: diminishing returns. If graph+TrajOpt can't find a path in 10s on your hardware, more time usually doesn't help — the problem is geometric, not compute.

For benchmarking, use **3s timeout** — fast iteration. For production, **10s** gives marginal failures a chance.

## GPU memory hygiene

CUDA memory fragments over time, especially with many short-lived processes. Symptoms:
- Warmup fails with OOM despite having "enough" memory
- Plans slow down after hours of operation

Mitigations:

```bash
# Before launching
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True

# Kill stale processes holding GPU memory
nvidia-smi --query-compute-apps=pid --format=csv,noheader | xargs kill -9

# Avoid running RViz on the same GPU if you're memory-tight
```

Long-term, periodically restart the `cumotion_planner_node` — it's cheap (30s warmup) and releases fragmented memory.

## Measurement baseline

Always measure on YOUR robot + YOUR dataset before optimizing:

```bash
python tools/validate_dataset.py \
    --dataset pairs_100.json --out-dir baseline/ \
    --pipeline cumotion --planning-timeout 10.0 \
    --planning-group manipulator --arm-joints ...
```

Then tune one parameter at a time. Rough tuning order of impact:

1. **XRDF sphere quality** (biggest lever — see [04_sphere_fitting.md](04_sphere_fitting.md))
2. `maximum_trajectory_dt` (fixes DT_EXCEPTION = free wins)
3. `include_trajopt_retract_seed: false` (better trajectories)
4. `num_graph_seeds` up
5. `num_trajopt_time_steps` if corridors tight
6. `trajopt_finetune_iters` if trajectories look ugly

Don't tune `num_trajopt_seeds` high unless you've confirmed the others are maxed — it costs the most GPU and has the smallest gain.
