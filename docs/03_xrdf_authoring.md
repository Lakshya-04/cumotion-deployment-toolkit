# 03 — XRDF Authoring

How to write a correct XRDF for your robot. The XRDF is cuRobo's equivalent of a URDF+SRDF combined — it specifies actuated joints, collision spheres, and self-collision rules.

## Anatomy

```yaml
format: xrdf
format_version: 1.0

# ── cspace: actuated joints + limits ──────────────────────────────────
cspace:
  joint_names: [j1, j2, j3, ...]
  acceleration_limits: [...]      # rad/s² (revolute), m/s² (prismatic)
  jerk_limits: [...]
  cspace_distance_weight: [...]   # relative weight for path-length cost
  retract_config: [...]           # "home" pose

# ── default_joint_positions: fallback pose ────────────────────────────
default_joint_positions:
  j1: 0.0
  ...

# ── tool frame(s): for pose-goal planning ─────────────────────────────
tool_frames: ["tool0"]

# ── collision: link to sphere group ───────────────────────────────────
collision:
  geometry: auto_generated_collision_sphere_group

# ── self-collision ignore list ────────────────────────────────────────
self_collision:
  ignore:
    link_a: [link_b, link_c]
    ...

# ── sphere geometry ───────────────────────────────────────────────────
geometry:
  auto_generated_collision_sphere_group:
    spheres:
      link_a:
        - center: [x, y, z]
          radius: r
        ...
      link_b: []   # empty = no collision checking
```

## Generating an XRDF from scratch

1. **Start from the template**: copy `templates/config/xrdf_template.xrdf`
2. **Fill cspace** from the URDF's `<joint type="revolute|prismatic">` elements in your planning group. Match `acceleration_limits` to URDF `<limit effort="..." velocity="..."/>` roughly (cuRobo doesn't use velocity limits, only accel/jerk)
3. **Seed the ignore list** from your SRDF's `<disable_collisions reason="Never|Adjacent"/>` entries
4. **Generate spheres** — see [04_sphere_fitting.md](04_sphere_fitting.md)

## Cspace rules

### Only actuated joints

cspace must list only **actuated** joints — the ones cuRobo optimizes. Fixed joints are handled automatically via URDF. Mimic joints are **not supported**; see [04_sphere_fitting.md](04_sphere_fitting.md#mimic-joint-limitation).

### Joint order matters

The order of `joint_names` defines the order of joint values in all cuRobo API calls. Typically match your MoveIt SRDF planning group order.

### Limits are hard

cuRobo enforces `acceleration_limits` and `jerk_limits` strictly. If your URDF has tighter velocity limits, cuRobo's minimum-jerk trajectories may violate them — use MoveIt's `AddTimeOptimalParameterization` adapter (see [06_ompl_fallback.md](06_ompl_fallback.md#time-parameterization)) to re-scale using velocity limits too.

## Sphere geometry rules

### Frames

Sphere centers are in the **link's own frame**. Not mesh-local frame, not parent-link frame. Getting this wrong is the #1 bug in XRDF authoring — see [04_sphere_fitting.md](04_sphere_fitting.md#critical-gotcha-sphere-frame-of-reference).

### Empty lists

`link_name: []` means cuRobo ignores this link entirely for collision. Use for:
- Virtual frames (tool0 mount points, ft_frame, etc.)
- Links covered by a parent link's spheres
- Zero-mass helper frames

### Sphere count

Fewer spheres = faster cuRobo but more approximation. Guideline for 6-DOF arms: ~60-100 spheres total across all links. More than 200 noticeably slows planning.

## Self-collision ignore rules

### When to ignore

Add a pair to the ignore list if ANY of:

- **Adjacent**: two links connected by a joint. They will always overlap at the joint.
- **SRDF `Never`**: MoveIt's collision analysis determined the pair can't physically collide across the joint ranges.
- **SRDF `Default`**: the pair collides in the default pose but won't during planning (rare, be cautious).

### When NOT to ignore

- Two non-adjacent links that happen to be close: check with FCL first, may be a sphere over-approximation issue to fix via tighter spheres, not by ignoring.
- A pair that causes FCL post-check to reject trajectories: removing from ignore list is correct; if it was ignored by mistake, you're hiding real collisions.

### Symmetry

`ignore: {A: [B]}` is symmetric to `ignore: {B: [A]}`. Write each pair only once.

### Validating the ignore list

After initial authoring, run `tools/sphere_blocker_analysis.py`. If a link pair shows up as blocking many pairs but is genuinely non-colliding (verified visually or via FCL), it's a candidate for adding to the ignore list.

## ASCII-only

cuRobo reads the XRDF as ASCII. Non-ASCII characters crash the load with `UnicodeDecodeError`. Watch out for:
- em-dash `—` (use `-`)
- right-arrow `→` (use `->`)
- ±, ≥, ≤ (use `+/-`, `>=`, `<=`)
- Smart quotes (use straight quotes)

Verify before deploy:
```bash
python -c "open('robot.xrdf', 'rb').read().decode('ascii')"
```

## Iterating

1. Write initial XRDF
2. Launch with the XRDF
3. Visualize spheres vs mesh in RViz (`tools/replay_trajectories.py` includes a viz mode, or use any XRDF sphere publisher)
4. Run `tools/validate_dataset.py` against a representative pair set
5. Run `tools/fcl_validate_trajectories.py` to check for mesh-level violations
6. Run `tools/sphere_blocker_analysis.py` to find biggest over-approximators
7. Refine: shrink oversized spheres OR add to ignore list OR add more spheres for under-covered regions
8. Repeat until success rate converges
