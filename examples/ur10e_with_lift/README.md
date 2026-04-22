# UR10e + Vertical Lift (7-DOF Example)

A UR10e mounted on a vertical prismatic lift joint. This demonstrates two things most 6-DOF examples don't cover:

1. **Adding an extra DOF** to a standard arm (lift, rail, gantry, etc.)
2. **Workarounds for URDF mimic joints** — cuRobo doesn't support them, and many real robots use them for telescoping columns, grippers, etc.

## Configuration

- Joint 1: `lift` — vertical prismatic, 0 → 1.4 m
- Joints 2-7: standard UR10e arm (shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3)
- Arm is mounted on top of the lift via a fixed joint

Total: 7-DOF joint-space planning problem.

## Files

```
ur10e_with_lift/
├── README.md                        (this file)
├── urdf/
│   └── ur10e_with_lift.urdf.xacro  (minimal xacro: UR10e on a prismatic lift)
├── xrdf/
│   └── ur10e_with_lift.xrdf        (7-DOF XRDF — lift as first joint)
├── config/
│   ├── cumotion_params.yaml
│   └── visualize.rviz              (RViz layout with URDF + XRDF spheres)
├── launch/
│   ├── visualize.launch.py         (visualize only — no planning)
│   └── fallback.launch.py          (full cuMotion + OMPL stack)
└── scripts/
    └── simple_plan_example.py
```

## Quick start

### Visualize first

Before planning, verify the XRDF spheres align with the composite URDF (arm + lift):

```bash
ros2 launch $PWD/launch/visualize.launch.py
```

Drag the `lift` slider in the joint GUI — the column spheres should stay rigid on the column as it rises, and the arm (with its spheres) should move with it.

## Extra DOF — key differences vs plain UR10e

### cspace

The lift joint is added as the first entry. cuRobo's TrajOpt scales difficulty ~exponentially with DOF, so plan times double vs 6-DOF.

### Joint vel/acc limits

Prismatic lifts typically move much slower than arm joints (m/s limits, not rad/s). Set realistic values in the XRDF `cspace`:

```yaml
cspace:
  joint_names: [lift, shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
  # lift = 0.5 m/s² typical; arm joints 12 rad/s²
  acceleration_limits: [0.5, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0]
  jerk_limits:         [5.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0]
```

### Larger `maximum_trajectory_dt`

A 1.4 m lift traverse at 0.5 m/s² takes ~2.4 s minimum. Combined with arm motion, total trajectory can exceed 4.8 s (default cap). Use 0.5 or higher for `maximum_trajectory_dt`.

## Handling a telescoping column (mimic joints)

Many real robot lifts have a telescoping structure where the column "middle" follows the "top" at half speed — URDF expresses this as a mimic joint:

```xml
<joint name="lift_middle" type="prismatic">
  <mimic joint="lift_top" multiplier="0.5"/>
</joint>
```

**cuRobo does NOT support mimic joints.** It treats `lift_middle` as fixed at joint=0. Workarounds:

### Option A: accept the positional error

If the mimic joint moves collision geometry that's always present along its full travel (e.g. a middle tube that's part of a telescoping column overlapping with other tubes), the geometric effect is limited. cuRobo's sphere model may over-approximate but rarely gives false negatives.

This is what this example uses — we don't model the mimic.

### Option B: add the mimic as an actuated joint with a linear constraint

cuRobo supports joint-space linear constraints. Add `lift_middle` to cspace, then:

```yaml
cspace:
  joint_names: [lift_top, lift_middle, shoulder_pan, ...]
  # Constraint: lift_middle = 0.5 * lift_top
  # linear_constraints: [...]   # cuRobo API specifics
```

### Option C: URDF surgery

Collapse the mimic chain into a single prismatic joint with fixed child links. Cleanest long-term but invasive.

See [`docs/04_sphere_fitting.md`](../../docs/04_sphere_fitting.md#mimic-joint-limitation) for more.

## Performance notes

With 7-DOF, expect ~1.5-2x plan time vs 6-DOF. On RTX 3050:
- cuMotion: 0.5-2s per plan
- OMPL RRTConnect: 0.2-1s per plan

Dispatcher combined success rate on random pairs: ~60-80% (lower than pure 6-DOF due to the larger cspace).
