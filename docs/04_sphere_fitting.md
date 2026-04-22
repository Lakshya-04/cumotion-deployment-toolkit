# 04 — Sphere Fitting

cuRobo uses spheres for collision. Getting the sphere model right is the single biggest predictor of planning success rate.

## The core trade-off

| Sphere model | Effect |
|---|---|
| Too loose (big spheres) | Safe but conservative — blocks many valid paths. Low success rate. |
| Too tight (small spheres) | Fast but risky — can miss real mesh collisions. Dangerous. |
| Correctly fit | High success, verified safe. |

A 37mm over-approximation on a column obstacle can cost 20+ percentage points of planning success.

## Sphere over-approximation: the unavoidable cost

Spheres can't perfectly represent non-spherical geometry. For a square column with 88mm half-width:
- Minimum sphere radius to cover corners: `sqrt(88² + 88²) ≈ 124mm`
- Arm link sphere radius: ~50mm (UR10e tube ~45mm)
- cuMotion blocking distance: 124 + 50 = **174mm** from column center
- True mesh collision distance: ~138mm (corner-to-arm-edge)
- **False-positive zone: ~36mm on each flat face**

Every pair where the arm needs to swing within 36mm of a column face will fail in cuMotion but succeed in mesh-based planners. This is the primary cost of the sphere model.

## Sources of sphere data

Listed by quality, best first:

1. **FOAM** ([CoMMALab/foam](https://github.com/CoMMALab/foam)) — medial-axis sphere decomposition. The gold standard: tight, varied-radius spheres that follow the geometry's skeleton. Handles concave and thin features well. **Use this unless you have a specific reason not to.**

2. **VAMP** ([CoMMALab/vamp](https://github.com/CoMMALab/vamp)) — uses FOAM internally and exposes sphere output as JSON. Convenient wrapper if you also want VAMP as a fallback planner. Same sphere quality as FOAM directly.

3. **K-means on mesh surface samples** — quick approximation. Uniform radii, less tight than FOAM. Use as a fallback if FOAM/VAMP won't install or for rapid prototyping. See `tools/generate_spheres_from_mesh.py --method kmeans`.

4. **Hand-placed** — for simple shapes (e.g. a single sphere on a cable clip) or when you need to force-cover specific features that FOAM skips.

5. **Refit an existing set** — keep FOAM/VAMP sphere centers, recompute radii by Voronoi assignment against mesh surface points. Tighter than the original without moving any sphere. See `tools/generate_spheres_from_mesh.py --method tighten`.

### Bulk XRDF refit

For tightening an entire XRDF against a URDF in one shot, use `tools/tighten_xrdf_from_urdf.py`:

```bash
# Tighten radii on every link (keeps centers)
python tools/tighten_xrdf_from_urdf.py \
    --urdf robot.urdf --xrdf-in robot.xrdf --xrdf-out robot_tight.xrdf \
    --package-root my_robot_pkg=/path/to/my_robot_pkg

# Fresh regeneration with FOAM
python tools/tighten_xrdf_from_urdf.py \
    --urdf robot.urdf --xrdf-in robot.xrdf --xrdf-out robot_new.xrdf \
    --regenerate --method foam --n 40

# Specific links only
python tools/tighten_xrdf_from_urdf.py \
    --urdf robot.urdf --xrdf-in robot.xrdf --xrdf-out robot_tight.xrdf \
    --links forearm_link wrist_1_link wrist_2_link
```

The tool resolves `package://` URIs in mesh paths via `--package-root pkg_name=/path` (repeat per package). Handles the visual/collision origin transform from mesh-local → link frame automatically.

### FOAM/VAMP in practice

```bash
# Install VAMP (brings FOAM as dep)
pip install vamp

# Generate spheres for a mesh
python -c "
import vamp
spheres = vamp.sphere_decomposition('my_mesh.stl', n_spheres=40)
# spheres is a list of {center, radius} dicts in mesh-local frame
"
```

After generation you still need to **transform spheres into the target link's frame** — see [the frame gotcha below](#critical-gotcha-sphere-frame-of-reference). Mis-applying the transform is the single most common bug.

## Critical gotcha: sphere frame of reference

**The bug we hit, and it costs hours if you miss it:**

Sphere centers in the XRDF are in the **link's own frame** — not the mesh's STL-local frame, not the parent link's frame. Miss this and your spheres appear in the wrong place.

### The chain for an end-effector tool

Given a typical URDF snippet:
```xml
<link name="tool_mount">
  <collision>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>       <!-- mesh offset in tool_mount -->
    <geometry><mesh filename="sander.stl"/></geometry>
  </collision>
</link>

<joint name="tool_mount_joint" type="fixed">
  <parent link="tool_mount"/>
  <child link="tool0"/>
  <origin xyz="0 0 0.275" rpy="0 0 3.14159"/>  <!-- tool0 placed in tool_mount -->
</joint>
```

To put spheres on `tool0` (where cuRobo tool_frame points), a mesh point P_mesh needs:

```python
# Step 1: Apply mesh's visual/collision origin → into tool_mount frame
p_tool_mount = p_mesh + visual_origin_xyz   # [0, 0, +0.2]

# Step 2: Apply INVERSE of joint transform → into tool0 frame
# Joint places tool0 at xyz=[0, 0, 0.275], rpy=[0, 0, π] inside tool_mount.
# Inverse: R^T @ (p_tool_mount - joint_xyz)
R_joint = rpy_to_mat([0, 0, 3.14159])  # 180° around Z → diag(-1, -1, 1)
p_tool0 = R_joint.T @ (p_tool_mount - [0, 0, 0.275])
```

Common mistake: applying the joint xyz twice, or forgetting the visual origin, or using the wrong rotation direction. The result: spheres shifted by ~200mm (one joint length), appearing inside the previous link.

### Verification

Always verify sphere placement in RViz before trusting the model:

1. Launch `visualize_xrdf_spheres.launch.py` (or equivalent)
2. Check: each sphere overlaps the corresponding mesh region
3. Move joints via `joint_state_publisher_gui` — spheres should follow the meshes

If a sphere floats in empty space or pokes through an unrelated link, your transform is wrong.

## Mimic joint limitation

cuRobo does NOT support URDF mimic joints. If you have:
```xml
<joint name="follower" type="prismatic">
  <mimic joint="driver" multiplier="0.5"/>
</joint>
```

cuRobo treats `follower` as **fixed at joint=0** regardless of `driver`'s actual value. This matters when the mimic joint moves collision geometry (e.g. a telescoping column middle section).

**Workarounds in order of effort:**

1. **Accept the error** — if the mimic joint moves a geometry that's always present along its full travel (e.g. a telescoping column section overlapping with other sections at joint=0), the positional error may not matter for collision outcomes. This is often the case.

2. **Add the mimic joint to cspace manually** with a linear constraint. cuRobo supports joint-space linear constraints:
   ```yaml
   cspace:
     joint_names: [driver, follower, ...]
     linear_constraints:
       - lhs: [-0.5, 1, 0, 0, 0, 0, 0]   # follower - 0.5*driver = 0
         rhs: 0
   ```

3. **Restructure the URDF** — collapse the mimic chain into a single prismatic joint with fixed child links. Cleanest long-term fix but requires URDF surgery.

## Ignore list rules

The XRDF `self_collision.ignore` list tells cuRobo to skip specific link pairs. Use it for:

- **Adjacent links that always overlap** (e.g. `wrist_2_link` × `wrist_3_link`)
- **Parent-child pairs connected by a joint** (e.g. `tool0` × `flange`, `tool0` × `tool_mount`)
- **Physically unreachable pairs** per SRDF `disable_collisions reason="Never"`

Do NOT use it to:
- Hide sphere over-approximation false positives (those will still be caught by post-FCL validation and surprise you)
- "Fix" apparent bugs without verifying with `sphere_blocker_analysis.py`

### Symmetry

cuRobo treats ignore entries as symmetric. `ignore: {A: [B]}` is equivalent to `ignore: {B: [A]}`. You only need it in one direction.

### Debugging the ignore list

If planning fails with `START_STATE_IN_COLLISION` consistently but FCL says the state is fine:
1. Run `tools/sphere_blocker_analysis.py` to find which specific sphere pair overlaps
2. If the pair is geometrically impossible to collide (e.g. base_link × wrist_3_link at full extension), add to ignore
3. If the pair is a real-but-rare collision, shrink the over-approximating sphere or accept the failure

## Typical sphere counts per link

For reference (UR10e, fitted with VAMP or good k-means):
- Base/mount: 5-15 spheres
- Upper arm: 5-8 spheres along the tube
- Forearm: 5-8 spheres along the tube
- Wrist links: 1-2 spheres each
- End-effector (tool): 10-40 spheres depending on shape complexity

Total per manipulator: ~60-100 spheres. More than 200 can slow cuRobo warmup significantly.

## Workflow summary

1. Get the mesh (STL) and its URDF `<collision>` or `<visual>` origin
2. Generate candidate spheres in mesh-local frame (VAMP or k-means, see `tools/generate_spheres_from_mesh.py`)
3. Apply visual origin transform → link frame
4. Apply any parent-joint inverse transform → target frame (if attaching to a descendant link)
5. Write into XRDF
6. Visualize in RViz, compare to mesh
7. Iterate until aligned
8. Run validation dataset, check success rate + FCL-clean rate
9. Use blocker analysis to shrink oversized spheres or adjust ignore list
