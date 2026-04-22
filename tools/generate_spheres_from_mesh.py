#!/usr/bin/env python3
"""Generate collision spheres for a mesh and output XRDF-ready JSON.

Supports three backends:
  --method foam    (recommended) medial-axis packing via CoMMALab/foam
  --method vamp    equivalent; uses VAMP's wrapper around FOAM
  --method kmeans  quick k-means over surface samples (fallback)
  --method tighten keep given centers, recompute radii to tightest Voronoi fit

Applies optional link-frame transforms so the spheres are in the correct
XRDF coordinate system (not mesh-local).

Usage:
    generate_spheres_from_mesh.py --mesh sander.stl --n 40 --method foam \\
        --visual-origin "0 0 0.2039" \\
        --joint-to-parent "0 0 0.2748 / 0 0 3.14159" \\
        --out tool0_spheres.json
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import List, Tuple

import numpy as np


def rpy_to_mat(rpy: List[float]) -> np.ndarray:
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr               ],
    ])


def parse_xyz(s: str) -> np.ndarray:
    parts = s.strip().split()
    if len(parts) != 3:
        raise ValueError(f"expected 3 floats, got {s!r}")
    return np.array([float(x) for x in parts])


def parse_joint(s: str) -> Tuple[np.ndarray, np.ndarray]:
    """Format: 'x y z / r p y'. Returns (xyz, rot_matrix)."""
    xyz_part, rpy_part = s.split("/")
    xyz = parse_xyz(xyz_part)
    rpy = parse_xyz(rpy_part)
    return xyz, rpy_to_mat(rpy)


def generate_foam(mesh, n_spheres: int) -> List[dict]:
    """FOAM medial-axis sphere packing. Preferred method."""
    try:
        import foam  # noqa: F401
    except ImportError:
        sys.exit(
            "ERROR: foam not installed. Try: pip install git+https://github.com/CoMMALab/foam\n"
            "Or fall back to --method kmeans."
        )
    # API as of FOAM ~0.3; adjust if upstream changes
    result = foam.decompose(mesh, num_spheres=n_spheres)
    return [{"center": list(s.center), "radius": float(s.radius)} for s in result]


def generate_vamp(mesh_path: str, n_spheres: int) -> List[dict]:
    """VAMP's wrapper around FOAM. Same output quality as FOAM directly."""
    try:
        import vamp
    except ImportError:
        sys.exit(
            "ERROR: vamp not installed. Try: pip install vamp\n"
            "Or use --method foam (direct) or --method kmeans (fallback)."
        )
    spheres = vamp.sphere_decomposition(str(mesh_path), n_spheres=n_spheres)
    return [{"center": list(s["center"]), "radius": float(s["radius"])} for s in spheres]


def generate_kmeans(mesh, n_spheres: int, sample_count: int = 5000) -> List[dict]:
    """K-means on mesh surface samples. Quick fallback — uniform radii."""
    try:
        from sklearn.cluster import KMeans
        from trimesh.sample import sample_surface
    except ImportError:
        sys.exit("ERROR: pip install scikit-learn trimesh")

    points, _ = sample_surface(mesh, sample_count)

    # Also sample inside via voxelization for better interior coverage
    try:
        voxels = mesh.voxelized(pitch=0.015).fill()
        interior = voxels.points
        points = np.vstack([points, interior])
    except Exception:
        pass  # if voxelization fails, surface-only is fine

    km = KMeans(n_clusters=n_spheres, n_init=10, random_state=42).fit(points)
    spheres = []
    for ci in range(n_spheres):
        cluster = points[km.labels_ == ci]
        r = float(np.linalg.norm(cluster - km.cluster_centers_[ci], axis=1).max()) * 1.02
        spheres.append({"center": km.cluster_centers_[ci].tolist(), "radius": r})
    spheres.sort(key=lambda s: -s["radius"])
    return spheres


def tighten(existing: List[dict], mesh, sample_count: int = 10000) -> List[dict]:
    """Keep centers, recompute radii to tightest Voronoi fit."""
    try:
        from trimesh.sample import sample_surface
    except ImportError:
        sys.exit("ERROR: pip install trimesh")

    points, _ = sample_surface(mesh, sample_count)
    centers = np.array([s["center"] for s in existing])
    dists = np.linalg.norm(points[:, None] - centers[None, :], axis=2)  # (N, K)
    assign = dists.argmin(axis=1)  # each sample to nearest center

    tightened = []
    for i, s in enumerate(existing):
        cluster = points[assign == i]
        if len(cluster) == 0:
            tightened.append(s)
            continue
        r = float(np.linalg.norm(cluster - centers[i], axis=1).max()) * 1.02
        tightened.append({"center": list(centers[i]), "radius": r})
    return tightened


def apply_transforms(
    spheres: List[dict],
    visual_origin: np.ndarray,  # mesh-local → link that owns the mesh
    visual_rot: np.ndarray,
    joint_xyz: np.ndarray = None,  # optional: that-link → target-link via inverse joint
    joint_rot: np.ndarray = None,
) -> List[dict]:
    """Transform spheres from mesh-local frame into the XRDF target link frame.

    Step 1: apply visual origin → mesh_link frame
    Step 2 (optional): apply inverse joint transform → child link frame
    """
    out = []
    for s in spheres:
        c = np.array(s["center"])
        c = visual_rot @ c + visual_origin   # mesh → mesh_link
        if joint_xyz is not None:
            c = joint_rot.T @ (c - joint_xyz)  # mesh_link → child link (inverse)
        out.append({"center": [float(c[0]), float(c[1]), float(c[2])],
                    "radius": float(s["radius"])})
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mesh", required=True, help="STL/OBJ/PLY path")
    ap.add_argument("--method", default="foam", choices=["foam", "vamp", "kmeans", "tighten"])
    ap.add_argument("--n", type=int, default=40, help="Number of spheres (ignored for tighten)")
    ap.add_argument("--existing", help="JSON with existing spheres (required for --method tighten)")
    ap.add_argument(
        "--visual-origin",
        default="0 0 0",
        help="Mesh origin in owning link's frame: 'x y z'",
    )
    ap.add_argument("--visual-rpy", default="0 0 0", help="Mesh RPY in owning link: 'r p y'")
    ap.add_argument(
        "--joint-to-parent",
        default=None,
        help=(
            "Optional: joint transform to go from mesh's owning link → target link. "
            "Format: 'x y z / r p y'. The inverse is applied."
        ),
    )
    ap.add_argument("--out", required=True, help="Output JSON path")
    args = ap.parse_args()

    try:
        import trimesh
    except ImportError:
        sys.exit("ERROR: pip install trimesh")

    mesh = trimesh.load(args.mesh)

    if args.method == "foam":
        raw = generate_foam(mesh, args.n)
    elif args.method == "vamp":
        raw = generate_vamp(args.mesh, args.n)
    elif args.method == "kmeans":
        raw = generate_kmeans(mesh, args.n)
    elif args.method == "tighten":
        if not args.existing:
            sys.exit("--method tighten requires --existing spheres.json")
        with open(args.existing) as f:
            raw = json.load(f)
        raw = tighten(raw, mesh)
    else:
        raise ValueError(args.method)

    visual_xyz = parse_xyz(args.visual_origin)
    visual_rot = rpy_to_mat(parse_xyz(args.visual_rpy).tolist())
    joint_xyz = joint_rot = None
    if args.joint_to_parent:
        joint_xyz, joint_rot = parse_joint(args.joint_to_parent)

    transformed = apply_transforms(raw, visual_xyz, visual_rot, joint_xyz, joint_rot)

    with open(args.out, "w") as f:
        json.dump(transformed, f, indent=2)

    zs = [s["center"][2] for s in transformed]
    rs = [s["radius"] for s in transformed]
    print(f"Generated {len(transformed)} spheres via {args.method}")
    print(f"  Z range: [{min(zs):.3f}, {max(zs):.3f}] m")
    print(f"  R range: [{min(rs)*1000:.1f}, {max(rs)*1000:.1f}] mm, mean {sum(rs)/len(rs)*1000:.1f} mm")
    print(f"  Saved to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
