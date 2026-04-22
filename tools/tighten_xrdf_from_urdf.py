#!/usr/bin/env python3
"""Tighten every link's spheres in an XRDF against its URDF collision mesh.

Reads an XRDF + URDF, for each link with spheres:
  1. Finds the link's collision mesh from the URDF
  2. Samples mesh surface points
  3. For each existing sphere, computes the tightest enclosing radius
     against all mesh points assigned to it (Voronoi-style)
  4. Writes a new XRDF with tightened radii (sphere centers unchanged)

Optionally also runs full FOAM/VAMP regeneration per link with --regenerate.

Usage:
    # Tighten radii, keep centers
    tighten_xrdf_from_urdf.py --urdf robot.urdf --xrdf-in robot.xrdf \\
        --xrdf-out robot_tight.xrdf

    # Regenerate from scratch with FOAM
    tighten_xrdf_from_urdf.py --urdf robot.urdf --xrdf-in robot.xrdf \\
        --xrdf-out robot_new.xrdf --regenerate --method foam --n 40

    # Only specific links
    tighten_xrdf_from_urdf.py --urdf robot.urdf --xrdf-in robot.xrdf \\
        --xrdf-out robot_tight.xrdf --links forearm_link wrist_1_link
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import yaml
except ImportError:
    sys.exit("ERROR: pip install pyyaml")

try:
    import trimesh
    from trimesh.sample import sample_surface
except ImportError:
    sys.exit("ERROR: pip install trimesh")


# ── URDF parsing helpers ──────────────────────────────────────────────────────


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


def parse_origin(elem) -> Tuple[List[float], List[float]]:
    if elem is None:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    xyz = [float(v) for v in elem.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in elem.get("rpy", "0 0 0").split()]
    return xyz, rpy


def resolve_mesh_path(filename: str, urdf_dir: Path, package_roots: Dict[str, Path]) -> Optional[Path]:
    """Resolve package:// or relative mesh path. Pass a {package_name: path} dict."""
    if filename.startswith("package://"):
        rest = filename[len("package://"):]
        pkg, subpath = rest.split("/", 1)
        if pkg in package_roots:
            return package_roots[pkg] / subpath
        return None
    if Path(filename).is_absolute():
        return Path(filename)
    return urdf_dir / filename


def get_collision_mesh(urdf_root, link_name: str, urdf_dir: Path,
                       package_roots: Dict[str, Path]) -> Optional[Tuple[Path, np.ndarray, np.ndarray]]:
    """Return (mesh_path, origin_xyz, origin_rot) for a link's collision mesh."""
    for link in urdf_root.iter("link"):
        if link.get("name") != link_name:
            continue
        col = link.find("collision")
        if col is None:
            return None
        origin_xyz, origin_rpy = parse_origin(col.find("origin"))
        geom = col.find("geometry")
        if geom is None:
            return None
        mesh = geom.find("mesh")
        if mesh is None:
            return None
        fn = mesh.get("filename")
        path = resolve_mesh_path(fn, urdf_dir, package_roots)
        if path is None or not path.exists():
            return None
        return path, np.array(origin_xyz), rpy_to_mat(origin_rpy)
    return None


# ── Core tightening logic ────────────────────────────────────────────────────


def tighten_link_spheres(
    spheres: List[dict],
    mesh_path: Path,
    visual_origin_xyz: np.ndarray,
    visual_origin_rot: np.ndarray,
    sample_count: int = 10000,
    margin: float = 1.02,
) -> List[dict]:
    """Keep sphere centers, recompute radii via Voronoi assignment."""
    mesh = trimesh.load(mesh_path)
    points_mesh, _ = sample_surface(mesh, sample_count)
    # Transform mesh points into link frame: p_link = visual_rot @ p_mesh + visual_xyz
    points_link = (visual_origin_rot @ points_mesh.T).T + visual_origin_xyz

    centers = np.array([s["center"] for s in spheres])
    if len(centers) == 0:
        return []
    # Assign each point to nearest sphere center
    dists = np.linalg.norm(points_link[:, None, :] - centers[None, :, :], axis=2)
    assign = dists.argmin(axis=1)

    out = []
    for i, s in enumerate(spheres):
        cluster = points_link[assign == i]
        if len(cluster) == 0:
            out.append(s)  # no points assigned; keep original
            continue
        r_new = float(np.linalg.norm(cluster - centers[i], axis=1).max()) * margin
        out.append({"center": list(centers[i]), "radius": r_new})
    return out


def regenerate_link_spheres(
    mesh_path: Path,
    visual_origin_xyz: np.ndarray,
    visual_origin_rot: np.ndarray,
    method: str,
    n: int,
) -> List[dict]:
    """Fresh sphere generation via FOAM/VAMP/k-means, transformed into link frame."""
    mesh = trimesh.load(mesh_path)

    if method == "foam":
        try:
            import foam
        except ImportError:
            sys.exit("ERROR: pip install git+https://github.com/CoMMALab/foam")
        result = foam.decompose(mesh, num_spheres=n)
        raw = [{"center": list(s.center), "radius": float(s.radius)} for s in result]
    elif method == "vamp":
        try:
            import vamp
        except ImportError:
            sys.exit("ERROR: pip install vamp")
        vs = vamp.sphere_decomposition(str(mesh_path), n_spheres=n)
        raw = [{"center": list(s["center"]), "radius": float(s["radius"])} for s in vs]
    elif method == "kmeans":
        from sklearn.cluster import KMeans
        points, _ = sample_surface(mesh, 5000)
        try:
            interior = mesh.voxelized(pitch=0.015).fill().points
            points = np.vstack([points, interior])
        except Exception:
            pass
        km = KMeans(n_clusters=n, n_init=10, random_state=42).fit(points)
        raw = []
        for ci in range(n):
            cl = points[km.labels_ == ci]
            r = float(np.linalg.norm(cl - km.cluster_centers_[ci], axis=1).max()) * 1.02
            raw.append({"center": km.cluster_centers_[ci].tolist(), "radius": r})
    else:
        raise ValueError(f"unknown method: {method}")

    # Transform from mesh-local → link frame via visual origin
    out = []
    for s in raw:
        c = visual_origin_rot @ np.array(s["center"]) + visual_origin_xyz
        out.append({"center": [float(c[0]), float(c[1]), float(c[2])],
                    "radius": float(s["radius"])})
    return out


# ── XRDF read/write ──────────────────────────────────────────────────────────


def load_xrdf(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def save_xrdf(doc: dict, path: Path) -> None:
    with open(path, "w") as f:
        yaml.dump(doc, f, default_flow_style=False, allow_unicode=False, width=200)
    # Sanity-check ASCII-clean (cuRobo reads XRDF as ASCII)
    path.read_bytes().decode("ascii")


def get_sphere_group(xrdf: dict) -> dict:
    geom = xrdf.get("geometry", {})
    # XRDF typically has one or more geometry groups; use the first
    for group_name, group in geom.items():
        if "spheres" in group:
            return group["spheres"]
    raise KeyError("no sphere geometry group found in XRDF")


# ── Main ──────────────────────────────────────────────────────────────────────


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", required=True, type=Path)
    ap.add_argument("--xrdf-in", required=True, type=Path)
    ap.add_argument("--xrdf-out", required=True, type=Path)
    ap.add_argument(
        "--links",
        nargs="*",
        default=None,
        help="Specific links to process (default: all links with non-empty sphere lists).",
    )
    ap.add_argument(
        "--package-root",
        action="append",
        default=[],
        help="Map a ROS package name to a path: 'pkg_name=/abs/path'. Repeat for multiple.",
    )
    ap.add_argument(
        "--regenerate",
        action="store_true",
        help="Fresh sphere generation (default: tighten existing radii only).",
    )
    ap.add_argument("--method", default="foam", choices=["foam", "vamp", "kmeans"])
    ap.add_argument("--n", type=int, default=30, help="Number of spheres when --regenerate.")
    args = ap.parse_args()

    # Parse package roots
    package_roots: Dict[str, Path] = {}
    for spec in args.package_root:
        name, path = spec.split("=", 1)
        package_roots[name] = Path(path).resolve()

    urdf_tree = ET.parse(args.urdf)
    urdf_root = urdf_tree.getroot()
    urdf_dir = args.urdf.resolve().parent

    xrdf = load_xrdf(args.xrdf_in)
    spheres_map = get_sphere_group(xrdf)

    links_to_process = args.links or [l for l, ss in spheres_map.items() if ss]
    print(f"Processing {len(links_to_process)} links ({'regenerate' if args.regenerate else 'tighten'})")

    for link in links_to_process:
        existing = spheres_map.get(link, [])
        if not args.regenerate and not existing:
            print(f"  {link}: no existing spheres, skipping (use --regenerate to create)")
            continue

        info = get_collision_mesh(urdf_root, link, urdf_dir, package_roots)
        if info is None:
            print(f"  {link}: no collision mesh found in URDF, skipping")
            continue
        mesh_path, origin_xyz, origin_rot = info

        if args.regenerate:
            new_spheres = regenerate_link_spheres(mesh_path, origin_xyz, origin_rot, args.method, args.n)
        else:
            new_spheres = tighten_link_spheres(existing, mesh_path, origin_xyz, origin_rot)

        spheres_map[link] = new_spheres

        # Stats
        old_max = max((s["radius"] for s in existing), default=0.0)
        new_max = max((s["radius"] for s in new_spheres), default=0.0)
        print(
            f"  {link}: {len(existing)} → {len(new_spheres)} spheres, "
            f"max R {old_max*1000:.0f}mm → {new_max*1000:.0f}mm"
        )

    save_xrdf(xrdf, args.xrdf_out)
    print(f"\nWrote {args.xrdf_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
