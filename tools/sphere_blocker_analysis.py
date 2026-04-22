#!/usr/bin/env python3
"""Find which specific XRDF spheres block the most planning pairs.

For each pair in the dataset, linearly interpolates start → goal in joint
space and checks which non-ignored sphere-pair combinations collide along
the interpolated path. Aggregates across all pairs to identify the worst
offenders — spheres that block many trajectories, not just one.

The output lets you target sphere shrinking or ignore-list additions based
on real blocker impact rather than guessing.

Usage:
    python3 sphere_blocker_analysis.py \\
        --dataset pairs.json --xrdf robot.xrdf --urdf robot.urdf \\
        --n-waypoints 32 --top 20
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

try:
    import yaml
except ImportError:
    sys.exit("ERROR: pip install pyyaml")


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


def axis_angle_mat(axis: List[float], angle: float) -> np.ndarray:
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    t = 1 - c
    return np.array([
        [t*x*x + c,   t*x*y - s*z, t*x*z + s*y],
        [t*x*y + s*z, t*y*y + c,   t*y*z - s*x],
        [t*x*z - s*y, t*y*z + s*x, t*z*z + c  ],
    ])


class UrdfFK:
    """Minimal forward-kinematics from a URDF. Supports revolute + prismatic."""

    def __init__(self, urdf_path: Path):
        self._root = ET.parse(urdf_path).getroot()
        self._joints: List[dict] = []
        self._joint_index: Dict[str, int] = {}
        for j in self._root.iter("joint"):
            name = j.get("name")
            jtype = j.get("type")
            parent = j.find("parent").get("link")
            child = j.find("child").get("link")
            origin = j.find("origin")
            xyz = [float(x) for x in (origin.get("xyz", "0 0 0").split() if origin is not None else "0 0 0".split())]
            rpy = [float(x) for x in (origin.get("rpy", "0 0 0").split() if origin is not None else "0 0 0".split())]
            axis_elem = j.find("axis")
            axis = [float(x) for x in (axis_elem.get("xyz").split() if axis_elem is not None else "1 0 0".split())]
            self._joints.append({
                "name": name, "type": jtype, "parent": parent,
                "child": child, "xyz": xyz, "rpy": rpy, "axis": axis,
            })

    def set_actuated(self, names: List[str]) -> None:
        self._actuated = names
        self._actuated_index = {n: i for i, n in enumerate(names)}

    def compute(self, q: List[float]) -> Dict[str, np.ndarray]:
        """Return {link_name: 4x4 transform from root link to this link}."""
        T_world: Dict[str, np.ndarray] = {}
        root_link = next((j["parent"] for j in self._joints), None)
        if root_link is None:
            return T_world
        T_world[root_link] = np.eye(4)

        # Process joints in order (URDFs are usually listed parent-first)
        for j in self._joints:
            parent_T = T_world.get(j["parent"])
            if parent_T is None:
                continue
            T = np.eye(4)
            T[:3, :3] = rpy_to_mat(j["rpy"])
            T[:3, 3] = j["xyz"]
            if j["type"] == "revolute" and j["name"] in self._actuated_index:
                angle = q[self._actuated_index[j["name"]]]
                Tj = np.eye(4)
                Tj[:3, :3] = axis_angle_mat(j["axis"], angle)
                T = T @ Tj
            elif j["type"] == "prismatic" and j["name"] in self._actuated_index:
                d = q[self._actuated_index[j["name"]]]
                Tj = np.eye(4)
                Tj[:3, 3] = [a * d for a in j["axis"]]
                T = T @ Tj
            T_world[j["child"]] = parent_T @ T
        return T_world


def load_spheres_and_ignores(xrdf_path: Path):
    with open(xrdf_path) as f:
        d = yaml.safe_load(f)
    # First geometry group with spheres
    group = next(iter(d["geometry"].values()))
    spheres = {k: v for k, v in group["spheres"].items() if v}

    ignore_cfg = d.get("self_collision", {}).get("ignore", {}) or {}
    ignore = set()
    for a, ps in ignore_cfg.items():
        for b in (ps or []):
            ignore.add(frozenset([a, b]))

    return spheres, ignore


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset", required=True, type=Path)
    ap.add_argument("--xrdf", required=True, type=Path)
    ap.add_argument("--urdf", required=True, type=Path)
    ap.add_argument("--joint-names", nargs="+", required=True,
                    help="Actuated joint names in order (must match dataset pair values)")
    ap.add_argument("--n-waypoints", type=int, default=32)
    ap.add_argument("--max-pairs", type=int, default=200)
    ap.add_argument("--top", type=int, default=20)
    args = ap.parse_args()

    with open(args.dataset) as f:
        data = json.load(f)
    pairs = data["pairs"][: args.max_pairs]

    spheres, ignore = load_spheres_and_ignores(args.xrdf)
    fk = UrdfFK(args.urdf)
    fk.set_actuated(args.joint_names)

    # Count how many pairs each (link_a[si], link_b[sj]) blocks
    blocker: Dict[Tuple[str, str, int, int], int] = defaultdict(int)

    for pair in pairs:
        start = np.array(pair["start"])
        goal = np.array(pair["goal"])
        seen_in_pair = set()
        for w in range(args.n_waypoints):
            alpha = w / (args.n_waypoints - 1)
            q = list(start + alpha * (goal - start))
            T = fk.compute(q)

            # Transform spheres into world frame
            world: Dict[str, List[Tuple[np.ndarray, float]]] = {}
            for link, sph_list in spheres.items():
                if link not in T:
                    continue
                link_T = T[link]
                world[link] = [
                    ((link_T @ np.array([*s["center"], 1.0]))[:3], float(s["radius"]))
                    for s in sph_list
                ]

            # Pairwise check (non-ignored only)
            links = list(world.keys())
            for i in range(len(links)):
                for j in range(i + 1, len(links)):
                    la, lb = links[i], links[j]
                    if frozenset([la, lb]) in ignore:
                        continue
                    for si, (ca, ra) in enumerate(world[la]):
                        for sj, (cb, rb) in enumerate(world[lb]):
                            if np.linalg.norm(ca - cb) < ra + rb:
                                key = (la, lb, si, sj)
                                if key in seen_in_pair:
                                    continue
                                seen_in_pair.add(key)
                                blocker[key] += 1

    print(f"\nTop {args.top} blocker sphere pairs across {len(pairs)} pairs:\n")
    print(f"{'Link A':<22} {'Link B':<22} {'Sph_A':>5} {'Sph_B':>5} {'Pairs':>6}  Radii")
    print("-" * 80)
    for (la, lb, si, sj), n in sorted(blocker.items(), key=lambda x: -x[1])[: args.top]:
        r_a = spheres[la][si]["radius"]
        r_b = spheres[lb][sj]["radius"]
        print(f"{la:<22} {lb:<22} {si:>5} {sj:>5} {n:>5}x  "
              f"R={r_a*1000:.0f}/{r_b*1000:.0f}mm")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
