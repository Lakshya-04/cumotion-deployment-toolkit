#!/usr/bin/env python3
"""Publish XRDF collision spheres as RViz markers for visual verification.

Reads an XRDF, iterates every link's spheres, and publishes one marker per
sphere with `frame_id = link_name`. RViz + robot_state_publisher's TF tree
will render them at the correct world position automatically as the robot
moves — so you can visually confirm spheres track the meshes.

Works with ANY XRDF — no robot-specific code.

Usage:
    ros2 run <your_pkg> xrdf_sphere_publisher --ros-args -p xrdf_path:=/path/to/robot.xrdf

Or standalone:
    python3 xrdf_sphere_publisher.py --ros-args -p xrdf_path:=/path/to/robot.xrdf

Marker topic:    /xrdf_spheres            (MarkerArray, latched)
Color coding:
  - Normal spheres: translucent green
  - Ignored pair participants: translucent blue (less "hot")
"""

from __future__ import annotations

import argparse
import sys
from typing import Dict, List, Set

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, Pose, Point


class XrdfSpherePublisher(Node):
    def __init__(self):
        super().__init__("xrdf_sphere_publisher")

        self.declare_parameter("xrdf_path", "")
        self.declare_parameter("marker_topic", "/xrdf_spheres")
        self.declare_parameter("alpha", 0.35)
        self.declare_parameter("publish_rate_hz", 1.0)  # latched + periodic

        xrdf_path = self.get_parameter("xrdf_path").value
        if not xrdf_path:
            self.get_logger().error("Required param: xrdf_path")
            raise SystemExit(1)

        self._alpha = self.get_parameter("alpha").value
        topic = self.get_parameter("marker_topic").value

        # Parse XRDF
        with open(xrdf_path) as f:
            xrdf = yaml.safe_load(f)
        self._spheres = self._extract_spheres(xrdf)
        self._ignore_pairs = self._extract_ignore_pairs(xrdf)
        self._ignored_links = self._ignore_pair_flatten(self._ignore_pairs)

        total = sum(len(ss) for ss in self._spheres.values())
        self.get_logger().info(
            f"Loaded {total} spheres across {len(self._spheres)} links from {xrdf_path}"
        )

        # Latched QoS so late subscribers (e.g. RViz started after us) get the markers
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(MarkerArray, topic, qos)

        rate = self.get_parameter("publish_rate_hz").value
        self.create_timer(1.0 / max(rate, 0.1), self._publish)

        # Publish once immediately so subscribers started at launch don't wait
        self._publish()

    def _extract_spheres(self, xrdf: dict) -> Dict[str, List[dict]]:
        geom = xrdf.get("geometry", {}) or {}
        # First group that has 'spheres' wins
        for group in geom.values():
            sph = group.get("spheres")
            if sph:
                # Filter out empty lists
                return {link: items for link, items in sph.items() if items}
        return {}

    def _extract_ignore_pairs(self, xrdf: dict) -> Dict[str, List[str]]:
        return xrdf.get("self_collision", {}).get("ignore", {}) or {}

    def _ignore_pair_flatten(self, ignore: Dict[str, List[str]]) -> Set[str]:
        """Links that appear in any ignore pair (visual cue only)."""
        out: Set[str] = set()
        for a, partners in ignore.items():
            out.add(a)
            for b in (partners or []):
                out.add(b)
        return out

    def _publish(self) -> None:
        marker_array = MarkerArray()
        mid = 0
        now = self.get_clock().now().to_msg()

        for link_name, sph_list in self._spheres.items():
            is_ignored_link = link_name in self._ignored_links
            for i, s in enumerate(sph_list):
                m = Marker()
                m.header.stamp = now
                m.header.frame_id = link_name   # let TF handle world pose
                m.ns = f"xrdf_{link_name}"
                m.id = i
                m.type = Marker.SPHERE
                m.action = Marker.ADD

                center = s.get("center", [0.0, 0.0, 0.0])
                radius = float(s.get("radius", 0.01))
                m.pose = Pose()
                m.pose.position = Point(x=float(center[0]), y=float(center[1]), z=float(center[2]))
                m.pose.orientation.w = 1.0
                m.scale = Vector3(x=radius * 2.0, y=radius * 2.0, z=radius * 2.0)

                # Color: translucent green normally, translucent blue if ignored
                if is_ignored_link:
                    m.color = ColorRGBA(r=0.2, g=0.4, b=0.9, a=self._alpha)
                else:
                    m.color = ColorRGBA(r=0.2, g=0.85, b=0.3, a=self._alpha)
                m.lifetime.sec = 0  # forever

                marker_array.markers.append(m)
                mid += 1

        self._pub.publish(marker_array)


def main() -> int:
    rclpy.init()
    try:
        node = XrdfSpherePublisher()
    except SystemExit as e:
        rclpy.shutdown()
        return int(e.code) if e.code else 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
