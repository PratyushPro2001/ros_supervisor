from __future__ import annotations

import time
from typing import Any, Dict, List, Set, Tuple

import rclpy
import threading
from rclpy.node import Node

# FastAPI may serve multiple requests concurrently; rclpy spin_once is not re-entrant.
_SPIN_LOCK = threading.Lock()

from rclpy.qos import QoSProfile, DurabilityPolicy

from tf2_msgs.msg import TFMessage


class _TFInspectorNode(Node):
    def __init__(self, duration_sec: float):
        super().__init__("tf_inspector")
        self.duration_sec = duration_sec

        self.edges: Set[Tuple[str, str]] = set()
        self.frames: Set[str] = set()

        # Dynamic TF: default QoS
        self._sub_tf = self.create_subscription(
            TFMessage,
            "/tf",
            self._cb,
            10,
        )

        # Static TF: must be TRANSIENT_LOCAL to receive latched transforms
        static_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._sub_tf_static = self.create_subscription(
            TFMessage,
            "/tf_static",
            self._cb,
            static_qos,
        )

    def _cb(self, msg: TFMessage):
        for t in msg.transforms:
            parent = t.header.frame_id.strip()
            child = t.child_frame_id.strip()
            if not parent or not child:
                continue
            self.edges.add((parent, child))
            self.frames.add(parent)
            self.frames.add(child)

    def collect(self) -> Dict[str, Any]:
        end = time.time() + self.duration_sec
        while rclpy.ok() and time.time() < end:
            with _SPIN_LOCK:
                rclpy.spin_once(self, timeout_sec=0.1)
        children_map: Dict[str, List[str]] = {}
        parents_map: Dict[str, List[str]] = {}
        for p, c in sorted(self.edges):
            children_map.setdefault(p, []).append(c)
            parents_map.setdefault(c, []).append(p)

        roots = [f for f in sorted(self.frames) if f not in parents_map]

        return {
            "frames": sorted(self.frames),
            "edges": [{"parent": p, "child": c} for (p, c) in sorted(self.edges)],
            "roots": roots,
            "children_map": children_map,
        }


def inspect_tf(duration_sec: float = 3.0) -> Dict[str, Any]:
    node = _TFInspectorNode(duration_sec=duration_sec)
    try:
        data = node.collect()
    finally:
        node.destroy_node()
    return data
