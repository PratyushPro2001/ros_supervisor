"""Persistent TF monitor: watches /tf and /tf_static continuously and reports the
frame tree, per-edge rate/age, and validation (multi-parent, cycles) every snapshot."""
from __future__ import annotations

import time
from collections import deque
from typing import Any, Dict, List, Set, Tuple

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

from backend.core.tf_validator import validate_tf

STALE_AFTER_SEC = 3.0


class TFMonitor:
    def __init__(self, node):
        self._edges: Dict[Tuple[str, str], deque] = {}
        self._static: Set[Tuple[str, str]] = set()
        dyn = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=100)
        stat = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          history=HistoryPolicy.KEEP_LAST, depth=100)
        node.create_subscription(TFMessage, "/tf", lambda m: self._cb(m, False), dyn)
        node.create_subscription(TFMessage, "/tf_static", lambda m: self._cb(m, True), stat)

    def _cb(self, msg, static: bool):
        now = time.monotonic()
        for t in msg.transforms:
            p = t.header.frame_id.strip().lstrip("/")
            c = t.child_frame_id.strip().lstrip("/")
            if not p or not c:
                continue
            e = (p, c)
            if static:
                self._static.add(e)
            self._edges.setdefault(e, deque(maxlen=500)).append(now)

    def report(self, window_sec: float = 5.0) -> Dict[str, Any]:
        now = time.monotonic()
        edges: List[Dict[str, Any]] = []
        frames: Set[str] = set()
        children: Dict[str, List[str]] = {}
        has_parent: Set[str] = set()
        stale: List[str] = []
        for (p, c), dq in sorted(self._edges.items()):
            frames.update((p, c))
            children.setdefault(p, []).append(c)
            has_parent.add(c)
            static = (p, c) in self._static
            stamps = [t for t in tuple(dq) if now - t <= window_sec]
            hz = round((len(stamps) - 1) / max(stamps[-1] - stamps[0], 1e-6), 1) if len(stamps) >= 2 else 0.0
            age = round(now - dq[-1], 2) if dq else None
            if not static and (age is None or age > STALE_AFTER_SEC):
                stale.append(f"{p} -> {c}")
            edges.append({"parent": p, "child": c, "static": static, "hz": hz, "age_sec": age})
        roots = sorted(f for f in frames if f not in has_parent)
        rep = {"frames": sorted(frames), "edges": edges, "roots": roots, "children_map": children}
        v = validate_tf(rep)
        # validate_tf warns about "no TF" - keep it, but the UI shows it as info.
        rep["validation"] = v
        rep["stale"] = stale
        return rep


def get_tf_monitor(adapter) -> TFMonitor:
    mon = getattr(adapter, "_tf_monitor", None)
    if mon is None:
        mon = TFMonitor(adapter._node)
        adapter._tf_monitor = mon
    return mon
