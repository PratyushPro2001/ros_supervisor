from __future__ import annotations

import threading
from typing import Any, Dict, List, Tuple

import rclpy
from rclpy.node import Node

from backend.core.ros_context import init_once

_ADAPTER_LOCK = threading.Lock()
_ADAPTER: "ROSAdapter | None" = None


def _graph_warmup(node: Node):
    # best-effort spin to refresh graph caches
    try:
        rclpy.spin_once(node, timeout_sec=0.05)
    except Exception:
        pass


class ROSAdapter:
    """
    Stable, singleton ROS access layer.
    IMPORTANT: keep method names compatible with the rest of the codebase.
    """

    def __init__(self):
        init_once()
        self._node = Node("ros_supervisor_adapter")

    # ---- Nodes ----
    def get_nodes(self) -> List[str]:
        """Return list of node names (no namespaces)."""
        _graph_warmup(self._node)
        return [n for (n, _ns) in self._node.get_node_names_and_namespaces()]

    # ---- Topics ----
    def get_topic_names_and_types(self) -> List[Tuple[str, List[str]]]:
        _graph_warmup(self._node)
        return [(name, list(types)) for (name, types) in self._node.get_topic_names_and_types()]

    def get_topics(self) -> List[Dict[str, Any]]:
        """
        Compatibility helper used by graph_model.py.
        Returns: [{"name": "/topic", "types": ["pkg/msg/Type", ...]}, ...]
        """
        out: List[Dict[str, Any]] = []
        for name, types in self.get_topic_names_and_types():
            out.append({"name": name, "types": types})
        return out

    def get_publishers_by_topic(self, topic_name: str) -> List[str]:
        """Return publisher node names for a given topic."""
        _graph_warmup(self._node)
        pubs = self._node.get_publishers_info_by_topic(topic_name)
        return sorted({p.node_name for p in pubs})

    def get_subscribers_by_topic(self, topic_name: str) -> List[str]:
        """Return subscriber node names for a given topic."""
        _graph_warmup(self._node)
        subs = self._node.get_subscriptions_info_by_topic(topic_name)
        return sorted({s.node_name for s in subs})

    # ---- Per-node pub/sub (used by node health, etc.) ----
    def get_publishers(self, node_name: str) -> List[str]:
        """Return topic names published by node_name."""
        _graph_warmup(self._node)
        out = []
        for tname, _types in self._node.get_topic_names_and_types():
            pubs = self._node.get_publishers_info_by_topic(tname)
            for p in pubs:
                if p.node_name == node_name:
                    out.append(tname)
        return sorted(set(out))

    def get_subscribers(self, node_name: str) -> List[str]:
        """Return topic names subscribed by node_name."""
        _graph_warmup(self._node)
        out = []
        for tname, _types in self._node.get_topic_names_and_types():
            subs = self._node.get_subscriptions_info_by_topic(tname)
            for s in subs:
                if s.node_name == node_name:
                    out.append(tname)
        return sorted(set(out))

    # ---- TF (MVP stub) ----
    def get_tf_tree(self) -> Dict[str, Any]:
        """
        MVP-safe stub so /snapshot never fails.
        We'll wire real TF metrics later via /tf/report + /tf/validate.
        """
        return {"frames": [], "edges": [], "roots": [], "children_map": {}}

    def destroy(self):
        try:
            self._node.destroy_node()
        except Exception:
            pass


def create_ros_adapter() -> ROSAdapter:
    global _ADAPTER
    with _ADAPTER_LOCK:
        if _ADAPTER is None:
            _ADAPTER = ROSAdapter()
        return _ADAPTER
