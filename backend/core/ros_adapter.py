from __future__ import annotations

import time
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node


def _graph_warmup(node: Node, seconds: float = 0.8):
    """Spin briefly so ROS graph discovery settles."""
    end = time.time() + seconds
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)


class ROSAdapter:
    """
    Thin wrapper around rclpy Node graph/introspection APIs.
    Must preserve the interface expected by backend.core.graph_model.
    """

    def __init__(self):
        # Safe to call multiple times; rclpy will ignore if already init'd in-process.
        try:
            rclpy.init(args=None)
        except Exception:
            pass
        self._node = Node("ros_supervisor_adapter")

    # ---- Graph basics ----
    def get_nodes(self) -> List[str]:
        _graph_warmup(self._node)
        # rclpy returns list[str] of node names (without namespaces sometimes depending on API),
        # we keep it as-is and let callers normalize if needed.
        return list(self._node.get_node_names())

    def get_topic_names_and_types(self) -> List[Dict[str, Any]]:
        _graph_warmup(self._node)
        out: List[Dict[str, Any]] = []
        for name, types in self._node.get_topic_names_and_types():
            out.append({"name": name, "types": list(types)})
        return out

    def get_publishers(self, topic_name: str) -> List[str]:
        _graph_warmup(self._node)
        infos = self._node.get_publishers_info_by_topic(topic_name)
        return [i.node_name for i in infos]

    def get_subscribers(self, topic_name: str) -> List[str]:
        _graph_warmup(self._node)
        infos = self._node.get_subscriptions_info_by_topic(topic_name)
        return [i.node_name for i in infos]

    def get_topics(self) -> List[Dict[str, Any]]:
        """
        Convenience method used by some scripts: returns topics with types + pub/sub lists.
        """
        _graph_warmup(self._node)
        topics: List[Dict[str, Any]] = []
        for name, types in self._node.get_topic_names_and_types():
            pubs = self._node.get_publishers_info_by_topic(name)
            subs = self._node.get_subscriptions_info_by_topic(name)
            topics.append(
                {
                    "name": name,
                    "types": list(types),
                    "publishers": [p.node_name for p in pubs],
                    "subscribers": [s.node_name for s in subs],
                }
            )
        return topics

    # ---- TF ----
    def get_tf_tree(self) -> Dict[str, Any]:
        """Return a TF validation snapshot using the TF inspector/validator."""
        from backend.core.tf_inspector import inspect_tf
        from backend.core.tf_validator import validate_tf

        report = inspect_tf(duration_sec=1.0)
        result = validate_tf(report)
        # graph_model expects {"frames": ..., "errors": ...}
        return {
            "frames": report.get("frames", []),
            "edges": report.get("edges", []),
            "roots": report.get("roots", []),
            "errors": result.get("errors", []),
            "warnings": result.get("warnings", []),
        }

    # ---- Parameters ----
    def get_node_parameters(self, node_name: str) -> Dict[str, Any]:
        from backend.core.param_reader import read_all_parameters
        return read_all_parameters(node_name)


def create_ros_adapter() -> ROSAdapter:
    return ROSAdapter()
