from __future__ import annotations

from typing import Any, Dict, List

from backend.core.ros_adapter import ROSAdapter


def _serialize_qos_profile(qos) -> Dict[str, Any]:
    """
    Convert rclpy.qos.QoSProfile into a JSON-friendly dict.
    """
    # qos.* are enums or ints; convert enums to their names when possible
    def enum_name(x):
        return getattr(x, "name", str(x))

    return {
        "history": enum_name(qos.history),
        "depth": int(getattr(qos, "depth", 0)),
        "reliability": enum_name(qos.reliability),
        "durability": enum_name(qos.durability),
        "lifespan_ns": int(getattr(getattr(qos, "lifespan", None), "nanoseconds", 0)),
        "deadline_ns": int(getattr(getattr(qos, "deadline", None), "nanoseconds", 0)),
        "liveliness": enum_name(getattr(qos, "liveliness", "UNKNOWN")),
        "liveliness_lease_duration_ns": int(
            getattr(getattr(qos, "liveliness_lease_duration", None), "nanoseconds", 0)
        ),
        "avoid_ros_namespace_conventions": bool(
            getattr(qos, "avoid_ros_namespace_conventions", False)
        ),
    }


def _serialize_endpoint_info(info) -> Dict[str, Any]:
    """
    Convert TopicEndpointInfo into a JSON-friendly dict.
    """
    ns = getattr(info, "node_namespace", "")
    name = getattr(info, "node_name", "")
    full_node = f"/{name}" if ns in ("", "/") else f"{ns.rstrip('/')}/{name}"

    return {
        "node": full_node,
        "topic_type": getattr(info, "topic_type", None),
        "endpoint_type": getattr(getattr(info, "endpoint_type", None), "name", None),
        "qos": _serialize_qos_profile(getattr(info, "qos_profile", None)),
    }


def get_qos_report(adapter: ROSAdapter) -> Dict[str, Any]:
    """
    Build a QoS report for every topic:
    {
      "topics": [
        {
          "name": "/chatter",
          "publishers": [ {node, topic_type, qos...}, ...],
          "subscribers": [ {node, topic_type, qos...}, ...]
        },
        ...
      ]
    }
    """
    topics = adapter.get_topic_names_and_types()
    out: List[Dict[str, Any]] = []

    for topic_name, _types in topics:
        pubs_info = adapter.get_publishers_info_by_topic(topic_name)
        subs_info = adapter.get_subscriptions_info_by_topic(topic_name)

        out.append(
            {
                "name": topic_name,
                "publishers": [_serialize_endpoint_info(i) for i in pubs_info],
                "subscribers": [_serialize_endpoint_info(i) for i in subs_info],
            }
        )

    return {"topics": out}
