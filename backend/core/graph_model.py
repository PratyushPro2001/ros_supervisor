from typing import Dict, Any, List

from backend.core.ros_adapter import ROSAdapter


def build_system_snapshot(adapter: ROSAdapter) -> Dict[str, Any]:
    """
    MVP graph snapshot used by /graph and the snapshot daemon.

    Output shape:
    {
      "nodes": [{"name": "turtlesim"}, ...],
      "topics": [{"name": "/rosout", "types": ["rcl_interfaces/msg/Log"]}, ...]
    }

    IMPORTANT:
    - Only sorts by strings (NOT dicts), to avoid TypeError: '<' between dicts.
    - Keep it lightweight for realtime polling.
    """
    # Nodes (name only)
    nodes_raw = adapter._node.get_node_names_and_namespaces()
    nodes: List[Dict[str, Any]] = [{"name": n} for (n, _ns) in nodes_raw]
    nodes.sort(key=lambda x: x["name"])

    # Topics (name + types)
    topics_raw = adapter._node.get_topic_names_and_types()
    topics: List[Dict[str, Any]] = [{"name": name, "types": list(types)} for (name, types) in topics_raw]
    topics.sort(key=lambda x: x["name"])

    return {"nodes": nodes, "topics": topics}
