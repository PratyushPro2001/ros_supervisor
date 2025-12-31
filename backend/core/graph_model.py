from typing import Dict, Any, List

from backend.core.ros_adapter import ROSAdapter


def build_system_snapshot(adapter: ROSAdapter) -> Dict[str, Any]:
    """
    Build a simple system snapshot for the MVP.

    Shape:
    {
      "nodes": [
        "/talker",
        "/listener",
        ...
      ],
      "topics": [
        {
          "name": "/chatter",
          "types": ["std_msgs/msg/String"],
          "publishers": ["/talker"],
          "subscribers": ["/listener"]
        },
        ...
      ],
      "tf_tree": {
        "frames": [...],
        "errors": [...]
      }
    }
    """
    nodes: List[str] = adapter.get_nodes()
    raw_topics = adapter.get_topics()

    topics: List[Dict[str, Any]] = []
    for t in raw_topics:
        name = t["name"]
        types = t["types"]
        publishers = adapter.get_publishers(name)
        subscribers = adapter.get_subscribers(name)
        topics.append(
            {
                "name": name,
                "types": types,
                "publishers": publishers,
                "subscribers": subscribers,
            }
        )

    tf_tree = adapter.get_tf_tree()

    snapshot: Dict[str, Any] = {
        "nodes": nodes,
        "topics": topics,
        "tf_tree": tf_tree,
    }
    return snapshot
