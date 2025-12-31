import json
import sys
from typing import Any, Dict, List

import rclpy

from backend.core.ros_adapter import create_ros_adapter, ROSAdapter
from backend.core.graph_model import build_system_snapshot


def inspect_node(adapter: ROSAdapter, node_name: str) -> Dict[str, Any]:
    """
    Derive information about a single node from the system snapshot:
      - which topics it publishes
      - which topics it subscribes to
      - (later) parameters
    """
    snapshot = build_system_snapshot(adapter)
    all_nodes: List[str] = snapshot["nodes"]
    topics: List[Dict[str, Any]] = snapshot["topics"]

    # Normalize node name: allow "talker" or "/talker"
    if node_name not in all_nodes:
        if not node_name.startswith("/"):
            candidate = f"/{node_name}"
            if candidate in all_nodes:
                node_name = candidate
            else:
                raise ValueError(f"Node not found: {node_name}")
        else:
            raise ValueError(f"Node not found: {node_name}")

    publishes: List[str] = []
    subscribes: List[str] = []

    for t in topics:
        t_name = t["name"]
        pubs = t.get("publishers", [])
        subs = t.get("subscribers", [])

        if node_name in pubs:
            publishes.append(t_name)
        if node_name in subs:
            subscribes.append(t_name)

    # Parameters are still stubbed
    params = adapter.get_node_parameters(node_name)

    return {
        "name": node_name,
        "parameters": params,
        "publishes": publishes,
        "subscribes": subscribes,
    }


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]

    if not argv:
        print("Usage: python3 -m backend.scripts.inspect_node <node_name>", file=sys.stderr)
        sys.exit(1)

    raw_name = argv[0]

    adapter = create_ros_adapter()

    try:
        info = inspect_node(adapter, raw_name)
        print(json.dumps(info, indent=2, sort_keys=True))
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
