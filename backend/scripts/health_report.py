import json

import rclpy

from backend.core.ros_adapter import create_ros_adapter
from backend.core.graph_model import build_system_snapshot
from backend.core.node_health import build_node_health


def main():
    adapter = create_ros_adapter()
    snapshot = build_system_snapshot(adapter)
    report = build_node_health(snapshot, adapter, rate_window_sec=3.0)

    print(json.dumps(report, indent=2, sort_keys=True))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
