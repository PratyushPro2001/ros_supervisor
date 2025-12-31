import json
import sys

import rclpy

from backend.core.ros_adapter import create_ros_adapter, ROSAdapter
from backend.core.graph_model import build_system_snapshot


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]

    adapter: ROSAdapter = create_ros_adapter()
    snapshot = build_system_snapshot(adapter)

    print(json.dumps(snapshot, indent=2, sort_keys=True))

    rclpy.shutdown()


if __name__ == "__main__":
    main()
