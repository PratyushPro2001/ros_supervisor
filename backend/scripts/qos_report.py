import json

import rclpy

from backend.core.ros_adapter import create_ros_adapter
from backend.core.qos_inspector import get_qos_report


def main():
    adapter = create_ros_adapter()
    report = get_qos_report(adapter)
    print(json.dumps(report, indent=2, sort_keys=True))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
