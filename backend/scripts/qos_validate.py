import json

import rclpy

from backend.core.ros_adapter import create_ros_adapter
from backend.core.qos_inspector import get_qos_report
from backend.core.qos_validator import find_qos_mismatches


def main():
    adapter = create_ros_adapter()
    report = get_qos_report(adapter)
    mismatches = find_qos_mismatches(report)

    print(json.dumps({"mismatches": mismatches}, indent=2, sort_keys=True))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
