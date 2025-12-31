import json
import sys
import time

import rclpy

from backend.core.param_reader import read_all_parameters
from backend.core.param_diff import diff_params


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]

    if not argv:
        print("Usage: python3 -m backend.scripts.params_diff <node_name> [delay_sec]", file=sys.stderr)
        sys.exit(1)

    node = argv[0]
    delay = float(argv[1]) if len(argv) > 1 else 5.0

    rclpy.init(args=None)
    try:
        before = read_all_parameters(node)
        time.sleep(delay)
        after = read_all_parameters(node)

        diff = diff_params(before, after)
        print(json.dumps(diff, indent=2, sort_keys=True))
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
