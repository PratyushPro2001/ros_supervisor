import json
import sys

import rclpy

from backend.core.param_reader import read_all_parameters


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]

    if not argv:
        print("Usage: python3 -m backend.scripts.params_show <node_name>", file=sys.stderr)
        sys.exit(1)

    node_name = argv[0]

    rclpy.init(args=None)
    try:
        params = read_all_parameters(node_name)
        print(json.dumps({"node": node_name, "parameters": params}, indent=2, sort_keys=True))
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
