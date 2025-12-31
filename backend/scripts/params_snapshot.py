import json
import sys
import time

import rclpy

from backend.core.param_reader import read_all_parameters


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]

    if len(argv) < 2:
        print("Usage: python3 -m backend.scripts.params_snapshot <node_name> <out.json>", file=sys.stderr)
        sys.exit(1)

    node = argv[0]
    out_path = argv[1]

    rclpy.init(args=None)
    try:
        params = read_all_parameters(node)
        payload = {
            "node": node,
            "timestamp": time.time(),
            "parameters": params,
        }
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, sort_keys=True)
        print(f"Wrote snapshot: {out_path} ({len(params)} params)")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
