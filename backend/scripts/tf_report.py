import json
import sys

from backend.core.tf_inspector import inspect_tf


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]

    duration = 3.0
    if argv:
        try:
            duration = float(argv[0])
        except ValueError:
            print("Usage: python3 -m backend.scripts.tf_report [duration_sec]", file=sys.stderr)
            sys.exit(1)

    report = inspect_tf(duration_sec=duration)
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
