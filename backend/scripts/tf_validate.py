import json
import sys

from backend.core.tf_inspector import inspect_tf
from backend.core.tf_validator import validate_tf


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]

    duration = 3.0
    if argv:
        try:
            duration = float(argv[0])
        except ValueError:
            print("Usage: python3 -m backend.scripts.tf_validate [duration_sec]", file=sys.stderr)
            sys.exit(1)

    report = inspect_tf(duration_sec=duration)
    result = validate_tf(report)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
