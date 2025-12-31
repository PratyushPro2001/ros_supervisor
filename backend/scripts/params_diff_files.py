import json
import sys

from backend.core.param_diff import diff_params


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]

    if len(argv) != 2:
        print("Usage: python3 -m backend.scripts.params_diff_files <before.json> <after.json>", file=sys.stderr)
        sys.exit(1)

    before_path, after_path = argv

    with open(before_path, "r", encoding="utf-8") as f:
        before = json.load(f).get("parameters", {})

    with open(after_path, "r", encoding="utf-8") as f:
        after = json.load(f).get("parameters", {})

    print(json.dumps(diff_params(before, after), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
