import argparse
import sys


def _run(modpath: str, argv):
    mod = __import__(modpath, fromlist=["main"])
    if not hasattr(mod, "main"):
        raise SystemExit(f"{modpath} has no main()")
    try:
        return mod.main(argv)
    except TypeError:
        # Many of our scripts define main() with no argv param.
        return mod.main()
    except TypeError:
        # Many of our scripts define main() with no argv param.
        return mod.main()


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]

    parser = argparse.ArgumentParser(
        prog="ros-supervisor",
        description="ROS Supervisor CLI"
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("graph", help="Dump ROS graph snapshot")
    sub.add_parser("health", help="Node health report")
    sub.add_parser("qos-report", help="QoS report")
    sub.add_parser("qos-validate", help="QoS mismatch validation")
    sub.add_parser("tf-report", help="TF tree report")
    sub.add_parser("tf-validate", help="TF tree validation")

    ps = sub.add_parser("params-show", help="Show node parameters")
    ps.add_argument("node")

    pss = sub.add_parser("params-snapshot", help="Snapshot node parameters")
    pss.add_argument("node")
    pss.add_argument("out")

    pd = sub.add_parser("params-diff", help="Diff node params over time")
    pd.add_argument("node")
    pd.add_argument("window_sec", type=float)

    args, extra = parser.parse_known_args(argv)

    if args.cmd == "graph":
        return _run("backend.scripts.dump_graph", extra)
    if args.cmd == "health":
        return _run("backend.scripts.health_report", extra)
    if args.cmd == "qos-report":
        return _run("backend.scripts.qos_report", extra)
    if args.cmd == "qos-validate":
        return _run("backend.scripts.qos_validate", extra)
    if args.cmd == "tf-report":
        return _run("backend.scripts.tf_report", extra)
    if args.cmd == "tf-validate":
        return _run("backend.scripts.tf_validate", extra)
    if args.cmd == "params-show":
        return _run("backend.scripts.params_show", [args.node] + extra)
    if args.cmd == "params-snapshot":
        return _run("backend.scripts.params_snapshot", [args.node, args.out] + extra)
    if args.cmd == "params-diff":
        return _run("backend.scripts.params_diff", [args.node, str(args.window_sec)] + extra)

    raise SystemExit("Unknown command")


if __name__ == "__main__":
    main()
