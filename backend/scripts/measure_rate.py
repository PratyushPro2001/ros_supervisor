import sys

from backend.utils.rate_calculator import estimate_topic_rate


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]

    if not argv:
        print(
            "Usage: python3 -m backend.scripts.measure_rate <topic_name> [duration_sec]",
            file=sys.stderr,
        )
        sys.exit(1)

    topic_name = argv[0]
    duration_sec = 5.0
    if len(argv) >= 2:
        try:
            duration_sec = float(argv[1])
        except ValueError:
            print("duration_sec must be a number (seconds)", file=sys.stderr)
            sys.exit(1)

    try:
        hz = estimate_topic_rate(topic_name, duration_sec=duration_sec)
        print(f"Estimated rate on '{topic_name}' ~ {hz:.2f} Hz (over {duration_sec:.1f}s)")
    except Exception as e:
        print(f"Error measuring rate on '{topic_name}': {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()

# ---------------------------
# Helper for other modules
# ---------------------------
def estimate_topic_rate_hz(adapter, topic_name: str, window_sec: float = 5.0):
    """
    Programmatic wrapper used by health_report.
    Returns float Hz, or None if measurement failed.
    """
    try:
        # Reuse the existing calculator in this file (whatever main() uses)
        calc = TopicRateCalculator(adapter, topic_name, window_sec=float(window_sec))
        return calc.run()
    except Exception:
        return None
