from __future__ import annotations

from typing import Any, Dict, List

import os

from backend.core.topic_activity import is_rate_healthy
from backend.core.rate_monitor import get_rate_monitor

IGNORE_TOPICS = {"/rosout", "/parameter_events"}
OBSERVER_NODES = {"ros_supervisor_adapter"}
# Event-driven topics (only publish on user action): never flagged as stalled.
# Extend with ROS_SUPERVISOR_EVENT_TOPICS=/a,/b
EVENT_TOPICS = {"/clicked_point", "/initialpose", "/move_base_simple/goal"} | {
    t.strip() for t in os.environ.get("ROS_SUPERVISOR_EVENT_TOPICS", "").split(",") if t.strip()
}


def build_node_health(snapshot: Dict[str, Any], adapter, rate_window_sec: float) -> Dict[str, Any]:
    # Normalize nodes into a list of node-name strings
    nodes_raw = snapshot.get("nodes", []) or []
    nodes: List[str] = [
        x["name"] if isinstance(x, dict) else x
        for x in nodes_raw
        if x
    ]

    topics: List[Dict[str, Any]] = snapshot.get("topics", []) or []

    out_nodes: List[Dict[str, Any]] = []
    topic_rates: List[Dict[str, Any]] = []

    # Live rates from persistent subscriptions (all topics in parallel, never blocks).
    monitor = get_rate_monitor(adapter)
    type_map = {n: (ts[0] if ts else None) for n, ts in adapter.get_topic_names_and_types()}

    for t in topics:
        name = t.get("name")
        if not name or name in IGNORE_TOPICS:
            continue
        try:
            pubs = adapter.get_publishers_by_topic(name)
        except Exception:
            pubs = []
        if len(pubs) == 0:
            continue
        monitor.ensure(name, type_map.get(name))
        topic_rates.append(monitor.rate(name, rate_window_sec))

    rate_map = {tr["topic"]: tr for tr in topic_rates}

    for n in sorted(nodes):
        # Compute per-node pub/sub directly from ROS via adapter
        try:
            pubs: List[str] = adapter.get_publishers(n)
        except Exception:
            pubs = []
        try:
            subs: List[str] = adapter.get_subscribers(n)
        except Exception:
            subs = []

        warnings: List[str] = []

        meaningful_pubs = [x for x in pubs if x not in IGNORE_TOPICS]
        meaningful_subs = [x for x in subs if x not in IGNORE_TOPICS]

        if not meaningful_pubs and not meaningful_subs:
            if n in OBSERVER_NODES:
                warnings.append(
                    "INFO: observer node (no meaningful pub/sub expected)."
                )
            else:
                warnings.append(
                    "Node has no meaningful pub/sub activity (only rosout/parameter_events or nothing)."
                )

        for tname in meaningful_pubs:
            tr = rate_map.get(tname)
            if tr is None:
                continue
            if tname in EVENT_TOPICS:
                continue
            hz = tr.get("hz")
            if hz is None:
                warnings.append(f"{tname}: no data yet.")
            elif not is_rate_healthy(hz):
                warnings.append(
                    f"{tname}: STALLED, no messages in the last {rate_window_sec:.0f}s."
                )

        out_nodes.append(
            {
                "name": n,
                "publishes": sorted(pubs),
                "subscribes": sorted(subs),
                "warnings": warnings,
            }
        )

    return {"nodes": out_nodes, "topic_rates": topic_rates}


def summarize_status(graph: Dict[str, Any], health: Dict[str, Any], qos: Dict[str, Any] | None = None, tf: Dict[str, Any] | None = None):
    """Return (status, issues): status is GOOD / WARN / ERROR.

    ERROR: a required node is missing (ROS_SUPERVISOR_REQUIRED_NODES=a,b,c).
    WARN : a publishing topic is stalled / has no data yet.
    """
    issues: List[str] = []
    status = "GOOD"

    required = [x.strip() for x in os.environ.get("ROS_SUPERVISOR_REQUIRED_NODES", "").split(",") if x.strip()]
    present = {
        (x["name"] if isinstance(x, dict) else x) for x in (graph.get("nodes", []) or []) if x
    }
    for r in required:
        if r not in present:
            issues.append(f"Node {r} is MISSING.")
            status = "ERROR"

    for tr in health.get("topic_rates", []):
        name = tr["topic"]
        if name in EVENT_TOPICS or name in IGNORE_TOPICS:
            continue
        hz = tr.get("hz")
        if hz is None:
            issues.append(f"{name}: no data yet.")
        elif not is_rate_healthy(hz):
            issues.append(f"{name}: STALLED (no messages in the last {tr.get('window_sec', 0):.0f}s).")
        if status == "GOOD" and issues:
            status = "WARN"

    for m in (qos or {}).get("mismatches", []):
        issues.append(f"QoS {m['topic']}: {m['publisher']} -> {m['subscriber']}: {'; '.join(m['issues'])}")
        if status == "GOOD":
            status = "WARN"

    if tf:
        for e in tf.get("validation", {}).get("errors", []):
            issues.append(f"TF: {e}")
            status = "ERROR"
        for e in tf.get("stale", []):
            issues.append(f"TF {e}: STALLED.")
            if status == "GOOD":
                status = "WARN"
    return status, issues
