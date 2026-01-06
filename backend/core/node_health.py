from __future__ import annotations

from typing import Any, Dict, List

from backend.core.topic_activity import measure_topic_rate, is_rate_healthy

IGNORE_TOPICS = {"/rosout", "/parameter_events"}
OBSERVER_NODES = {"ros_supervisor_adapter"}


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

    # Measure rates only for meaningful topics that have at least one publisher
    for t in topics:
        name = t.get("name")
        if not name or name in IGNORE_TOPICS:
            continue

        # Use live ROS lookup for publishers (snapshot topic entries do not include publishers/subscribers)
        try:
            pubs = adapter.get_publishers_by_topic(name)
        except Exception:
            pubs = []

        if len(pubs) == 0:
            continue

        try:
            topic_rates.append(
                measure_topic_rate(adapter, name, window_sec=rate_window_sec)
            )
        except Exception as e:
            topic_rates.append(
                {
                    "topic": name,
                    "window_sec": rate_window_sec,
                    "hz": None,
                    "error": str(e),
                }
            )

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
            hz = tr.get("hz")
            if not is_rate_healthy(hz):
                warnings.append(
                    f"Publishes {tname} but rate is low/unknown (hz={hz})."
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
