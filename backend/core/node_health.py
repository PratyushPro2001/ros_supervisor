from __future__ import annotations

from typing import Any, Dict, List

from backend.core.topic_activity import measure_topic_rate, is_rate_healthy

IGNORE_TOPICS = {"/rosout", "/parameter_events"}


def build_node_health(snapshot: Dict[str, Any], adapter, rate_window_sec: float) -> Dict[str, Any]:
    nodes: List[str] = snapshot.get("nodes", []) or []
    topics: List[Dict[str, Any]] = snapshot.get("topics", []) or []

    out_nodes: List[Dict[str, Any]] = []
    topic_rates: List[Dict[str, Any]] = []

    # Measure rates only for meaningful topics that have at least one publisher
    for t in topics:
        name = t.get("name")
        pubs = t.get("publishers") or []
        if not name or name in IGNORE_TOPICS:
            continue
        if len(pubs) == 0:
            continue
        try:
            topic_rates.append(measure_topic_rate(adapter, name, window_sec=rate_window_sec))
        except Exception as e:
            topic_rates.append({"topic": name, "window_sec": rate_window_sec, "hz": None, "error": str(e)})

    rate_map = {tr["topic"]: tr for tr in topic_rates}

    for n in sorted(nodes):
        pubs = []
        subs = []

        for t in topics:
            tname = t.get("name")
            if not tname:
                continue
            if n in (t.get("publishers") or []):
                pubs.append(tname)
            if n in (t.get("subscribers") or []):
                subs.append(tname)

        warnings: List[str] = []

        meaningful_pubs = [x for x in pubs if x not in IGNORE_TOPICS]
        meaningful_subs = [x for x in subs if x not in IGNORE_TOPICS]

        if not meaningful_pubs and not meaningful_subs:
            warnings.append("Node has no meaningful pub/sub activity (only rosout/parameter_events or nothing).")

        # If node publishes a meaningful topic, ensure the topic is actually active
        for tname in meaningful_pubs:
            tr = rate_map.get(tname)
            if tr is None:
                continue
            hz = tr.get("hz")
            if not is_rate_healthy(hz):
                warnings.append(f"Publishes {tname} but rate is low/unknown (hz={hz}).")

        out_nodes.append(
            {
                "name": n,
                "publishes": sorted(pubs),
                "subscribes": sorted(subs),
                "warnings": warnings,
            }
        )

    return {"nodes": out_nodes, "topic_rates": topic_rates}
