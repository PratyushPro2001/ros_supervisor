from __future__ import annotations

import time
from typing import Any, Dict, List, Tuple

from backend.core.node_health import build_node_health

SYSTEM_TOPICS = {"/rosout", "/parameter_events"}


def _sorted_unique(xs: List[str]) -> List[str]:
    return sorted(set(x for x in xs if x))


def build_graph(adapter, include_system_edges: bool = False) -> Dict[str, Any]:
    """
    Enriched ROS graph snapshot:

    {
      "nodes": [{"name": "..."}],
      "topics": [{"name": "...", "types": [...], "publishers": [...], "subscribers": [...]}],
      "edges": [{"from": "...", "topic": "...", "to": "..."}]
    }
    """
    # Nodes
    try:
        node_names = adapter.get_nodes()
    except Exception:
        node_names = []
    node_names = _sorted_unique(node_names)
    nodes = [{"name": n} for n in node_names]

    # Topics + types
    try:
        topic_tuples: List[Tuple[str, List[str]]] = adapter.get_topic_names_and_types()
    except Exception:
        topic_tuples = []

    topics_out: List[Dict[str, Any]] = []
    edges: List[Dict[str, str]] = []

    for tname, ttypes in sorted(topic_tuples, key=lambda x: x[0]):
        # Enrich with pubs/subs
        try:
            pubs = adapter.get_publishers_by_topic(tname)
        except Exception:
            pubs = []
        try:
            subs = adapter.get_subscribers_by_topic(tname)
        except Exception:
            subs = []

        pubs = _sorted_unique(pubs)
        subs = _sorted_unique(subs)

        topics_out.append(
            {
                "name": tname,
                "types": list(ttypes) if ttypes else [],
                "publishers": pubs,
                "subscribers": subs,
            }
        )

        # Skip noisy system edges unless explicitly requested
        if (not include_system_edges) and (tname in SYSTEM_TOPICS):
            continue

        # Edges = publisher -> subscriber (topic stored on edge)
        for p in pubs:
            for s in subs:
                edges.append({"from": p, "topic": tname, "to": s})

    edges.sort(key=lambda e: (e["topic"], e["from"], e["to"]))

    return {"nodes": nodes, "topics": topics_out, "edges": edges}


def build_system_snapshot(adapter, rate_window_sec: float = 3.0) -> Dict[str, Any]:
    """
    This is what the FastAPI routes expect.

    Returns:
    {
      "ts": float,
      "ok": bool,
      "error": str|None,
      "graph": {...},
      "health": {...}
    }
    """
    ts = time.time()
    try:
        graph = build_graph(adapter, include_system_edges=False)
        health = build_node_health(graph, adapter, rate_window_sec=rate_window_sec)
        return {"ts": ts, "ok": True, "error": None, "graph": graph, "health": health}
    except Exception as e:
        return {"ts": ts, "ok": False, "error": f"{type(e).__name__}: {e}", "graph": None, "health": None}


# Back-compat aliases (in case older code uses these)
def build_graph_snapshot(adapter) -> Dict[str, Any]:
    return build_graph(adapter)
