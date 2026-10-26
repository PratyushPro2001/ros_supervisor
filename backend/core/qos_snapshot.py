"""Live QoS check for the snapshot: every publisher x subscriber pair on every topic."""
from __future__ import annotations

from typing import Any, Dict, List

from backend.core.qos_inspector import _serialize_endpoint_info
from backend.core.qos_validator import find_qos_mismatches

OBSERVER_NODES = {"/ros_supervisor_adapter"}


def build_qos_section(adapter) -> Dict[str, Any]:
    node = adapter._node
    topics_out: List[Dict[str, Any]] = []
    pairs = 0
    for name, _types in node.get_topic_names_and_types():
        try:
            pubs = [_serialize_endpoint_info(i) for i in node.get_publishers_info_by_topic(name)]
            subs = [_serialize_endpoint_info(i) for i in node.get_subscriptions_info_by_topic(name)]
        except Exception:
            continue
        pubs = [p for p in pubs if p["node"] not in OBSERVER_NODES]
        subs = [s for s in subs if s["node"] not in OBSERVER_NODES]
        if not pubs or not subs:
            continue
        pairs += len(pubs) * len(subs)
        topics_out.append({"name": name, "publishers": pubs, "subscribers": subs})

    mismatches = find_qos_mismatches({"topics": topics_out})
    return {
        "ok": len(mismatches) == 0,
        "topics_checked": len(topics_out),
        "pairs_checked": pairs,
        "mismatches": mismatches,
    }
