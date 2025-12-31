from __future__ import annotations
from typing import Any, Dict, List


def _is_reliability_compatible(pub_rel: str, sub_rel: str) -> bool:
    if sub_rel == "RELIABLE":
        return pub_rel == "RELIABLE"
    if sub_rel == "BEST_EFFORT":
        return pub_rel in ("BEST_EFFORT", "RELIABLE")
    return True


def _is_durability_compatible(pub_dur: str, sub_dur: str) -> bool:
    if sub_dur == "TRANSIENT_LOCAL":
        return pub_dur == "TRANSIENT_LOCAL"
    if sub_dur == "VOLATILE":
        return pub_dur in ("VOLATILE", "TRANSIENT_LOCAL")
    return True


def _suggest_fixes(pub_qos: Dict[str, Any], sub_qos: Dict[str, Any]) -> List[str]:
    """
    Human-friendly suggestions for resolving QoS mismatches.
    We keep it simple and practical for MVP.
    """
    suggestions: List[str] = []

    pub_rel = pub_qos.get("reliability", "UNKNOWN")
    sub_rel = sub_qos.get("reliability", "UNKNOWN")
    pub_dur = pub_qos.get("durability", "UNKNOWN")
    sub_dur = sub_qos.get("durability", "UNKNOWN")

    # Reliability suggestions
    if not _is_reliability_compatible(pub_rel, sub_rel):
        if sub_rel == "RELIABLE" and pub_rel == "BEST_EFFORT":
            suggestions.append("Set publisher reliability to RELIABLE (or set subscriber to BEST_EFFORT).")

    # Durability suggestions
    if not _is_durability_compatible(pub_dur, sub_dur):
        if sub_dur == "TRANSIENT_LOCAL" and pub_dur == "VOLATILE":
            suggestions.append("Set publisher durability to TRANSIENT_LOCAL (or set subscriber to VOLATILE).")

    # If we couldn't derive anything specific
    if not suggestions:
        suggestions.append("Make publisher and subscriber QoS settings compatible (reliability/durability).")

    return suggestions


def find_qos_mismatches(qos_report: Dict[str, Any]) -> List[Dict[str, Any]]:
    """
    Detect QoS mismatches by comparing *requested* vs *offered* QoS.

    Output records include 'issues' and 'suggestions':
    {
      "topic": "...",
      "publisher": "...",
      "subscriber": "...",
      "issues": [...],
      "suggestions": [...]
    }
    """
    mismatches: List[Dict[str, Any]] = []

    for t in qos_report.get("topics", []):
        topic = t.get("name")
        pubs = t.get("publishers", [])
        subs = t.get("subscribers", [])

        for pub in pubs:
            pub_node = pub.get("node")
            pub_qos = pub.get("qos", {})
            pub_rel = pub_qos.get("reliability", "UNKNOWN")
            pub_dur = pub_qos.get("durability", "UNKNOWN")

            for sub in subs:
                sub_node = sub.get("node")
                sub_qos = sub.get("qos", {})
                sub_rel = sub_qos.get("reliability", "UNKNOWN")
                sub_dur = sub_qos.get("durability", "UNKNOWN")

                issues: List[str] = []

                if not _is_reliability_compatible(pub_rel, sub_rel):
                    issues.append(f"reliability mismatch: pub={pub_rel} sub={sub_rel}")

                if not _is_durability_compatible(pub_dur, sub_dur):
                    issues.append(f"durability mismatch: pub={pub_dur} sub={sub_dur}")

                if issues:
                    mismatches.append(
                        {
                            "topic": topic,
                            "publisher": pub_node,
                            "subscriber": sub_node,
                            "issues": issues,
                            "suggestions": _suggest_fixes(pub_qos, sub_qos),
                        }
                    )

    return mismatches
