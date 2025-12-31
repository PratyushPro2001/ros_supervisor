from __future__ import annotations

from typing import Any, Dict, List, Set, Tuple


def validate_tf(tf_report: Dict[str, Any]) -> Dict[str, Any]:
    """
    Validate a TF snapshot produced by tf_inspector.inspect_tf().

    Returns:
    {
      "ok": bool,
      "errors": [...],
      "warnings": [...],
      "stats": {...}
    }
    """
    errors: List[str] = []
    warnings: List[str] = []

    frames: List[str] = tf_report.get("frames", []) or []
    edges_list = tf_report.get("edges", []) or []

    # Convert to edges
    edges: List[Tuple[str, str]] = [(e["parent"], e["child"]) for e in edges_list if "parent" in e and "child" in e]

    stats = {
        "num_frames": len(frames),
        "num_edges": len(edges),
        "num_roots": len(tf_report.get("roots", []) or []),
    }

    # 1) No TF at all
    if len(edges) == 0:
        warnings.append("No TF transforms observed on /tf or /tf_static during the sampling window.")

    # 2) Multiple parents (a TF child frame should generally have exactly one parent)
    parents_map: Dict[str, Set[str]] = {}
    for p, c in edges:
        parents_map.setdefault(c, set()).add(p)

    multi_parent = {child: sorted(list(parents)) for child, parents in parents_map.items() if len(parents) > 1}
    if multi_parent:
        errors.append(f"TF has child frames with multiple parents: {multi_parent}")

    # 3) Cycle detection (graph should be a tree/forest, cycles are bad)
    # Build adjacency
    adj: Dict[str, List[str]] = {}
    for p, c in edges:
        adj.setdefault(p, []).append(c)

    visited: Set[str] = set()
    stack: Set[str] = set()

    def dfs(u: str) -> bool:
        visited.add(u)
        stack.add(u)
        for v in adj.get(u, []):
            if v not in visited:
                if dfs(v):
                    return True
            elif v in stack:
                return True
        stack.remove(u)
        return False

    has_cycle = False
    for f in frames:
        if f not in visited:
            if dfs(f):
                has_cycle = True
                break

    if has_cycle:
        errors.append("TF graph contains a cycle (invalid TF).")

    ok = len(errors) == 0
    return {"ok": ok, "errors": errors, "warnings": warnings, "stats": stats}
