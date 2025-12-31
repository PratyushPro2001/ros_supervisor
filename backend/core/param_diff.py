from __future__ import annotations
from typing import Any, Dict


def diff_params(before: Dict[str, Any], after: Dict[str, Any]) -> Dict[str, Any]:
    """
    Compute a diff between two parameter snapshots.

    Returns:
    {
      "added": {param: value},
      "removed": {param: value},
      "changed": {param: {"before": x, "after": y}}
    }
    """
    added = {}
    removed = {}
    changed = {}

    before_keys = set(before.keys())
    after_keys = set(after.keys())

    for k in after_keys - before_keys:
        added[k] = after[k]

    for k in before_keys - after_keys:
        removed[k] = before[k]

    for k in before_keys & after_keys:
        if before[k] != after[k]:
            changed[k] = {"before": before[k], "after": after[k]}

    return {
        "added": added,
        "removed": removed,
        "changed": changed,
    }
