from __future__ import annotations

from typing import Any, Dict

from backend.core.ros_adapter import ROSAdapter


def get_params(adapter: ROSAdapter, node_name: str) -> Dict[str, Any]:
    """
    Return parameters for a node as a JSON-friendly dict.
    Uses ROSAdapter.get_node_parameters() (currently a safe stub in your adapter).
    """
    # Normalize: allow "talker" or "/talker"
    if not node_name.startswith("/"):
        node_name = f"/{node_name}"

    return adapter.get_node_parameters(node_name)
