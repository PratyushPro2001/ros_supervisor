from __future__ import annotations

from typing import Any, Dict, Optional

from backend.core.topic_rate_core import estimate_topic_rate_hz


def measure_topic_rate(adapter, topic_name: str, window_sec: float = 3.0) -> Dict[str, Any]:
    hz = estimate_topic_rate_hz(adapter, topic_name, window_sec=window_sec)
    return {"topic": topic_name, "window_sec": window_sec, "hz": hz}


def is_rate_healthy(hz: Optional[float], min_hz: float = 0.2) -> bool:
    if hz is None:
        return False
    return hz >= min_hz
