"""Persistent topic-rate monitor.

One long-lived subscription per topic, timestamps kept in a rolling deque.
The rate is computed on demand over a sliding window, so every topic is
measured in parallel and all the time (the old code subscribed to each topic
one after another for `window` seconds, which made one snapshot take
~window x n_topics seconds and froze the dashboard).

Subscriptions use BEST_EFFORT + VOLATILE, which matches both BEST_EFFORT and
RELIABLE publishers, so sensor-data topics are not silently missed.
"""
from __future__ import annotations

import time
from collections import deque
from typing import Any, Dict

from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.utilities import get_message

_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
)


class RateMonitor:
    def __init__(self, node):
        self._node = node
        self._subs: Dict[str, Any] = {}
        self._stamps: Dict[str, deque] = {}
        self._created: Dict[str, float] = {}
        self._errors: Dict[str, str] = {}

    def ensure(self, topic: str, type_str: str | None) -> None:
        if topic in self._subs or topic in self._errors or not type_str:
            return
        try:
            msg_type = get_message(type_str)
            dq: deque = deque(maxlen=3000)

            def cb(_msg, dq=dq):
                dq.append(time.monotonic())

            self._stamps[topic] = dq
            self._created[topic] = time.monotonic()
            self._subs[topic] = self._node.create_subscription(msg_type, topic, cb, _QOS)
        except Exception as e:  # unknown type, etc.
            self._errors[topic] = f"{type(e).__name__}: {e}"

    def rate(self, topic: str, window_sec: float) -> Dict[str, Any]:
        out: Dict[str, Any] = {"topic": topic, "window_sec": window_sec, "hz": None, "age_sec": None}
        if topic in self._errors:
            out["error"] = self._errors[topic]
            return out
        dq = self._stamps.get(topic)
        if dq is None:
            return out
        now = time.monotonic()
        stamps = [t for t in tuple(dq) if now - t <= window_sec]
        if dq:
            out["age_sec"] = round(now - dq[-1], 2)
        if len(stamps) >= 2:
            out["hz"] = round((len(stamps) - 1) / max(stamps[-1] - stamps[0], 1e-6), 2)
        elif now - self._created[topic] >= window_sec:
            out["hz"] = 0.0  # subscribed long enough, nothing (or one msg) arrived: stalled
        # else: just subscribed, no data yet -> hz stays None
        return out


def get_rate_monitor(adapter) -> RateMonitor:
    mon = getattr(adapter, "_rate_monitor", None)
    if mon is None:
        mon = RateMonitor(adapter._node)
        adapter._rate_monitor = mon
    return mon
