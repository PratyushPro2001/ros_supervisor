from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.node import Node


def estimate_topic_rate_hz(
    adapter,
    topic_name: str,
    window_sec: float = 3.0,
) -> Optional[float]:
    """
    Measure message rate on a topic.
    Assumes rclpy is already initialized.
    """
    node: Node = adapter._node  # reuse adapter node
    msg_times = []

    def cb(msg):
        msg_times.append(time.time())

    # Discover topic type
    topics = dict(node.get_topic_names_and_types())
    if topic_name not in topics:
        return None

    type_str = topics[topic_name][0]
    pkg, _, msg = type_str.partition("/msg/")
    mod = __import__(f"{pkg}.msg", fromlist=[msg])
    msg_type = getattr(mod, msg)

    sub = node.create_subscription(msg_type, topic_name, cb, 10)

    end = time.time() + window_sec
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_subscription(sub)

    if len(msg_times) < 2:
        return None

    duration = msg_times[-1] - msg_times[0]
    if duration <= 0:
        return None

    return (len(msg_times) - 1) / duration
