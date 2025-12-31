from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message


class _RateCalcNode(Node):
    def __init__(self, topic_name: str, duration_sec: float, wait_for_topic_sec: float):
        super().__init__("topic_rate_calculator")
        self.topic_name = topic_name
        self.duration_sec = duration_sec
        self.wait_for_topic_sec = wait_for_topic_sec
        self._count = 0
        self._start_time: Optional[float] = None

        topic_type = self._wait_and_resolve_topic_type(topic_name, wait_for_topic_sec)
        if topic_type is None:
            raise RuntimeError(f"Topic '{topic_name}' not found in ROS graph (after {wait_for_topic_sec:.1f}s)")

        msg_type = get_message(topic_type)
        self.get_logger().info(
            f"Subscribing to {topic_name} [{topic_type}] for {duration_sec:.1f}s"
        )

        self._subscription = self.create_subscription(
            msg_type,
            topic_name,
            self._callback,
            10,
        )

    def _wait_and_resolve_topic_type(self, topic_name: str, wait_sec: float) -> Optional[str]:
        deadline = time.time() + max(0.0, wait_sec)
        while rclpy.ok() and time.time() <= deadline:
            t = self._resolve_topic_type_once(topic_name)
            if t is not None:
                return t
            # Let DDS discovery progress
            rclpy.spin_once(self, timeout_sec=0.1)
        return None

    def _resolve_topic_type_once(self, topic_name: str) -> Optional[str]:
        for name, types in self.get_topic_names_and_types():
            if name == topic_name and types:
                return types[0]
        return None

    def _callback(self, msg):
        now = time.time()
        if self._start_time is None:
            self._start_time = now
        self._count += 1

    def measure(self) -> float:
        end_time = time.time() + self.duration_sec
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self._count <= 1 or self._start_time is None:
            return 0.0

        elapsed = time.time() - self._start_time
        if elapsed <= 0.0:
            return 0.0

        return float(self._count) / elapsed


def estimate_topic_rate(topic_name: str, duration_sec: float = 5.0, wait_for_topic_sec: float = 2.0) -> float:
    """
    Initialize rclpy, wait briefly for topic discovery, subscribe to the topic,
    and return an estimated message rate in Hz.
    """
    rclpy.init(args=None)
    try:
        node = _RateCalcNode(topic_name, duration_sec, wait_for_topic_sec)
        hz = node.measure()
        node.destroy_node()
        rclpy.shutdown()
        return hz
    except Exception:
        rclpy.shutdown()
        raise
