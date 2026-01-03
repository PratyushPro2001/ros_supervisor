from __future__ import annotations

import threading
import time
from typing import Any, Dict, Optional

from backend.core.ros_adapter import create_ros_adapter
from backend.core.graph_model import build_system_snapshot
from backend.core.node_health import build_node_health


class SnapshotDaemon:
    """
    Background poller that periodically samples ROS state and stores it in-memory.
    API reads are fast and do not touch rclpy.
    """

    def __init__(self, period_sec: float = 0.5):
        self.period_sec = float(period_sec)
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

        self._lock = threading.Lock()
        self._latest: Dict[str, Any] = {
            "ts": 0.0,
            "ok": False,
            "error": "not started",
            "graph": None,
            "health": None,
        }

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="snapshot-daemon", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def get_latest(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._latest)

    def _run(self):
        while not self._stop.is_set():
            t0 = time.time()
            try:
                adapter = create_ros_adapter()

                graph = build_system_snapshot(adapter)
                health = build_node_health(graph, adapter, float(3.0))

                payload = {
                    "ts": time.time(),
                    "ok": True,
                    "error": None,
                    "graph": graph,
                    "health": health,
                }
            except Exception as e:
                payload = {
                    "ts": time.time(),
                    "ok": False,
                    "error": f"{type(e).__name__}: {e}",
                    "graph": None,
                    "health": None,
                }

            with self._lock:
                self._latest = payload

            # maintain period
            elapsed = time.time() - t0
            sleep_for = max(0.05, self.period_sec - elapsed)
            self._stop.wait(sleep_for)


# single shared instance
daemon = SnapshotDaemon(period_sec=0.5)
