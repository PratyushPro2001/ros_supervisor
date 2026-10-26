from __future__ import annotations

import threading
import time
from typing import Any, Dict

from backend.core.ros_adapter import create_ros_adapter
from backend.core.graph_model import build_graph
import rclpy

from backend.core.node_health import build_node_health, summarize_status
from backend.core.qos_snapshot import build_qos_section
from backend.core.tf_monitor import get_tf_monitor


class SnapshotDaemon:
    def __init__(self, period_sec: float = 0.5):
        self.period_sec = float(period_sec)
        self._lock = threading.Lock()
        self._latest: Dict[str, Any] = {
            "ts": 0.0,
            "ok": False,
            "status": "ERROR",
            "issues": [],
            "error": "not started",
            "graph": None,
            "health": None,
        }
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def get_latest(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._latest)

    def _run(self):
        adapter = create_ros_adapter()  # singleton adapter
        while not self._stop.is_set():
            t0 = time.time()
            try:
                graph = build_graph(adapter, include_system_edges=False)
                health = build_node_health(graph, adapter, rate_window_sec=5.0)
                qos = build_qos_section(adapter)
                tf = get_tf_monitor(adapter).report(5.0)
                status, issues = summarize_status(graph, health, qos, tf)

                payload = {
                    "ts": time.time(),
                    "ok": True,
                    "status": status,
                    "issues": issues,
                    "error": None,
                    "graph": graph,
                    "health": health,
                    "qos": qos,
                    "tf": tf,
                }
            except Exception as e:
                payload = {
                    "ts": time.time(),
                    "ok": False,
                    "status": "ERROR",
                    "issues": [],
                    "error": f"{type(e).__name__}: {e}",
                    "graph": None,
                    "health": None,
                    "qos": None,
                    "tf": None,
                }

            with self._lock:
                self._latest = payload

            key = (payload["status"], tuple(payload["issues"]), payload["error"])
            if key != getattr(self, "_last_key", None):
                self._last_key = key
                print("[snapshot-daemon] status=%s issues=%s err=%s"
                      % (payload["status"], payload["issues"], payload["error"]), flush=True)

            # Spin the adapter node for the rest of the period so the rate
            # monitor's subscription callbacks run (and are timestamped) promptly.
            end = t0 + self.period_sec
            while not self._stop.is_set() and time.time() < end:
                try:
                    rclpy.spin_once(adapter._node, timeout_sec=0.02)
                except Exception:
                    time.sleep(0.02)


daemon = SnapshotDaemon(period_sec=0.5)
