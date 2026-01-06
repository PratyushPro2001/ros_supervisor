from __future__ import annotations

import threading
import time
from typing import Any, Dict

from backend.core.ros_adapter import create_ros_adapter
from backend.core.graph_model import build_graph
from backend.core.node_health import build_node_health


class SnapshotDaemon:
    def __init__(self, period_sec: float = 0.5):
        self.period_sec = float(period_sec)
        self._lock = threading.Lock()
        self._latest: Dict[str, Any] = {
            "ts": 0.0,
            "ok": False,
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
                health = build_node_health(graph, adapter, rate_window_sec=3.0)

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

            print(
                "[snapshot-daemon] ok=%s ts=%s err=%s"
                % (payload["ok"], payload["ts"], payload["error"])
            )

            elapsed = time.time() - t0
            self._stop.wait(max(0.05, self.period_sec - elapsed))


daemon = SnapshotDaemon(period_sec=0.5)
