from __future__ import annotations

import threading
import time
from typing import Any, Dict

from backend.core.ros_adapter import create_ros_adapter
from backend.core.graph_model import build_system_snapshot
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
        adapter = create_ros_adapter()  # ONE adapter, reused forever
        while not self._stop.is_set():
            t0 = time.time()
            try:
                graph = build_system_snapshot(adapter)
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

            # once per cycle log (not per request)
            print(
                "[snapshot-daemon] ok=%s ts=%s err=%s"
                % (payload.get("ok"), payload.get("ts"), payload.get("error"))
            )

            elapsed = time.time() - t0
            sleep_for = max(0.05, self.period_sec - elapsed)
            self._stop.wait(sleep_for)


daemon = SnapshotDaemon(period_sec=0.5)
