# ROS Supervisor

**ROS Supervisor** is an engineering‑grade **observability and diagnostics layer for ROS 2 systems**.

It continuously inspects the ROS graph, runtime behavior, and system health, and exposes that information in **machine‑readable form** (CLI, REST API, and dashboard‑ready snapshots).

This project is built for **robotics engineers debugging real, complex systems**, not tutorials or demos.

---

## Why This Exists

In real ROS 2 deployments, failures are rarely syntax errors — they are *runtime* failures:

* QoS mismatches that silently block communication
* Missing, stale, or disconnected TF frames
* Nodes that are technically alive but not doing meaningful work
* Topics publishing at incorrect, unstable, or stalled rates
* Parameters changing at runtime without visibility

ROS Supervisor turns these **hard‑to‑see runtime issues into structured, machine‑detectable diagnostics**.

---

## MVP Architecture (Current)

ROS Supervisor uses a **Snapshot Daemon architecture** to remain fast and stable under load:

* A background daemon continuously monitors ROS (graph + health)
* The daemon refreshes a cached system snapshot on a fixed interval
* HTTP endpoints and the UI **never talk to ROS directly**
* Clients read the latest snapshot in O(1) time

This avoids `rclpy` concurrency issues and enables near real‑time dashboards without stressing ROS.

---

## Features (Implemented / MVP)

### ROS Graph Inspection

* Nodes, topics, publishers, subscribers
* Live system snapshot
* Programmatic alternative to fragmented `ros2` CLI commands

### QoS Analysis

* Endpoint QoS reporting
* Automatic publisher ↔ subscriber mismatch detection
* Clear, human‑readable fix hints

### Topic Rate Measurement

* Estimate publish frequency over configurable time windows
* Detect stalled, slow, or unstable publishers

### TF Tree Validation

* Detect missing transforms
* Identify multiple roots and disconnected trees
* Validate TF topology correctness

### Parameter Introspection

* Snapshot node parameters
* Diff parameter changes over time
* Detect unintended runtime reconfiguration

### Node Health Report

* Detect inactive or misconfigured nodes
* Flag low‑rate publishers
* Identify observer‑only nodes (e.g. `/rosout`, `/parameter_events` only)

---

## What This Is NOT

* ❌ Not a simulator
* ❌ Not RViz or a visualization‑heavy tool
* ❌ Not a beginner tutorial project

ROS Supervisor is designed to **power** dashboards, CI checks, autonomy supervisors, and fleet monitoring systems — not replace visualization tools.

---

## Repository Layout

```
backend/
├── core/
│   ├── ros_adapter.py      # ROS graph + runtime access
│   ├── graph_model.py      # System snapshot builder
│   ├── qos_inspector.py    # QoS reporting & validation
│   ├── tf_inspector.py     # TF tree analysis
│   └── node_health.py      # Node health logic
│
├── api/
│   ├── main.py             # FastAPI entrypoint
│   ├── routes.py           # REST endpoints
│   └── snapshot_daemon.py  # Continuous snapshot worker
│
├── scripts/                # Standalone diagnostics scripts
├── cli.py                  # Unified CLI wrapper
└── README.md               # Project documentation

ui/
└── README.md               # Frontend‑only documentation
```

---

## Quick Start (Backend)

```bash
cd ~/ros_supervisor
source venv/bin/activate
source /opt/ros/humble/setup.bash

# Start backend with snapshot daemon
uvicorn backend.api.main:app --reload --host 127.0.0.1 --port 8000
```

---

## REST API (MVP)

* `GET /snapshot`
  Returns the latest cached system snapshot (recommended for dashboards)

* `GET /health`
  One‑shot health computation (serialized)

* `GET /graph`
  One‑shot ROS graph snapshot

Example:

```bash
curl http://127.0.0.1:8000/snapshot | python3 -m json.tool
```

---

## Typical Debugging Workflow

1. Observe live snapshot (`/snapshot`)
2. Inspect ROS graph consistency
3. Validate QoS compatibility
4. Check topic publish rates
5. Validate TF tree integrity
6. Inspect parameters and node health

This mirrors how **senior robotics engineers debug production ROS 2 systems**.

---

## Tested With

* ROS 2 Humble
* Ubuntu 22.04
* Python 3.10

---

## Project Status

* Backend diagnostics: **Stable (MVP)**
* Snapshot daemon: **Stable**
* REST API: **Stable for dashboards**
* Web UI: **Minimal MVP (real‑time, fast)**

---

## Roadmap (Post‑MVP)

* Persistent health monitoring with event history
* Lifecycle node awareness
* TF quality metrics (age, frequency, staleness)
* Parameter risk analysis
* Temporal diagnostics (event‑based warnings)
* Offline rosbag analysis
* Cross‑node contract / pipeline validation
* Extensible diagnostics framework

---

## One‑Line Pitch

**ROS Supervisor is a stack‑agnostic observability layer for ROS 2 that detects structural, temporal, and configuration‑level failures before they become runtime bugs.**

---

## Author

**Pratyush**
Robotics · ROS 2 · Systems Diagnostics

---

## License

MIT (planned)
