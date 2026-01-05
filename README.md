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
│   ├── ros_adapter.py
│   ├── graph_model.py
│   ├── qos_inspector.py
│   ├── tf_inspector.py
│   └── node_health.py
│
├── api/
│   ├── main.py
│   ├── routes.py
│   └── snapshot_daemon.py
│
├── scripts/
├── cli.py
└── README.md

ui/
└── README.md
```

---

## Quick Start (Backend Only)

```bash
cd ~/ros_supervisor
source venv/bin/activate
source /opt/ros/humble/setup.bash

uvicorn backend.api.main:app --reload --host 127.0.0.1 --port 8000
```

---

## Quick Start (ROS 2 Launch — Recommended)

This starts **everything at once** using a single ROS launch command:

* Backend (FastAPI + snapshot daemon)
* Web UI (Vite + React)
* Browser auto‑opens to the dashboard

### Prerequisites

* Ubuntu 22.04
* ROS 2 Humble
* Python 3.10+
* Node.js + npm
* colcon

### One‑Time Setup

```bash
git clone https://github.com/PratyushPro2001/ros_supervisor.git
cd ros_supervisor

python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

cd ui
npm install
cd ..
```

### Optional (Recommended): Configure Repo Path

```bash
mkdir -p ~/.config/ros_supervisor

cat > ~/.config/ros_supervisor/paths.env <<'EOF'
ROS_SUPERVISOR_REPO=/absolute/path/to/ros_supervisor
EOF
```

If this file is missing, the launch system will automatically infer the repo location.

### Build + Launch

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
colcon build --merge-install
source install/setup.bash

ros2 launch ros_supervisor supervisor.launch.py
```

Dashboard:

```
http://127.0.0.1:5173
```

---

## REST API (MVP)

* `GET /snapshot`
* `GET /health`
* `GET /graph`

Example:

```bash
curl http://127.0.0.1:8000/snapshot | python3 -m json.tool
```

---

## Tested With

* ROS 2 Humble
* Ubuntu 22.04
* Python 3.10

---

## Project Status

* Backend diagnostics: **Stable (MVP)**
* Snapshot daemon: **Stable**
* REST API: **Stable**
* Web UI: **Minimal MVP**

---

## Author

**Pratyush**  
M.S. Robotics, University of Delaware  
Robotics · ROS 2 · Systems Diagnostics

