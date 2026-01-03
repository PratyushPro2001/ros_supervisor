# ROS Supervisor

**ROS Supervisor** is a diagnostic and observability backend for ROS 2 systems.
It provides programmatic inspection of nodes, topics, QoS compatibility, TF trees, topic rates, parameters, and overall node health — entirely from Python.

This project helps **robotics engineers debug complex ROS 2 systems faster** by exposing runtime issues that are otherwise difficult to see, automate, or validate at scale.

---

## Why This Exists

In real ROS 2 systems, failures are rarely syntax errors — they are *runtime* problems:

* QoS mismatches that silently block communication
* Missing or disconnected TF frames
* Nodes that are alive but not doing meaningful work
* Topics publishing at incorrect or unstable rates
* Parameters changing at runtime without visibility

ROS Supervisor turns these **hard-to-see runtime issues into structured, machine‑detectable diagnostics**.

---

## Features

### ROS Graph Inspection

* Nodes, topics, publishers, subscribers
* Live system snapshot (`dump_graph`)
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
* Identify nodes that only publish `/rosout` or `/parameter_events`

---

## What This Is NOT

* ❌ Not a simulator
* ❌ Not a visualization tool like RViz
* ❌ Not a beginner tutorial project

This is an **engineering‑grade diagnostics backend**, designed to support:

* Dashboards
* CI checks
* Autonomy supervisors
* Fleet and system monitoring tools

---

## Architecture

```
backend/
├── core/
│   ├── ros_adapter.py      # ROS graph + runtime access
│   ├── graph_model.py      # System snapshot builder
│   ├── qos_inspector.py    # QoS reporting & validation
│   ├── tf_inspector.py     # TF tree analysis
│   └── node_health.py      # Node health logic
│
├── scripts/
│   ├── dump_graph.py
│   ├── inspect_node.py
│   ├── measure_rate.py
│   ├── qos_report.py
│   ├── qos_validate.py
│   ├── tf_report.py
│   ├── tf_validate.py
│   ├── params_snapshot.py
│   ├── params_diff.py
│   └── health_report.py
│
├── api/
│   ├── main.py             # FastAPI entrypoint
│   └── routes.py           # REST endpoints
│
└── cli.py                  # Unified CLI wrapper
```

---

## Quick Start

```bash
source venv/bin/activate
source /opt/ros/humble/setup.bash

# Script-based diagnostics
python3 -m backend.scripts.dump_graph
python3 -m backend.scripts.qos_validate
python3 -m backend.scripts.health_report

# Unified CLI
python3 -m backend.cli health
```

---

## REST API (Experimental)

```bash
uvicorn backend.api.main:app --reload
```

Example:

```bash
curl http://127.0.0.1:8000/health | python3 -m json.tool
curl http://127.0.0.1:8000/qos/validate | python3 -m json.tool
```

---

## Typical Debugging Workflow

1. Capture system snapshot (graph, topics, nodes)
2. Validate QoS compatibility
3. Inspect topic publish rates
4. Validate TF tree integrity
5. Snapshot and diff parameters
6. Generate node health report

This mirrors how **senior robotics engineers debug production ROS 2 systems**.

---

## Tested With

* ROS 2 Humble
* Ubuntu 22.04
* Python 3.10

---

## Project Status

* Backend diagnostics: **Stable**
* CLI interface: **Stable**
* REST API: **Working / evolving**
* Web UI: **Planned**

---

## Roadmap

* Web dashboard (graph + TF + QoS visualization)
* ROS bag offline analysis
* Multi‑robot / multi‑domain support
* CI‑friendly diagnostics mode

---

## Author

**Pratyush**
Robotics · ROS 2 · Systems Diagnostics

---

## License

MIT (planned)
