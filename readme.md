# ROS Supervisor 

**ROS Supervisor** is a diagnostic and observability backend for ROS 2 systems. It provides programmatic inspection of nodes, topics, QoS compatibility, TF trees, topic rates, parameters, and overall node health — all from Python.

This project helps **robotics engineers debug complex ROS 2 systems faster** by exposing runtime issues that are otherwise difficult to see.

---

## Features

- **ROS Graph Inspection**

  - Nodes, topics, publishers, subscribers
  - Live system snapshot (`dump_graph`)

- **QoS Analysis**

  - Endpoint QoS reporting
  - Automatic mismatch detection
  - Human‑readable fix suggestions

- **Topic Rate Measurement**

  - Estimate publish frequency over time windows
  - Detect stalled or slow publishers

- **TF Tree Validation**

  - Detect missing transforms
  - Identify multiple roots, disconnected trees

- **Parameter Introspection**

  - Snapshot node parameters
  - Diff parameter changes over time

- **Node Health Report**

  - Detect inactive / misconfigured nodes
  - Flag low‑rate publishers
  - Highlight meaningless pub/sub patterns

---

## Why This Exists

ROS 2 provides powerful CLI tools (`ros2 topic`, `ros2 node`, etc.), but they are **fragmented and terminal‑only**.

ROS Supervisor:

- Centralizes diagnostics
- Makes issues machine‑detectable
- Is designed to back future **dashboards, APIs, and autonomy supervisors**

---

## Architecture

```
backend/
├── core/
│   ├── ros_adapter.py      # ROS graph + runtime access
│   ├── graph_model.py      # System snapshot builder
│   ├── qos_inspector.py    # QoS reporting & validation
│   ├── tf_inspector.py     # TF tree analysis
│   └── health.py           # Node health logic
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
```

---

## Quick Start

```bash
source venv/bin/activate
source /opt/ros/humble/setup.bash

python3 -m backend.scripts.dump_graph
python3 -m backend.scripts.qos_validate
python3 -m backend.scripts.health_report
```

---

## Tested With

- ROS 2 Humble
- Ubuntu 22.04
- Python 3.10

---

## Roadmap

- REST API (FastAPI)
- Web dashboard (graph + TF + QoS)
- ROS bag offline analysis
- Multi‑robot / multi‑domain support

---

## Author

**Pratyush**\
Robotics · ROS 2 · Systems Diagnostics

---

## License

MIT (planned)

