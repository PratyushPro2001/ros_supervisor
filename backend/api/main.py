from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from backend.core.ros_adapter import create_ros_adapter, ROSAdapter
from backend.core.graph_model import build_system_snapshot


app = FastAPI(
    title="ROS Supervisor Backend",
    description="MVP backend for ROS Supervisor Runtime",
    version="0.1.0",
)

# Allow local frontend dev later
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # TODO: tighten in production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Single shared adapter instance for this process
adapter: ROSAdapter = create_ros_adapter()


@app.get("/health")
def health_check():
    return {"status": "ok"}


@app.get("/graph")
def get_graph():
    """
    Return the current ROS system snapshot as JSON.
    """
    snapshot = build_system_snapshot(adapter)
    return snapshot
