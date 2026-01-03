from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from backend.api.routes import router
from backend.api.snapshot_daemon import daemon

app = FastAPI(title="ROS Supervisor API")

# Dev CORS (Vite @ 5173)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://127.0.0.1:5173", "http://localhost:5173"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(router)

@app.on_event("startup")
def _startup():
    daemon.start()

@app.on_event("shutdown")
def _shutdown():
    daemon.stop()
