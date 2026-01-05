from pathlib import Path
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction


def _load_env_file(env_path: Path) -> None:
    """
    Minimal .env loader for lines like:
      KEY=value
    Ignores empty lines and comments starting with '#'.
    Does NOT override existing environment variables.
    """
    if not env_path.exists():
        return

    for raw in env_path.read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            continue
        key, val = line.split("=", 1)
        key = key.strip()
        val = val.strip().strip('"').strip("'")
        if key and key not in os.environ:
            os.environ[key] = val


def _resolve_repo_root() -> Path:
    # 1) Load optional user config: ~/.config/ros_supervisor/paths.env
    cfg = Path("~/.config/ros_supervisor/paths.env").expanduser()
    _load_env_file(cfg)

    # 2) If ROS_SUPERVISOR_REPO is set (from config or shell), use it
    override = os.environ.get("ROS_SUPERVISOR_REPO", "").strip()
    if override:
        p = Path(override).expanduser().resolve()
        if p.exists():
            return p

    # 3) Fallback: derive repo root relative to this launch file.
    # Path: <repo>/ros_ws/src/ros_supervisor/launch/supervisor.launch.py
    # parents[4] => <repo>
    return Path(__file__).resolve().parents[4]


def generate_launch_description():
    repo_root = _resolve_repo_root()

    backend_cmd = (
        f'cd "{repo_root}" && '
        'source /opt/ros/humble/setup.bash && '
        'source venv/bin/activate && '
        'python -m uvicorn backend.api.main:app --host 127.0.0.1 --port 8000'
    )

    ui_cmd = (
        f'cd "{repo_root / "ui"}" && '
        'npm run dev -- --host 127.0.0.1 --port 5173'
    )

    open_browser_cmd = 'xdg-open http://127.0.0.1:5173'

    return LaunchDescription([
        ExecuteProcess(
            cmd=["bash", "-lc", backend_cmd],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["bash", "-lc", ui_cmd],
            output="screen",
        ),
        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    cmd=["bash", "-lc", open_browser_cmd],
                    output="screen",
                )
            ],
        ),
    ])
