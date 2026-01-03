import threading
import rclpy

_LOCK = threading.Lock()
_INITED = False

def init_once():
    global _INITED
    with _LOCK:
        # If ROS is already up, just mark it.
        if _INITED and rclpy.ok():
            return
        if not rclpy.ok():
            rclpy.init(args=None)
        _INITED = True

def shutdown_once():
    global _INITED
    with _LOCK:
        if rclpy.ok():
            rclpy.shutdown()
        _INITED = False
