from __future__ import annotations

from typing import Any, Dict, List

import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, ListParameters


def _param_value_to_python(v) -> Any:
    t = v.type
    if t == ParameterType.PARAMETER_BOOL:
        return bool(v.bool_value)
    if t == ParameterType.PARAMETER_INTEGER:
        return int(v.integer_value)
    if t == ParameterType.PARAMETER_DOUBLE:
        return float(v.double_value)
    if t == ParameterType.PARAMETER_STRING:
        return str(v.string_value)

    if t == ParameterType.PARAMETER_BYTE_ARRAY:
        return list(v.byte_array_value)
    if t == ParameterType.PARAMETER_BOOL_ARRAY:
        return list(v.bool_array_value)
    if t == ParameterType.PARAMETER_INTEGER_ARRAY:
        return list(v.integer_array_value)
    if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return list(v.double_array_value)
    if t == ParameterType.PARAMETER_STRING_ARRAY:
        return list(v.string_array_value)

    # NOT_SET or unknown
    return None


def read_all_parameters(node_name: str, timeout_sec: float = 2.0) -> Dict[str, Any]:
    """
    Fetch all parameters from a remote node via ROS2 parameter services.
    Returns {param_name: python_value}.
    """
    if not node_name.startswith("/"):
        node_name = f"/{node_name}"

    # Create a temporary node just for service calls
    caller = Node("ros_supervisor_param_reader")

    list_srv = f"{node_name}/list_parameters"
    get_srv = f"{node_name}/get_parameters"

    list_client = caller.create_client(ListParameters, list_srv)
    get_client = caller.create_client(GetParameters, get_srv)

    if not list_client.wait_for_service(timeout_sec=timeout_sec):
        caller.destroy_node()
        raise RuntimeError(f"Service not available: {list_srv}")

    if not get_client.wait_for_service(timeout_sec=timeout_sec):
        caller.destroy_node()
        raise RuntimeError(f"Service not available: {get_srv}")

    # 1) List parameters
    list_req = ListParameters.Request()
    list_req.prefixes = []
    list_req.depth = 0  # 0 = unlimited depth in ROS2 parameter API

    list_future = list_client.call_async(list_req)
    rclpy.spin_until_future_complete(caller, list_future, timeout_sec=timeout_sec)
    if not list_future.done() or list_future.result() is None:
        caller.destroy_node()
        raise RuntimeError(f"Failed to call {list_srv}")

    names: List[str] = list(list_future.result().result.names)

    # Edge case: node exposes no parameters (rare, but possible)
    if not names:
        caller.destroy_node()
        return {}

    # 2) Get parameter values
    get_req = GetParameters.Request()
    get_req.names = names

    get_future = get_client.call_async(get_req)
    rclpy.spin_until_future_complete(caller, get_future, timeout_sec=timeout_sec)
    if not get_future.done() or get_future.result() is None:
        caller.destroy_node()
        raise RuntimeError(f"Failed to call {get_srv}")

    values = get_future.result().values
    out: Dict[str, Any] = {}
    for n, v in zip(names, values):
        out[n] = _param_value_to_python(v)

    caller.destroy_node()
    return out
