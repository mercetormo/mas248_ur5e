#!/usr/bin/env python3
"""Check the live MAS248 stack in planning-only mode."""
import argparse
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

from validate_description import (
    check_measured_workcell,
    hardware_parameters,
    load_common,
    read_calibration_hash,
)


SCALED = "scaled_joint_trajectory_controller"
REQUIRED_ACTIVE = {"joint_state_broadcaster", "io_and_status_controller"}
MOTION_CONTROLLERS = {
    SCALED, "joint_trajectory_controller", "forward_position_controller",
    "forward_velocity_controller", "forward_effort_controller", "force_mode_controller",
    "passthrough_trajectory_controller", "freedrive_mode_controller",
    "tool_contact_controller",
}


def request(node, service_type, name, payload, timeout):
    import rclpy

    client = node.create_client(service_type, name)
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"Service unavailable: {name}")
        future = client.call_async(payload)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError(f"Service timed out: {name}")
        return future.result()
    finally:
        node.destroy_client(client)


def parameter_values(node, owner, names, timeout):
    from rcl_interfaces.srv import GetParameters

    payload = GetParameters.Request()
    payload.names = list(names)
    values = request(node, GetParameters, owner + "/get_parameters", payload, timeout).values
    if len(values) != len(names):
        raise RuntimeError(f"Missing parameters on {owner}: {names}")
    return values


def check_controllers(controllers):
    states = {controller.name: controller.state for controller in controllers}
    for name in sorted(REQUIRED_ACTIVE):
        if states.get(name) != "active":
            raise RuntimeError(f"{name} must be active; observed {states.get(name)}.")
    if states.get(SCALED) != "inactive":
        raise RuntimeError(f"{SCALED} must be inactive; observed {states.get(SCALED)}.")
    active_motion = sorted(name for name in MOTION_CONTROLLERS if states.get(name) == "active")
    if active_motion:
        raise RuntimeError(f"Unexpected active motion controllers: {active_motion}")


def wait_for_live_state(node, joints, timeout):
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool

    latest_joints = None
    program_running = None

    def on_joints(message):
        nonlocal latest_joints
        latest_joints = message

    def on_program(message):
        nonlocal program_running
        program_running = message.data

    joint_sub = node.create_subscription(
        JointState, "/joint_states", on_joints, qos_profile_sensor_data
    )
    program_sub = node.create_subscription(
        Bool, "/io_and_status_controller/robot_program_running", on_program,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    try:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if latest_joints is None or program_running is not True:
                continue
            names = latest_joints.name
            positions = latest_joints.position
            if (len(names) == len(positions) and len(set(names)) == len(names)
                    and set(joints).issubset(names) and all(math.isfinite(x) for x in positions)):
                return
        if program_running is not True:
            raise RuntimeError("External Control program is not confirmed running.")
        raise RuntimeError("No complete UR5e joint state received.")
    finally:
        node.destroy_subscription(program_sub)
        node.destroy_subscription(joint_sub)


def check_ready(node, *, calibration=None, timeout=12.0):
    from controller_manager_msgs.srv import ListControllers
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.srv import GetCartesianPath
    from rclpy.action import ActionClient
    from rcl_interfaces.msg import ParameterType

    common = load_common()
    moveit, execution = parameter_values(
        node, "/move_group", ("robot_description", "allow_trajectory_execution"), timeout
    )
    (driver,) = parameter_values(node, "/robot_state_publisher", ("robot_description",), timeout)
    if execution.type != ParameterType.PARAMETER_BOOL or execution.bool_value:
        raise RuntimeError("MoveIt must be started with enable_execution:=false.")
    if any(value.type != ParameterType.PARAMETER_STRING for value in (driver, moveit)):
        raise RuntimeError("Missing robot description on MoveIt or robot_state_publisher.")
    common.validate_rendered(driver.string_value)
    common.validate_rendered(moveit.string_value)
    if ET.canonicalize(driver.string_value, strip_text=True) != ET.canonicalize(
        moveit.string_value, strip_text=True
    ):
        raise RuntimeError("Driver and MoveIt robot descriptions differ.")
    root = check_measured_workcell(driver.string_value)
    hardware = hardware_parameters(root)
    value = hardware.get("kinematics/hash", "")
    if not value.startswith("calib_"):
        raise RuntimeError("Robot description has no UR calibration hash.")
    if calibration is not None and value != read_calibration_hash(calibration):
        raise RuntimeError("The loaded model calibration does not match the supplied YAML.")
    response = request(node, ListControllers, "/controller_manager/list_controllers",
                       ListControllers.Request(), timeout)
    check_controllers(response.controller)
    nodes = node.get_node_names_and_namespaces()
    for name in ("move_group", "robot_state_publisher", "controller_manager"):
        if nodes.count((name, "/")) != 1:
            raise RuntimeError(f"Expected one /{name}; check the ROS domain and old launches.")
    action = ActionClient(node, MoveGroup, "/move_action")
    cartesian = node.create_client(GetCartesianPath, "/compute_cartesian_path")
    try:
        if not action.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("MoveIt action /move_action is unavailable.")
        if not cartesian.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("MoveIt service /compute_cartesian_path is unavailable.")
    finally:
        node.destroy_client(cartesian)
        action.destroy()
    wait_for_live_state(node, common.JOINTS, timeout)
    if len(node.get_publishers_info_by_topic("/joint_states")) != 1:
        raise RuntimeError("Expected one /joint_states publisher; check for old launches.")
    node.get_logger().info(
        f"Planning-only stack ready. Measured workcell, matching URDFs, "
        f"calibration hash {value}, External Control and joint states checked."
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", type=Path, help="Compare the loaded hash to this YAML")
    args = parser.parse_args()

    try:
        import rclpy
        from rclpy.node import Node

        rclpy.init(args=[])
        try:
            node = Node("mas248_validate_real_stack")
            try:
                check_ready(node, calibration=args.calibration)
            finally:
                node.destroy_node()
        finally:
            rclpy.try_shutdown()
    except (Exception, KeyboardInterrupt) as error:
        print(f"Stack validation failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
