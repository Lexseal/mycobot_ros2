"""MyCobot driver node. Tested with pymycobot==3.3.0"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from mycobot_interfaces.srv import (
    CloseGripper,
    OpenGripper,
    SetAngles,
    SetCoords,
    SetGripper,
)
from pymycobot import PI_BAUD, PI_PORT
from pymycobot.mycobot import MyCobot
from rcl_interfaces.srv import GetParameters
from rclpy import Parameter
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node as ROSNode
from rclpy.parameter import ParameterValue
from rclpy.signals import SignalHandlerGuardCondition
from rclpy.utilities import timeout_sec_to_nsec
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


def wait_for_message(msg_type, node: ROSNode, topic: str, time_to_wait=-1):
    """
    Wait for the next incoming message.

    :param msg_type: message type
    :param node: node to initialize the subscription on
    :param topic: topic name to wait for message
    :param time_to_wait: seconds to wait before returning
    :returns: (True, msg) if a message was successfully received, (False, None) if message
        could not be obtained or shutdown was triggered asynchronously on the context.
    """
    # NOTE: this file is directly taken from rclpy rolling branch:
    # https://github.com/ros2/rclpy/blob/220d714b2b6da81a4abd6a804e3ed4ee8cfd7c3f/rclpy/rclpy/wait_for_message.py
    context = node.context
    wait_set = _rclpy.WaitSet(1, 1, 0, 0, 0, 0, context.handle)
    wait_set.clear_entities()

    sub = node.create_subscription(msg_type, topic, lambda _: None, 1)
    try:
        wait_set.add_subscription(sub.handle)
        sigint_gc = SignalHandlerGuardCondition(context=context)
        wait_set.add_guard_condition(sigint_gc.handle)

        timeout_nsec = timeout_sec_to_nsec(time_to_wait)
        wait_set.wait(timeout_nsec)

        subs_ready = wait_set.get_ready_entities("subscription")
        guards_ready = wait_set.get_ready_entities("guard_condition")

        if guards_ready and sigint_gc.handle.pointer in guards_ready:
            return False, None

        if subs_ready and sub.handle.pointer in subs_ready:
            msg_info = sub.handle.take_message(sub.msg_type, sub.raw)
            if msg_info is not None:
                return True, msg_info[0]
    finally:
        node.destroy_subscription(sub)

    return False, None


class Node(ROSNode):
    """Wrapper class to add some convenience methods from ROS2 rolling to ROS2 nodes."""

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)

    def get_fully_qualified_node_names(self) -> list[str]:
        """
        Get a list of fully qualified names for discovered nodes.

        Similar to ``get_node_names_namespaces()``, but concatenates the names and namespaces.

        :return: List of fully qualified node names.

        Copied from ROS2 rolling:
        https://github.com/ros2/rclpy/blob/5cbb110b8ac301d7cdf27768010367dc27f92f59/rclpy/rclpy/node.py#L2076
        """  # noqa: E501
        names_and_namespaces = self.get_node_names_and_namespaces()
        return [
            ns + ("" if ns.endswith("/") else "/") + name
            for name, ns in names_and_namespaces
        ]

    def wait_for_node(self, fully_qualified_node_name: str, timeout: float) -> bool:
        """
        Wait until node name is present in the system or timeout.

        The node name should be the full name with namespace.

        :param node_name: Fully qualified name of the node to wait for.
        :param timeout: Seconds to wait for the node to be present. If negative, the function
                         won't timeout.
        :return: ``True`` if the node was found, ``False`` if timeout.

        Copied from ROS2 rolling:
        https://github.com/ros2/rclpy/blob/5cbb110b8ac301d7cdf27768010367dc27f92f59/rclpy/rclpy/node.py#L2269
        """  # noqa: E501
        if not fully_qualified_node_name.startswith("/"):
            fully_qualified_node_name = f"/{fully_qualified_node_name}"

        start = time.time()
        flag = False
        # TODO refactor this implementation when we can react to guard condition events, or replace  # noqa: E501
        # it entirely with an implementation in rcl. see https://github.com/ros2/rclpy/issues/929
        while time.time() - start < timeout and not flag:
            fully_qualified_node_names = self.get_fully_qualified_node_names()
            flag = fully_qualified_node_name in fully_qualified_node_names
            time.sleep(0.1)
        return flag

    def get_remote_parameters(
        self, remote_node_name: str, parameter_names: str | list[str], timeout_sec=10
    ) -> ParameterValue | list[ParameterValue]:
        input_is_list = isinstance(parameter_names, list)

        client = self.create_client(GetParameters, f"{remote_node_name}/get_parameters")
        assert client.wait_for_service(
            timeout_sec=timeout_sec
        ), f"Service does not become ready within {timeout_sec} seconds"

        request = GetParameters.Request()
        request.names = parameter_names if input_is_list else [parameter_names]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        assert future.done(), f"Failed to get response within {timeout_sec} seconds"

        client.destroy()
        return future.result().values if input_is_list else future.result().values[0]  # type: ignore


class Driver(Node):
    def __init__(self):
        super().__init__("mycobot_driver")
        self.logger = self.get_logger()

        port = self.declare_parameter("port", PI_PORT)
        baudrate = self.declare_parameter("baudrate", PI_BAUD)
        init_qpos = self.declare_parameter("init_qpos", [0.0] * 5 + [0.785398])
        publish_frequency = self.declare_parameter("publish_frequency", 30)
        self.declare_parameter("gripper_value_limits", [0, 100], ignore_override=True)

        self.mc = MyCobot(port.value, baudrate.value, debug=False, thread_lock=True)  # type: ignore
        time.sleep(0.05)
        self.mc.set_free_mode(1)
        time.sleep(0.05)

        # Initialize qpos and calibrate gripper
        self.sync_send_radians(init_qpos.value, 50)  # type: ignore
        self.gripper_q_limits = (-0.7, 0.15)  # (low, high)
        self.gripper_value_limits: tuple[int, int] = (0, 100)  # (low, high)
        self.calibrate_gripper()
        self.set_parameters([
            Parameter(
                "gripper_value_limits",
                Parameter.Type.INTEGER_ARRAY,
                list(self.gripper_value_limits),
            )
        ])

        # Publishing joint states
        self.joint_pub = self.create_publisher(
            JointState, "joint_states", qos_profile=10
        )
        self.timer = self.create_timer(
            1 / publish_frequency.value,  # type: ignore
            self.publish_joint_states_callback,
        )
        self.joint_state_msg = JointState()
        self.joint_state_msg.header = Header()
        self.joint_state_msg.name = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "gripper_controller",
        ]
        self.joint_state_msg.velocity = [0.0] * 7
        self.joint_state_msg.effort = [0.0] * 7

        # Services for controlling the robot
        self.srv_set_angles = self.create_service(
            SetAngles, "set_angles", self.set_angles_callback
        )
        self.srv_set_coords = self.create_service(
            SetCoords, "set_coords", self.set_coords_callback
        )
        self.srv_set_gripper = self.create_service(
            SetGripper, "set_gripper", self.set_gripper_callback
        )
        self.srv_open_gripper = self.create_service(
            OpenGripper, "open_gripper", self.open_gripper_callback
        )
        self.srv_close_gripper = self.create_service(
            CloseGripper, "close_gripper", self.close_gripper_callback
        )

        self.logger.info("MyCobot_280pi driver is ready.")

    def publish_joint_states_callback(self):
        gripper_q_low, gripper_q_high = self.gripper_q_limits
        gripper_value_low, gripper_value_high = self.gripper_value_limits

        # Get real angles from server
        joint_angles = self.mc.get_radians()
        while (gripper_value := self.mc.get_gripper_value()) == -1:  # [0, 100]
            time.sleep(0.1)
            self.logger.warning("Got -1 gripper value, attempting again...")

        # Convert gripper value to gripper q value
        gripper_q_val = (gripper_value - gripper_value_low) / (  # type: ignore
            gripper_value_high - gripper_value_low
        ) * (gripper_q_high - gripper_q_low) + gripper_q_low
        gripper_q_val = np.clip(gripper_q_val, gripper_q_low, gripper_q_high)

        state = joint_angles + [gripper_q_val]
        self.logger.debug(f"Joint states: {state}")  # noqa: G004

        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.position = state
        self.joint_pub.publish(self.joint_state_msg)

    def sync_send_radians(self, angles: list[float], speed: int, timeout=15):
        """
        Send the angle in synchronous state and return when the target point is reached

        Args:
            angles: a list of angle values in radians, length 6.
            speed: (int) 0 ~ 100
            timeout: default 15s.
        """
        degrees = np.asarray(angles) / np.pi * 180
        start_t = time.time()
        self.mc.send_radians(angles, speed)
        while time.time() - start_t < timeout:
            if self.mc.is_in_position(degrees, 0) == 1:
                break
            time.sleep(0.1)

    def calibrate_gripper(self):
        """Calibrate MyCobot Adaptive Gripper for conversion parameters"""
        self.logger.info("Calibrating Gripper...")

        while True:
            self.mc.set_gripper_state(1, 100)  # close gripper
            time.sleep(3)
            while (value_min := self.mc.get_gripper_value()) == -1:
                time.sleep(0.1)
                self.logger.warning("Got -1 gripper value, attempting again...")
            if value_min < 50:  # type: ignore
                break
            self.logger.warning(
                f"Got gripper value={value_min} >= 50 when closed, attempting again..."  # noqa: G004
            )

        while True:
            self.mc.set_gripper_state(0, 100)  # open gripper
            time.sleep(3)
            while (value_max := self.mc.get_gripper_value()) == -1:
                time.sleep(0.1)
                self.logger.warning("Got -1 gripper value, attempting again...")
            if value_max > 50:  # type: ignore
                break
            self.logger.warning(
                f"Got gripper value={value_max} <= 50 when opened, attempting again..."  # noqa: G004
            )

        self.gripper_value_limits = (value_min, value_max)  # type: ignore
        self.logger.info(f"Gripper calibrated! (low, high)={self.gripper_value_limits}")  # noqa: G004

    def set_angles_callback(
        self, request: SetAngles.Request, response: SetAngles.Response
    ) -> SetAngles.Response:
        """
        :param request: SetAngles.Request
            angles: list of joint angles in radians
            speed: integer, range: [0, 100]
        :param response: SetAngles.Response
            flag: bool, success or failure
        """
        self.logger.info(
            f"Received SetAngles angles={request.angles} speed={request.speed}"  # noqa: G004
        )

        self.mc.send_radians(request.angles, request.speed)

        response.flag = True
        return response

    def set_coords_callback(
        self, request: SetCoords.Request, response: SetCoords.Response
    ) -> SetCoords.Response:
        """
        :param request: SetCoords.Request
            pose: list of end effector's pose, [x, y, z, rx, ry, rz]
            speed: integer, range: [0, 100]
            mode: bool, False: non-linear head movement path.
                  True: linear head movement path.
        :param response: SetCoords.Response
            flag: bool, success or failure
        """
        self.logger.info(
            f"Received SetCoords pose={request.pose} speed={request.speed} "  # noqa: G004
            f"mode={request.mode}"
        )

        self.mc.send_coords(request.pose, request.speed, int(request.mode))

        response.flag = True
        return response

    def set_gripper_callback(
        self, request: SetGripper.Request, response: SetGripper.Response
    ) -> SetGripper.Response:
        """
        :param request: SetGripper.Request
            value: gripper value, range: [-0.7, 0.15]. -0.7 is closed, 0.15 is opened.
            speed: integer, range: [0, 100]
        :param response: SetGripper.Response
            flag: bool, success or failure
        """
        self.logger.info(
            f"Received SetGripper value={request.value} speed={request.speed}"  # noqa: G004
        )

        gripper_q_low, gripper_q_high = self.gripper_q_limits
        gripper_value_low, gripper_value_high = self.gripper_value_limits

        gripper_q_val = np.clip(request.value, gripper_q_low, gripper_q_high)
        gripper_value = (gripper_q_val - gripper_q_low) / (
            gripper_q_high - gripper_q_low
        ) * (gripper_value_high - gripper_value_low) + gripper_value_low
        self.logger.info(f"Setting gripper value={gripper_value}")  # noqa: G004

        self.mc.set_gripper_value(int(gripper_value), request.speed)

        response.flag = True
        return response

    def open_gripper_callback(
        self, request: OpenGripper.Request, response: OpenGripper.Response
    ) -> OpenGripper.Response:
        """
        :param request: OpenGripper.Request
            speed: integer, range: [0, 100]
        :param response: OpenGripper.Response
            flag: bool, success or failure
        """
        self.logger.info(f"Received OpenGripper speed={request.speed}")  # noqa: G004

        self.mc.set_gripper_state(0, request.speed)

        response.flag = True
        return response

    def close_gripper_callback(
        self, request: CloseGripper.Request, response: CloseGripper.Response
    ) -> CloseGripper.Response:
        """
        :param request: CloseGripper.Request
            speed: integer, range: [0, 100]
        :param response: CloseGripper.Response
            flag: bool, success or failure
        """
        self.logger.info(f"Received CloseGripper speed={request.speed}")  # noqa: G004

        self.mc.set_gripper_state(1, request.speed)

        response.flag = True
        return response


def main(args=None):
    rclpy.init(args=args)

    driver_node = Driver()
    rclpy.spin(driver_node)

    driver_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
