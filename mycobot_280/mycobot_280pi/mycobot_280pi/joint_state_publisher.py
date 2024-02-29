from __future__ import annotations

import time

import numpy as np
import rclpy
from pymycobot import PI_BAUD, PI_PORT
from pymycobot.mycobot import MyCobot
from rclpy import Parameter
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class JointStatePublisher(Node):
    def __init__(self):
        super().__init__("joint_state_publisher")  # type: ignore
        self.logger = self.get_logger()

        port = self.declare_parameter("port", PI_PORT)
        baudrate = self.declare_parameter("baudrate", PI_BAUD)
        init_qpos = self.declare_parameter("init_qpos", [0.0] * 5 + [0.785398])
        self.declare_parameter("gripper_value_limits", [0, 100], ignore_override=True)

        self.mc = MyCobot(port.value, baudrate.value)

        # Initialize qpos and calibrate gripper
        self.sync_send_radians(init_qpos.value, 50)  # type: ignore
        self.gripper_q_limits = (-0.7, 0.15)  # (low, high)
        self.gripper_value_limits = (0, 100)  # (low, high)
        self.calibrate_gripper()
        self.set_parameters([
            Parameter(
                "gripper_value_limits",
                Parameter.Type.INTEGER_ARRAY,
                list(self.gripper_value_limits),
            )
        ])

        self.joint_pub = self.create_publisher(
            JointState, "joint_states", qos_profile=10
        )
        self.timer = self.create_timer(1 / 30, self.timer_callback)

        # rate = self.create_rate(30)  # 30hz

        # Create joint state
        # joint_state = JointState()
        # joint_state.header = Header()
        # joint_state.name = [
        #     "joint1",
        #     "joint2",
        #     "joint3",
        #     "joint4",
        #     "joint5",
        #     "joint6",
        #     "gripper_controller",
        # ]
        # joint_state.velocity = [0.0] * 7
        # joint_state.effort = [0.0] * 7
        #
        # gripper_q_low, gripper_q_high = self.gripper_q_limits
        # gripper_value_low, gripper_value_high = self.gripper_value_limits
        #
        # while rclpy.ok():
        #     rclpy.spin_once(self)
        #
        #     # Get real angles from server
        #     joint_angles = self.mc.get_radians()
        #     gripper_value = self.mc.get_gripper_value()  # [0, 100]
        #
        #     # Convert gripper value to gripper q value
        #     gripper_q_val = (gripper_value - gripper_value_low) / (
        #         gripper_value_high - gripper_value_low
        #     ) * (gripper_q_high - gripper_q_low) + gripper_q_low
        #     gripper_q_val = np.clip(gripper_q_val, gripper_q_low, gripper_q_high)
        #
        #     state = joint_angles + [gripper_q_val]
        #     self.logger.debug(f"Joint states: {state}")  # noqa: G004
        #
        #     joint_state.header.stamp = self.get_clock().now().to_msg()
        #     joint_state.position = state
        #     self.joint_pub.publish(joint_state)
        #
        #     rate.sleep()

    def timer_callback(self):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.name = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "gripper_controller",
        ]
        joint_state.velocity = [0.0] * 7
        joint_state.effort = [0.0] * 7

        gripper_q_low, gripper_q_high = self.gripper_q_limits
        gripper_value_low, gripper_value_high = self.gripper_value_limits

        # Get real angles from server
        joint_angles = self.mc.get_radians()
        gripper_value = self.mc.get_gripper_value()  # [0, 100]

        # Convert gripper value to gripper q value
        gripper_q_val = (gripper_value - gripper_value_low) / (
            gripper_value_high - gripper_value_low
        ) * (gripper_q_high - gripper_q_low) + gripper_q_low
        gripper_q_val = np.clip(gripper_q_val, gripper_q_low, gripper_q_high)

        state = joint_angles + [gripper_q_val]
        self.logger.debug(f"Joint states: {state}")  # noqa: G004

        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.position = state
        self.joint_pub.publish(joint_state)

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

        self.mc.set_gripper_state(1, 100)  # close gripper
        time.sleep(3)
        while (value_min := self.mc.get_gripper_value()) == -1:
            time.sleep(0.1)
            self.logger.warning("Got -1 gripper value, attempting again...")

        self.mc.set_gripper_state(0, 100)  # open gripper
        time.sleep(3)
        while (value_max := self.mc.get_gripper_value()) == -1:
            time.sleep(0.1)
            self.logger.warning("Got -1 gripper value, attempting again...")

        self.gripper_value_limits = (value_min, value_max)
        self.logger.info(f"Gripper calibrated! (low, high)={self.gripper_value_limits}")  # noqa: G004


def main(args=None):
    rclpy.init(args=args)

    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)

    joint_state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
