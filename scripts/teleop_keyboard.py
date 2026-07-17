#!/usr/bin/env python3

"""
Node to move a differential-drive mobile robot by sending control velocities
(V, W), saturating maximum velocities.

WARNING: This node does not generate time-based speed ramps. Speed ramping must
be implemented by the motor driver or another node.

Press Ctrl+C to finish.
"""

__author__ = "Jossue Espinoza"

import os
import select
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool

if os.name == "nt":
    import msvcrt
else:
    import termios
    import tty


MAX_LIN_VEL = 6.0
MAX_ANG_VEL = 6.0

LIN_VEL_STEP_SIZE = 0.05
ANG_VEL_STEP_SIZE = 0.1

HELP = """
Control Your Diff-drive Mobile Robot!
-------------------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (max: 6.0 m/s)
a/d : increase/decrease angular velocity (max: 6.0 rad/s)
p   : set maximum forward linear velocity

space, s : force stop
Ctrl+C   : quit
"""


def get_key(settings):
    if os.name == "nt":
        return msvcrt.getch().decode("utf-8")

    tty.setraw(sys.stdin.fileno())
    readable, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if readable else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_velocities(linear_velocity, angular_velocity):
    print(f"currently:\tV: {linear_velocity:.2f}\tW: {angular_velocity:.2f}")


def constrain(value, lower_bound, upper_bound):
    return min(max(value, lower_bound), upper_bound)


def check_linear_limit_velocity(velocity):
    return constrain(velocity, -MAX_LIN_VEL, MAX_LIN_VEL)


def check_angular_limit_velocity(velocity):
    return constrain(velocity, -MAX_ANG_VEL, MAX_ANG_VEL)


def make_twist(linear_velocity, angular_velocity):
    twist = Twist()
    twist.linear.x = linear_velocity
    twist.angular.z = angular_velocity
    return twist


def publish_motor_enable(publisher, enabled):
    message = Bool()
    message.data = enabled
    publisher.publish(message)


def main():
    settings = None
    if os.name != "nt":
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = rclpy.create_node("teleop_keyboard")
    qos = QoSProfile(depth=10)
    publisher = node.create_publisher(Twist, "cmd_vel", qos)
    motor_enable_publisher = node.create_publisher(Bool, "motor_enable", qos)

    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    status = 0

    node.get_logger().warning("Publishing velocity commands on: cmd_vel")
    node.get_logger().info(
        "Namespace example: --ros-args -r __ns:=/agv"
    )
    node.get_logger().info(
        "Topic remap example: --ros-args -r cmd_vel:=/agv/cmd_vel"
    )

    try:
        node.get_logger().info("Waiting for the ODrive motor_enable subscriber...")
        while (
            rclpy.ok()
            and motor_enable_publisher.get_subscription_count() == 0
        ):
            rclpy.spin_once(node, timeout_sec=0.1)

        if not rclpy.ok():
            return

        # Publish repeatedly to cover DDS discovery and immediately arm the
        # driver before accepting keyboard commands.
        for _ in range(3):
            publish_motor_enable(motor_enable_publisher, True)
            rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().warning("Motors enabled automatically")

        print(HELP)
        while rclpy.ok():
            key = get_key(settings)
            if key.lower() == "w":
                target_linear_velocity = check_linear_limit_velocity(
                    target_linear_velocity + LIN_VEL_STEP_SIZE
                )
            elif key.lower() == "x":
                target_linear_velocity = check_linear_limit_velocity(
                    target_linear_velocity - LIN_VEL_STEP_SIZE
                )
            elif key.lower() == "p":
                target_linear_velocity = MAX_LIN_VEL
            elif key.lower() == "a":
                target_angular_velocity = check_angular_limit_velocity(
                    target_angular_velocity + ANG_VEL_STEP_SIZE
                )
            elif key.lower() == "d":
                target_angular_velocity = check_angular_limit_velocity(
                    target_angular_velocity - ANG_VEL_STEP_SIZE
                )
            elif key == " " or key.lower() == "s":
                target_linear_velocity = 0.0
                target_angular_velocity = 0.0
            elif key == "\x03":
                break

            if key:
                status += 1
                print_velocities(
                    target_linear_velocity, target_angular_velocity
                )

            if status >= 20:
                print(HELP)
                status = 0

            publisher.publish(
                make_twist(target_linear_velocity, target_angular_velocity)
            )
            rclpy.spin_once(node, timeout_sec=0.0)

    except Exception as exc:
        node.get_logger().error(f"Teleoperation failed: {exc}")
    finally:
        # Stop first, then put both axes in IDLE before exiting.
        for _ in range(3):
            publisher.publish(make_twist(0.0, 0.0))
            publish_motor_enable(motor_enable_publisher, False)
            rclpy.spin_once(node, timeout_sec=0.05)
        if os.name != "nt" and settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
