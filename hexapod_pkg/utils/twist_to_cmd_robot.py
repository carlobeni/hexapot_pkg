#!/usr/bin/env python3
# twist_to_cmd_robot.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class TwistToCmdRobot(Node):

    def __init__(self):
        super().__init__("twist_to_cmd_robot")

        self.declare_parameter("twist_topic", "/cmd_vel")
        self.declare_parameter("cmd_robot_topic", "/cmd_robot")
        self.declare_parameter("deadzone", 0.01)

        self.twist_topic = self.get_parameter("twist_topic").value
        self.cmd_robot_topic = self.get_parameter("cmd_robot_topic").value
        self.deadzone = float(self.get_parameter("deadzone").value)

        self.pub = self.create_publisher(String, self.cmd_robot_topic, 10)
        self.sub = self.create_subscription(
            Twist, self.twist_topic, self.cb, 10
        )

        self.last_cmd = None

        self.get_logger().info(
            "Twist → cmd_robot READY\n"
            f"  SUB ← {self.twist_topic}\n"
            f"  PUB → {self.cmd_robot_topic}"
        )

    def cb(self, msg: Twist):

        cmd = None

        # ---- LINEAL ----
        if msg.linear.x > self.deadzone:
            cmd = "forward"
        elif msg.linear.x < -self.deadzone:
            cmd = "backward"

        elif msg.linear.y > self.deadzone:
            cmd = "lateral_left"
        elif msg.linear.y < -self.deadzone:
            cmd = "lateral_right"

        # ---- ROTACIÓN ----
        elif msg.angular.z > self.deadzone:
            cmd = "turn_left"
        elif msg.angular.z < -self.deadzone:
            cmd = "turn_right"

        # ---- STOP ----
        else:
            cmd = "stop"

        # Evita spam
        if cmd != self.last_cmd:
            out = String()
            out.data = cmd
            self.pub.publish(out)

            self.get_logger().info(f"[MAP] Twist → cmd_robot = '{cmd}'")
            self.last_cmd = cmd


def main(args=None):
    rclpy.init(args=args)
    node = TwistToCmdRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
