#!/usr/bin/env python3
# cmd_vel_relay_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class CmdVelRelay(Node):
    """
    Lee Twist desde un tópico de entrada y lo republica a:
      - /cmd_vel_robot (robot real, para command_talker)
      - /diff_cont/cmd_vel_unstamped (simulación)
    """

    def __init__(self):
        super().__init__("cmd_vel_relay")

        # QoS (ajustable si querés)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Parámetros: entrada y salidas
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("out_robot_topic", "/cmd_vel_robot")
        self.declare_parameter("out_sim_topic", "/diff_cont/cmd_vel_unstamped")

        in_topic = self.get_parameter("input_topic").value
        out_robot = self.get_parameter("out_robot_topic").value
        out_sim = self.get_parameter("out_sim_topic").value

        self.pub_robot = self.create_publisher(Twist, out_robot, qos)
        self.pub_sim = self.create_publisher(Twist, out_sim, qos)

        self.sub = self.create_subscription(Twist, in_topic, self.cb, qos)

        self.get_logger().info(
            f"CmdVelRelay activo:\n"
            f"  SUB  ← {in_topic}\n"
            f"  PUB  → {out_robot}\n"
            f"  PUB  → {out_sim}"
        )

    def cb(self, msg: Twist):
        # Republicar el MISMO mensaje a ambos
        self.pub_robot.publish(msg)
        self.pub_sim.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
