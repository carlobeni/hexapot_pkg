#!/usr/bin/env python3
# command_talker_node.py

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)

import hw_config as cfg


class CommandTalker(Node):
    """
    Nodo puente:
    Twist (/cmd_vel_robot) → String (/cmd_serial)
    """

    def __init__(self):
        super().__init__("command_talker")
        cfg.check_domain_id(self.get_logger())

        # ===== QoS por parámetros =====
        self.declare_parameter("qos_reliability", "reliable")
        self.declare_parameter("qos_history", "keep_last")
        self.declare_parameter("qos_depth", 10)
        self.declare_parameter("qos_durability", "volatile")

        qos = self._build_qos()

        # ===== ID GLOBAL =====
        self.cmd_id = 0

        # ===== PUBLISHER (→ Pi5) =====
        self.pub = self.create_publisher(
            String,
            cfg.TOPIC_CMD_SERIAL,
            qos
        )

        # ===== SUBSCRIBER (← Managers / twist_mux) =====
        self.sub = self.create_subscription(
            Twist,
            cfg.TOPIC_CMD_VEL_ROBOT,
            self.twist_cb,
            qos
        )

        self.get_logger().info(
            f"CommandTalker activo:"
            f"\n  SUB  ← {cfg.TOPIC_CMD_VEL_ROBOT}"
            f"\n  PUB  → {cfg.TOPIC_CMD_SERIAL}"
        )

    # --------------------------------------------------

    def _build_qos(self):
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE
            if self.get_parameter("qos_reliability").value == "reliable"
            else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.get_parameter("qos_depth").value,
            durability=DurabilityPolicy.VOLATILE,
        )

    # --------------------------------------------------

    def twist_cb(self, msg: Twist):
        """
        Convierte Twist a comando serial para hexápodo
        """

        vx = msg.linear.x
        vy = msg.linear.y

        yaw   = msg.angular.z
        pitch = msg.angular.y
        roll  = msg.angular.x

        self.cmd_id += 1

        # cmd_txt = (
        #     f"{self.cmd_id}:MOVE "
        #     f"{vx:.3f} {vy:.3f} "
        #     f"{yaw:.3f} {pitch:.3f} {roll:.3f}"
        # )
        cmd_txt = (
            f"{self.cmd_id}:SET90"
        )

        out = String()
        out.data = cmd_txt
        self.pub.publish(out)

        self.get_logger().info(f"→ {cmd_txt}")


# ======================================================

def main(args=None):
    rclpy.init(args=args)
    node = CommandTalker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
