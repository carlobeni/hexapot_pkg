#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ControlMasterHexapod(Node):
    def __init__(self):
        super().__init__("control_master_hexapod")

        self.pub = self.create_publisher(
            String,
            "/control_master/mode",
            10
        )

        self.mode = "teleop"  # modo por defecto

        self.timer = self.create_timer(0.2, self.publish_mode)

        self.get_logger().info(
            "Control Master iniciado\n"
            "Modos disponibles:\n"
            " - teleop\n"
            " - navigation\n"
            " - social\n"
            "Usa: ros2 topic pub /control_master/mode std_msgs/String '{data: teleop}'"
        )

    def publish_mode(self):
        msg = String()
        msg.data = self.mode
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControlMasterHexapod()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
