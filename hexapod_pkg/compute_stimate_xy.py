#!/usr/bin/env python3
# compute_stimate_xy.py

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, String
from rclpy.qos import qos_profile_sensor_data


class PoseEstimator(Node):

    def __init__(self):
        super().__init__("compute_stimate_xy")

        # =====================================================
        # PARÁMETROS (CONFIGURABLES DESDE EL LAUNCHER)
        # =====================================================
        self.declare_parameter("gps_xy_topic", "/localization/gps/local_xy")
        self.declare_parameter("heading_topic", "/localization/heading_deg")
        self.declare_parameter("hl_cmd_topic", "/hl_cmd")
        self.declare_parameter("output_topic", "/localization/local_stimate_xy")

        self.declare_parameter("velocity", 0.06)      # [m/s]
        self.declare_parameter("alpha", 1.0)          # peso dead-reckoning
        self.declare_parameter("update_rate", 20.0)   # [Hz]

        # =====================================================
        # LECTURA DE PARÁMETROS
        # =====================================================
        gps_xy_topic = self.get_parameter("gps_xy_topic").value
        heading_topic = self.get_parameter("heading_topic").value
        hl_cmd_topic = self.get_parameter("hl_cmd_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.v = self.get_parameter("velocity").value
        self.alpha = self.get_parameter("alpha").value
        self.update_rate = self.get_parameter("update_rate").value

        # =====================================================
        # ESTADO INTERNO
        # =====================================================
        self.x = 0.0
        self.y = 0.0
        self.heading_deg = 0.0

        self.gps_x = None
        self.gps_y = None

        self.hl_cmd = "stop"
        self.last_time = self.get_clock().now()

        # =====================================================
        # SUBSCRIPTORES
        # =====================================================
        self.create_subscription(
            PointStamped,
            gps_xy_topic,
            self.gps_cb,
            10,
        )

        self.create_subscription(
            Float32,
            heading_topic,
            self.heading_cb,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            String,
            hl_cmd_topic,
            self.hl_cmd_cb,
            10,
        )

        # =====================================================
        # PUBLISHER
        # =====================================================
        self.pub_pose = self.create_publisher(
            PointStamped,
            output_topic,
            10,
        )

        # =====================================================
        # TIMER
        # =====================================================
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.update,
        )

        # =====================================================
        # LOG
        # =====================================================
        self.get_logger().info(
            f"Pose estimator READY | gps={gps_xy_topic} heading={heading_topic}"
        )
        self.get_logger().info(
            f"v={self.v} m/s alpha={self.alpha} rate={self.update_rate} Hz"
        )

    # =====================================================
    # CALLBACKS
    # =====================================================
    def gps_cb(self, msg: PointStamped):
        self.gps_x = msg.point.x
        self.gps_y = msg.point.y

    def heading_cb(self, msg: Float32):
        self.heading_deg = msg.data

    def hl_cmd_cb(self, msg: String):
        self.hl_cmd = msg.data.lower()

    # =====================================================
    # LOOP PRINCIPAL
    # =====================================================
    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0:
            return

        # ----------------------
        # INTEGRACIÓN CINEMÁTICA
        # ----------------------
        if self.hl_cmd in ("forward", "backward"):
            heading_rad = math.radians(self.heading_deg)

            dx = self.v * math.cos(heading_rad) * dt
            dy = self.v * math.sin(heading_rad) * dt

            if self.hl_cmd == "forward":
                self.x += dx
                self.y += dy
            else:  # backward
                self.x -= dx
                self.y -= dy

        # ----------------------
        # CORRECCIÓN CON GPS
        # ----------------------
        if self.gps_x is not None:
            self.x = self.alpha * self.x + (1.0 - self.alpha) * self.gps_x
            self.y = self.alpha * self.y + (1.0 - self.alpha) * self.gps_y

        # ----------------------
        # PUBLICAR RESULTADO
        # ----------------------
        msg = PointStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "map"

        msg.point.x = self.x
        msg.point.y = self.y
        msg.point.z = 0.0

        self.pub_pose.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
