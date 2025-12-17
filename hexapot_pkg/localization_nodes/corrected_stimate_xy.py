#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, String
from rclpy.qos import qos_profile_sensor_data

class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')

        # ======================
        # Parámetros
        # ======================
        self.v = 0.06            # m/s velocidad constante
        self.alpha = 1          # peso del dead-reckoning
        self.update_rate = 20.0    # Hz

        # ======================
        # Estado interno
        # ======================
        self.x = 0.0
        self.y = 0.0
        self.heading_deg = 0.0

        self.gps_x = None
        self.gps_y = None

        self.hl_cmd = "stop"

        self.last_time = self.get_clock().now()

        # ======================
        # Subscriptores
        # ======================
        self.create_subscription(
            PointStamped,
            '/localization/gps/local_xy',
            self.gps_cb,
            10
        )

        self.sub_heading = self.create_subscription(
            Float32, "/localization/heading_deg", self.heading_cb,
            qos_profile_sensor_data
        )

        self.create_subscription(
            String,
            '/hl_cmd',
            self.hl_cmd_cb,
            10
        )

        # ======================
        # Publisher
        # ======================
        self.pub_pose = self.create_publisher(
            PointStamped,
            '/localization/local_stimate_xy',
            10
        )

        # ======================
        # Timer principal
        # ======================
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.update
        )

        self.get_logger().info("Pose estimator iniciado")

    # ======================
    # Callbacks
    # ======================
    def gps_cb(self, msg):
        self.gps_x = msg.point.x
        self.gps_y = msg.point.y

    def heading_cb(self, msg):
        self.heading_deg = msg.data

    def hl_cmd_cb(self, msg):
        self.hl_cmd = msg.data.lower()

    # ======================
    # Loop principal
    # ======================
    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0:
            return

        # ----------------------
        # Integración cinemática
        # ----------------------
        if self.hl_cmd == "forward":
            heading_rad = math.radians(self.heading_deg)

            dx = self.v * math.cos(heading_rad) * dt
            dy = self.v * math.sin(heading_rad) * dt

            self.x += dx
            self.y += dy

        if self.hl_cmd == "backward":
            heading_rad = math.radians(self.heading_deg)

            dx = self.v * math.cos(heading_rad) * dt
            dy = self.v * math.sin(heading_rad) * dt

            self.x -= dx
            self.y -= dy

        # ----------------------
        # Corrección con GPS
        # ----------------------
        if self.gps_x is not None:
            self.x = self.alpha * self.x + (1 - self.alpha) * self.gps_x
            self.y = self.alpha * self.y + (1 - self.alpha) * self.gps_y

        # ----------------------
        # Publicar resultado
        # ----------------------
        msg = PointStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "map"

        msg.point.x = self.x
        msg.point.y = self.y
        msg.point.z = 0.0

        self.pub_pose.publish(msg)


def main():
    rclpy.init()
    node = PoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
