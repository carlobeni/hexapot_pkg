#!/usr/bin/env python3
import math
import csv
import os
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from hexapod_pkg import hw_config as cfg

class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')

        self.declare_parameter("topic_gps_to_xy", cfg.TOPIC_XY_BY_GPS_CURRENT_POSITION)
        self.declare_parameter("topic_estimate_heading", cfg.TOPIC_HEADING_COMPASS)
        self.declare_parameter("topic_estimate_xy", cfg.TOPIC_XY_ODOM_CURRENT_POSITION)
        self.declare_parameter("topic_cmd_robot", cfg.TOPIC_CMD_GZ_ROBOT)

        topic_gps_to_xy = self.get_parameter("topic_gps_to_xy").value
        topic_estimate_heading = self.get_parameter("topic_estimate_heading").value
        topic_estimate_xy= self.get_parameter("topic_estimate_xy").value
        topic_cmd_robot = self.get_parameter("topic_cmd_robot").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ======================
        # Parámetros
        # ======================
        self.v = 0.105            # m/s velocidad constante
        self.alpha = 0          # peso del dead-reckoning
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
        self.start_time = self.last_time.nanoseconds * 1e-9

        # ======================
        # Subscriptores
        # ======================
        self.create_subscription(
            PointStamped,
            topic_gps_to_xy,
            self.gps_cb,
            10
        )

        self.sub_heading = self.create_subscription(
            Float32, topic_estimate_heading, self.heading_cb,
            qos
        )

        self.create_subscription(
            String,
            topic_cmd_robot,
            self.hl_cmd_cb,
            10
        )


        # ======================
        # Publisher
        # ======================
        self.pub_pose = self.create_publisher(
            PointStamped,
            topic_estimate_xy,
            qos
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
        # Definir directorio formato y directorio para el csv
        # ======================
        log_dir = os.path.expanduser(
            "~/ros2_projects/ros2_hex_ws/src/csv"
        )
        os.makedirs(log_dir, exist_ok=True)
        self.csv_path = os.path.join(log_dir, "pose_estimator_log.csv")
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            "time_s",
            "gps_x", "gps_y",
            "est_x", "est_y",
            "heading_deg",
            "cmd_robot"
        ])

        self.start_time = self.get_clock().now().nanoseconds * 1e-9

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

        # ---------------------
        # Escribir en csv
        # ---------------------
        t = now.nanoseconds * 1e-9 - self.start_time
        self.csv_writer.writerow([
            f"{(now.nanoseconds * 1e-9 - self.start_time):.3f}",
            f"{self.gps_x:.3f}" if self.gps_x is not None else "NaN",
            f"{self.gps_y:.3f}" if self.gps_y is not None else "NaN",
            f"{self.x:.3f}", 
            f"{self.y:.3f}",
            f"{self.heading_deg:.2f}",
            self.hl_cmd
        ])

def main():
    rclpy.init()
    node = PoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
