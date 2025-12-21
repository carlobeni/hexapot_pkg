#!/usr/bin/env python3
# compute_stimate_xy.py

import math
import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, String
from rclpy.qos import qos_profile_sensor_data


class PoseEstimator(Node):

    def __init__(self):
        super().__init__("compute_stimate_xy")

        # ================= PARÁMETROS =================
        self.declare_parameter("gps_xy_topic", "/localization/gps/local_xy")
        self.declare_parameter("heading_topic", "/localization/heading_deg")
        self.declare_parameter("hl_cmd_topic", "/hl_cmd")
        self.declare_parameter("output_topic", "/localization/local_stimate_xy")

        self.declare_parameter("velocity", 0.06)     # [m/s]
        self.declare_parameter("alpha", 0.95)        # estimador dominante
        self.declare_parameter("update_rate", 10.0)  # Hz
        self.declare_parameter("fixed_dt", 0.0)      # 0 → usar dt real

        # ================= LECTURA =================
        self.v = float(self.get_parameter("velocity").value)
        self.alpha = float(self.get_parameter("alpha").value)
        self.update_rate = float(self.get_parameter("update_rate").value)
        self.fixed_dt = float(self.get_parameter("fixed_dt").value)

        gps_xy_topic = self.get_parameter("gps_xy_topic").value
        heading_topic = self.get_parameter("heading_topic").value
        hl_cmd_topic = self.get_parameter("hl_cmd_topic").value
        output_topic = self.get_parameter("output_topic").value

        # ================= ESTADO =================
        self.x = None
        self.y = None
        self.heading_deg = 0.0

        self.gps_x = None
        self.gps_y = None

        # desplazamiento acumulado SOLO en forward
        self.dx_cmd = 0.0
        self.dy_cmd = 0.0

        self.hl_cmd = "stop"
        self.last_time = self.get_clock().now()

        # GPS lento (1 Hz)
        self.gps_period = 1.0  # [s]
        self.last_gps_correction_time = None

        # ================= SUBS =================
        self.create_subscription(PointStamped, gps_xy_topic, self.gps_cb, 10)
        self.create_subscription(Float32, heading_topic, self.heading_cb, qos_profile_sensor_data)
        self.create_subscription(String, hl_cmd_topic, self.hl_cmd_cb, 10)

        self.pub_pose = self.create_publisher(PointStamped, output_topic, 10)

        self.timer = self.create_timer(1.0 / self.update_rate, self.update)

        # ================= CSV =================
        log_dir = os.path.expanduser(
            "~/ros2_projects/ros2_hex_ws/src/hexapod_pkg/logs"
        )
        os.makedirs(log_dir, exist_ok=True)

        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_path = os.path.join(log_dir, f"pose_estimator_log.csv")

        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            "time_s",
            "gps_x", "gps_y",
            "est_x", "est_y",
            "dx_cmd", "dy_cmd",
            "heading_deg",
            "dt_used",
            "gps_correction",
            "err_x", "err_y", "err_norm",
            "hl_cmd"
        ])

        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info("PoseEstimator READY")
        self.get_logger().info("Ejes: X=NORTE, Y=ESTE")
        self.get_logger().info(f"v={self.v} m/s | alpha={self.alpha}")
        self.get_logger().info("GPS correction period = 1.0 s")
        self.get_logger().info(f"CSV → {self.csv_path}")

    # ================= CALLBACKS =================
    def gps_cb(self, msg: PointStamped):
        self.gps_x = msg.point.x
        self.gps_y = msg.point.y

        if self.x is None:
            self.x = self.gps_x
            self.y = self.gps_y
            self.get_logger().info("Inicializado desde GPS")

    def heading_cb(self, msg: Float32):
        self.heading_deg = float(msg.data)

    def hl_cmd_cb(self, msg: String):
        self.hl_cmd = msg.data.lower()

    # ================= LOOP =================
    def update(self):
        if self.x is None or self.gps_x is None:
            return

        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9 - self.start_time

        # ---------------- DT ----------------
        if self.fixed_dt > 0.0:
            dt = self.fixed_dt
        else:
            dt = (now - self.last_time).nanoseconds * 1e-9
            self.last_time = now

        if dt <= 0.0:
            return

        # ---------------- DEAD RECKONING ----------------
        if self.hl_cmd == "forward":
            heading_rad = math.radians(self.heading_deg)

            dx = self.v * math.cos(heading_rad) * dt   # NORTE
            dy = self.v * math.sin(heading_rad) * dt   # ESTE

            self.dx_cmd += dx
            self.dy_cmd += dy

            self.x += dx
            self.y += dy

        # ---------------- GPS LENTO (1 Hz) ----------------
        gps_correction = False

        if self.last_gps_correction_time is None:
            gps_correction = True
        else:
            dt_gps = (now - self.last_gps_correction_time).nanoseconds * 1e-9
            if dt_gps >= self.gps_period:
                gps_correction = True

        if gps_correction:
            self.x = self.alpha * self.x + (1.0 - self.alpha) * self.gps_x
            self.y = self.alpha * self.y + (1.0 - self.alpha) * self.gps_y
            self.last_gps_correction_time = now

        # ---------------- ERRORES ----------------
        ex = self.x - self.gps_x
        ey = self.y - self.gps_y
        e = math.hypot(ex, ey)

        # ---------------- PUBLICAR ----------------
        msg = PointStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "map"
        msg.point.x = self.x
        msg.point.y = self.y
        msg.point.z = 0.0
        self.pub_pose.publish(msg)

        # ---------------- CSV ----------------
        self.csv_writer.writerow([
            f"{t:.3f}",
            f"{self.gps_x:.3f}", f"{self.gps_y:.3f}",
            f"{self.x:.3f}", f"{self.y:.3f}",
            f"{self.dx_cmd:.3f}", f"{self.dy_cmd:.3f}",
            f"{self.heading_deg:.2f}",
            f"{dt:.4f}",
            int(gps_correction),
            f"{ex:.3f}", f"{ey:.3f}", f"{e:.3f}",
            self.hl_cmd
        ])
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
