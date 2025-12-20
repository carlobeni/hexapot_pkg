#!/usr/bin/env python3
# compute_gps_to_local_xy.py
#
# GPS → sistema cartesiano local (Paraguay, WGS84)
#
# Convención:
#   x → Norte [m]
#   y → Este  [m]
#
# Incluye logging CSV para debugging
#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped

import math
import csv
import os
from datetime import datetime


class GPSToLocalXY(Node):
    def __init__(self):
        super().__init__("compute_gps_to_local_xy")

        # =====================================================
        # PARÁMETROS
        # =====================================================
        self.declare_parameter("gps_topic", "/sensor/raw_data/gps")
        self.declare_parameter("output_topic", "/localization/gps/local_xy")

        self.declare_parameter("origin_lat_deg", -25.330480)
        self.declare_parameter("origin_lon_deg", -57.518124)

        gps_topic = self.get_parameter("gps_topic").value
        output_topic = self.get_parameter("output_topic").value

        origin_lat_deg = self.get_parameter("origin_lat_deg").value
        origin_lon_deg = self.get_parameter("origin_lon_deg").value

        # =====================================================
        # ORIGEN (RADIANES)
        # =====================================================
        self.lat0 = math.radians(origin_lat_deg)
        self.lon0 = math.radians(origin_lon_deg)

        # WGS84
        self.R = 6378137.0  # [m]

        # =====================================================
        # ESTADO TIEMPO
        # =====================================================
        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        # =====================================================
        # CSV LOG
        # =====================================================
        log_dir = os.path.expanduser(
            "~/ros2_projects/ros2_hex_ws/src/hexapod_pkg/logs"
        )
        os.makedirs(log_dir, exist_ok=True)

        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_path = os.path.join(
            log_dir, f"gps_local_xy.csv"
        )

        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            "time_s",
            "lat_deg",
            "lon_deg",
            "x_north_m",
            "y_east_m",
            "alt_m"
        ])

        self.get_logger().info(f"GPS CSV logging → {self.csv_path}")

        # =====================================================
        # SUB / PUB
        # =====================================================
        self.sub = self.create_subscription(
            NavSatFix,
            gps_topic,
            self.cb,
            10
        )

        self.pub = self.create_publisher(
            PointStamped,
            output_topic,
            10
        )

        # =====================================================
        # LOG
        # =====================================================
        self.get_logger().info(
            "GPS → Local XY READY (Paraguay)"
        )
        self.get_logger().info(
            f"Origin lat={origin_lat_deg}, lon={origin_lon_deg}"
        )
        self.get_logger().info(
            "Axis: x=North, y=East [meters]"
        )

    # =====================================================
    # CALLBACK GPS
    # =====================================================
    def cb(self, msg: NavSatFix):
        # Ignorar GPS inválido
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return

        lat = math.radians(msg.latitude)
        lon = math.radians(msg.longitude)

        # =====================================================
        # CONVERSIÓN LOCAL (PLANO TANGENTE)
        # =====================================================
        x_north = self.R * (lat - self.lat0)
        y_east  = self.R * (lon - self.lon0) * math.cos(self.lat0)

        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9 - self.start_time

        # =====================================================
        # PUBLICAR
        # =====================================================
        out = PointStamped()
        out.header.stamp = now.to_msg()
        out.header.frame_id = "map"

        out.point.x = x_north
        out.point.y = y_east
        out.point.z = msg.altitude

        self.pub.publish(out)

        # =====================================================
        # CSV LOG
        # =====================================================
        self.csv_writer.writerow([
            f"{t:.3f}",
            f"{msg.latitude:.8f}",
            f"{msg.longitude:.8f}",
            f"{x_north:.3f}",
            f"{y_east:.3f}",
            f"{msg.altitude:.2f}"
        ])

        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPSToLocalXY()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
