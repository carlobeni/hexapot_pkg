#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
import math
import random

class GPSToLocalXY(Node):
    def __init__(self):
        super().__init__("gps_to_local_xy")

        # --- Origen ---
        self.lat0 = math.radians(-25.330480)
        self.lon0 = math.radians(-57.518124)

        # Radio de la Tierra (WGS84)
        self.R = 6378137.0

        # --- Modelo de ruido ---
        self.gps_sigma = 2.5  # metros (1σ)

        self.sub = self.create_subscription(
            NavSatFix,
            "sensor/raw_data/gps",
            self.cb,
            10
        )

        self.pub = self.create_publisher(
            PointStamped,
            "/localization/gps/local_xy",
            10
        )

        self.get_logger().info(
            "GPS → Local XY con ruido artificial σ = 2.5 m"
        )

    def cb(self, msg: NavSatFix):
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return

        lat = math.radians(msg.latitude)
        lon = math.radians(msg.longitude)

        # Conversión local (plano tangente)
        x = self.R * (lon - self.lon0) * math.cos(self.lat0)
        y = self.R * (lat - self.lat0)

        # --- Ruido GPS (metros) ---
        x += random.gauss(0.0, self.gps_sigma)
        y += random.gauss(0.0, self.gps_sigma)

        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"

        out.point.x = x
        out.point.y = y
        out.point.z = msg.altitude  

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GPSToLocalXY()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()