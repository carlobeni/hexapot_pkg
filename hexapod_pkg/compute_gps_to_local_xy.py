#!/usr/bin/env python3
# compute_gps_to_local_xy.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
import math

class GPSToLocalXY(Node):
    def __init__(self):
        super().__init__("compute_gps_to_local_xy")

        # --- Origen (debe coincidir con tu world) ---
        self.lat0 = math.radians(-25.330480)
        self.lon0 = math.radians(-57.518124)

        # Radio de la Tierra (WGS84)
        self.R = 6378137.0

        self.sub = self.create_subscription(
            NavSatFix,
            "sensor/raw_data/gps",
            self.cb,
            10
        )
        self.pub = self.create_publisher(PointStamped, "/localization/gps/local_xy", 10)

        self.get_logger().info("GPS → Local XY node READY (Mercator local)")

    def cb(self, msg: NavSatFix):
        # Ignorar valores vacíos
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return

        lat = math.radians(msg.latitude)
        lon = math.radians(msg.longitude)

        # Conversión Mercator local
        x = self.R * (lon - self.lon0) * math.cos(self.lat0)
        y = self.R * (lat - self.lat0)

        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"

        out.point.x = x
        out.point.y = y
        out.point.z = msg.altitude

        self.pub.publish(out)

        #self.get_logger().info(f"XY=({x:.3f}, {y:.3f}) from lat={msg.latitude}, lon={msg.longitude}")


def main(args=None):
    rclpy.init(args=args)
    node = GPSToLocalXY()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()