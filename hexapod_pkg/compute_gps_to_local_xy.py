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

        # =====================================================
        # PARÁMETROS (CONFIGURABLE DESDE EL LAUNCHER)
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

        # Radio de la Tierra (WGS84)
        self.R = 6378137.0

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
            f"GPS → Local XY READY | sub: {gps_topic} → pub: {output_topic}"
        )
        self.get_logger().info(
            f"Origin lat={origin_lat_deg}, lon={origin_lon_deg}"
        )

    def cb(self, msg: NavSatFix):
        # Ignorar valores inválidos
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return

        lat = math.radians(msg.latitude)
        lon = math.radians(msg.longitude)

        # =====================================================
        # CONVERSIÓN LOCAL (APROX. MERCATOR)
        # =====================================================
        x = self.R * (lon - self.lon0) * math.cos(self.lat0)
        y = self.R * (lat - self.lat0)

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
