#!/usr/bin/env python3
# manager_goal_free_navigation.py

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker
from rclpy.qos import qos_profile_sensor_data


def normalize_angle_deg(a):
    while a > 180: a -= 360
    while a < -180: a += 360
    return a


class ManagerGoalFreeNavigation(Node):

    def __init__(self):
        super().__init__("manager_goal_free_navigation")

        # ================= PARÁMETROS =================
        self.declare_parameter("reference_lat", -25.330480)
        self.declare_parameter("reference_lon", -57.518124)

        self.declare_parameter("goal_lat", -25.330300)
        self.declare_parameter("goal_lon", -57.518000)

        self.declare_parameter("epsilon", 0.5)
        self.declare_parameter("heading_tol", 6.0)

        self.declare_parameter("cmd_robot_topic", "/cmd_robot")

        # ================= LECTURA =================
        self.ref_lat = math.radians(self.get_parameter("reference_lat").value)
        self.ref_lon = math.radians(self.get_parameter("reference_lon").value)

        self.goal_lat = math.radians(self.get_parameter("goal_lat").value)
        self.goal_lon = math.radians(self.get_parameter("goal_lon").value)

        self.epsilon = float(self.get_parameter("epsilon").value)
        self.heading_tol = float(self.get_parameter("heading_tol").value)

        self.cmd_robot_topic = self.get_parameter("cmd_robot_topic").value

        self.R = 6378137.0

        # ================= ESTADO =================
        self.x = None
        self.y = None
        self.heading = None

        self.start_x = None
        self.start_y = None
        self.gps_ready = False

        self.ref_x, self.ref_y = self.latlon_to_xy(self.ref_lat, self.ref_lon)
        self.goal_x, self.goal_y = self.latlon_to_xy(self.goal_lat, self.goal_lon)

        # ================= ROS =================
        self.sub_pos = self.create_subscription(
            PointStamped,
            "/localization/local_stimate_xy",
            self.cb_position,
            10
        )

        self.sub_heading = self.create_subscription(
            Float32,
            "/localization/heading_deg",
            self.cb_heading,
            qos_profile_sensor_data
        )

        self.cmd_pub = self.create_publisher(
            String,
            self.cmd_robot_topic,
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            "/goal_nav/markers",
            10
        )

        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info(
            f"Manager READY → cmd_topic={self.cmd_robot_topic}"
        )

    # ================= GEO → XY =================
    def latlon_to_xy(self, lat, lon):
        x = self.R * (lon - self.ref_lon) * math.cos(self.ref_lat)
        y = self.R * (lat - self.ref_lat)
        return x, y

    # ================= CALLBACKS =================
    def cb_position(self, msg):
        self.x = msg.point.x
        self.y = msg.point.y

        if not self.gps_ready:
            self.start_x = self.x
            self.start_y = self.y
            self.gps_ready = True

    def cb_heading(self, msg):
        self.heading = msg.data

    # ================= LOOP =================
    def loop(self):
        if not self.gps_ready or self.heading is None:
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = math.hypot(dx, dy)

        desired = math.degrees(math.atan2(dy, dx))
        err = normalize_angle_deg(desired - self.heading)

        cmd = String()

        if dist <= self.epsilon:
            cmd.data = "stop"
        elif err > self.heading_tol:
            cmd.data = "turn_left"
        elif err < -self.heading_tol:
            cmd.data = "turn_right"
        else:
            cmd.data = "forward"

        self.cmd_pub.publish(cmd)
        self.publish_markers(dist)

    # ================= RVIZ MARKERS =================
    def publish_markers(self, dist):
        t = self.get_clock().now().to_msg()

        def sphere(mid, x, y, r, color):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = t
            m.ns = "goal_nav"
            m.id = mid
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.0
            m.scale.x = r
            m.scale.y = r
            m.scale.z = r
            m.color.r, m.color.g, m.color.b, m.color.a = (*color, 1.0)
            return m

        # Reference
        self.marker_pub.publish(sphere(0, self.ref_x, self.ref_y, 0.2, (0,0,0)))
        # Start
        self.marker_pub.publish(sphere(1, self.start_x, self.start_y, 0.2, (0,1,0)))
        # Goal
        self.marker_pub.publish(sphere(2, self.goal_x, self.goal_y, 0.2, (1,0,0)))
        # Current
        self.marker_pub.publish(sphere(3, self.x, self.y, 0.2, (0,0,1)))

        # Epsilon circle
        circle = Marker()
        circle.header.frame_id = "map"
        circle.header.stamp = t
        circle.ns = "goal_nav"
        circle.id = 4
        circle.type = Marker.LINE_STRIP
        circle.scale.x = 0.03
        circle.color.r = 1.0
        circle.color.a = 1.0

        for i in range(0, 361, 5):
            p = Point()
            p.x = self.goal_x + self.epsilon * math.cos(math.radians(i))
            p.y = self.goal_y + self.epsilon * math.sin(math.radians(i))
            circle.points.append(p)

        self.marker_pub.publish(circle)


def main(args=None):
    rclpy.init(args=args)
    node = ManagerGoalFreeNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
