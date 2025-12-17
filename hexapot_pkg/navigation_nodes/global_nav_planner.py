#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, String
from rclpy.qos import qos_profile_sensor_data


def normalize_angle_deg(a):
    while a > 180: a -= 360
    while a < -180: a += 360
    return a


class GlobalNavPlanner(Node):
    def __init__(self):
        super().__init__("global_nav_planner")

        self.sub_gps = self.create_subscription(
            PointStamped, "/localization/local_stimate_xy", self.cb_gps, 10)

        self.sub_heading = self.create_subscription(
            Float32, "/localization/heading_deg", self.cb_heading,
            qos_profile_sensor_data)

        self.pub_cmd = self.create_publisher(
            String, "/hl_global_cmd", 10)

        self.goal = (8.0, 6.0)

        self.x = None
        self.y = None
        self.heading = None

        self.heading_tol = 5
        self.dist_tol = 0.3

        self.timer = self.create_timer(0.5, self.loop)

    def cb_gps(self, msg):
        self.x = msg.point.x
        self.y = msg.point.y

    def cb_heading(self, msg):
        self.heading = float(msg.data)

    def loop(self):
        if None in (self.x, self.y, self.heading):
            return

        dx = self.goal[0] - self.x
        dy = self.goal[1] - self.y
        dist = math.hypot(dx, dy)

        desired_heading = math.degrees(math.atan2(dy, dx))
        err = normalize_angle_deg(desired_heading - self.heading)

        cmd = String()

        if dist < self.dist_tol:
            cmd.data = "stop"
        elif err > self.heading_tol:
            cmd.data = "turn_left"
        elif err < -self.heading_tol:
            cmd.data = "turn_right"
        else:
            cmd.data = "forward"

        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = GlobalNavPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
