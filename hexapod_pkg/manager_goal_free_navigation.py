#!/usr/bin/env python3

import math
import csv
import os

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, String
from rclpy.qos import qos_profile_sensor_data


def wrap_360(angle_deg: float) -> float:
    """Normaliza ángulo a [0, 360)."""
    a = angle_deg % 360.0
    if a < 0.0:
        a += 360.0
    return a


class ManagerGoalFreeNavigation(Node):

    def __init__(self):
        super().__init__("manager_goal_free_navigation")

        # ================= PARÁMETROS =================
        self.declare_parameter("reference_lat", -25.330480)
        self.declare_parameter("reference_lon", -57.518124)

        self.declare_parameter("goal_lat", -25.330300)
        self.declare_parameter("goal_lon", -57.518000)

        self.declare_parameter("epsilon", 0.5)        # [m]
        self.declare_parameter("heading_tol", 6.0)    # [deg]
        self.declare_parameter("time_forward", 0.8)  # [s]

        self.declare_parameter("current_position_xy", "/pc/internal/xy_odom_current_position")
        self.declare_parameter("heading_deg_topic", "/pc/internal/heading_mag")
        self.declare_parameter("cmd_robot_topic", "/cmd_robot")

        # ================= LECTURA =================
        self.ref_lat = math.radians(self.get_parameter("reference_lat").value)
        self.ref_lon = math.radians(self.get_parameter("reference_lon").value)

        self.goal_lat = math.radians(self.get_parameter("goal_lat").value)
        self.goal_lon = math.radians(self.get_parameter("goal_lon").value)

        self.epsilon = float(self.get_parameter("epsilon").value)
        self.heading_tol = float(self.get_parameter("heading_tol").value)
        self.time_forward = float(self.get_parameter("time_forward").value)

        self.pos_topic = self.get_parameter("current_position_xy").value
        self.heading_topic = self.get_parameter("heading_deg_topic").value
        self.cmd_topic = self.get_parameter("cmd_robot_topic").value

        self.R = 6378137.0  # WGS84

        # ================= ESTADO =================
        self.x = None           # NORTE
        self.y = None           # ESTE
        self.heading = None     # [0,360)

        self.start_x = None
        self.start_y = None
        self.desired_heading = None  # ABSOLUTO respecto al Norte

        self.gps_ready = False

        self.forward_locked = False
        self.forward_start_time = 0.0

        # Conversión del objetivo
        self.goal_x, self.goal_y = self.latlon_to_xy(self.goal_lat, self.goal_lon)

        # ================= ROS =================
        self.create_subscription(
            PointStamped,
            self.pos_topic,
            self.cb_position,
            10
        )

        self.create_subscription(
            Float32,
            self.heading_topic,
            self.cb_heading,
            qos_profile_sensor_data
        )

        self.cmd_pub = self.create_publisher(
            String,
            self.cmd_topic,
            10
        )

    # ================= GEO → XY =================
    def latlon_to_xy(self, lat, lon):
        """
        Conversión local:
        x = Norte
        y = Este
        """
        x_north = self.R * (lat - self.ref_lat)
        y_east  = self.R * (lon - self.ref_lon) * math.cos(self.ref_lat)
        return x_north, y_east

    # ================= CALLBACKS =================
    def cb_position(self, msg: PointStamped):
        self.x = msg.point.x
        self.y = msg.point.y

        # Inicialización UNA SOLA VEZ
        if not self.gps_ready:
            self.start_x = self.x
            self.start_y = self.y

            dx = self.goal_x - self.start_x
            dy = self.goal_y - self.start_y

            self.desired_heading = wrap_360(math.degrees(math.atan2(dy, dx)))
            self.gps_ready = True

            self.get_logger().info(
                f"Inicio → desired_heading = {self.desired_heading:.2f}°"
            )

    def cb_heading(self, msg: Float32):
        self.heading = wrap_360(float(msg.data))

    # ================= LOOP =================
    def loop(self):
        if not self.gps_ready or self.heading is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = math.hypot(dx, dy)

        # Error angular absoluto [0,360)
        err = wrap_360(self.desired_heading - self.heading)

        # ================= CONTROL =================
        if dist <= self.epsilon:
            cmd = "stop"
            self.forward_locked = False

        elif self.forward_locked:
            if now - self.forward_start_time < self.time_forward:
                cmd = "forward"
            else:
                self.forward_locked = False
                cmd = "forward"

        else:
            # Giro mínimo
            if err > self.heading_tol and err <= 180.0:
                cmd = "turn_left"
            elif err > 180.0 and (360.0 - err) > self.heading_tol:
                cmd = "turn_right"
            else:
                cmd = "forward"
                self.forward_locked = True
                self.forward_start_time = now

        self.cmd_pub.publish(String(data=cmd))

        # ================= CSV =================
        t = now - self.start_time

        self.csv_writer.writerow([
            f"{t:.3f}",
            f"{self.x:.3f}", f"{self.y:.3f}",
            f"{self.goal_x:.3f}", f"{self.goal_y:.3f}",
            f"{self.desired_heading:.2f}",
            f"{self.heading:.2f}",
            f"{err:.2f}",
            f"{dist:.3f}",
            int(self.forward_locked),
            cmd
        ])

        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ManagerGoalFreeNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
