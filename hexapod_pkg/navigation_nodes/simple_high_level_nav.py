#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, String
from rclpy.qos import qos_profile_sensor_data

def normalize_angle_deg(a):
    """Normaliza ángulo a [-180, 180]."""
    while a > 180: a -= 360
    while a < -180: a += 360
    return a


class HighLevelNav(Node):
    def __init__(self):
        super().__init__("high_level_nav")

        # Subscripción a sensores
        self.sub_gps = self.create_subscription(
            PointStamped, "/localization/local_stimate_xy", self.cb_gps, 10)

        self.sub_heading = self.create_subscription(
            Float32, "/localization/heading_deg", self.cb_heading,
            qos_profile_sensor_data
        )

        # Publicación de comandos de alto nivel
        self.cmd_pub = self.create_publisher(String, "/hl_cmd", 10)

        # Objetivo fijo (puedes cambiarlo o hacer un servicio para definir nuevos)
        self.goal = (8.0, 6.0)  # ejemplo

        # Últimas lecturas
        self.x = None
        self.y = None
        self.heading = None  # en grados

        # Tolerancias
        self.heading_tol = 6   # grados
        self.dist_tol = 0.1      # metros

        self.timer = self.create_timer(0.05, self.loop)

    def cb_gps(self, msg: PointStamped):
        self.x = msg.point.x
        self.y = msg.point.y

    def cb_heading(self, msg: Float32):
        self.heading = float(msg.data)

    def loop(self):
        if None in (self.x, self.y, self.heading):
            return

        gx, gy = self.goal
        dx = gx - self.x
        dy = gy - self.y

        dist = math.hypot(dx, dy)
        desired_heading = math.degrees(math.atan2(dy, dx))

        err = normalize_angle_deg(desired_heading - self.heading)

        cmd = String()

        # 1 — Si ya estás en el objetivo
        if dist < self.dist_tol:
            cmd.data = "stop"
            self.cmd_pub.publish(cmd)
            return

        # 2 — Corregir orientación
        if err > self.heading_tol:
            cmd.data = "turn_left"
        elif err < -self.heading_tol:
            cmd.data = "turn_right"
        else:
            # 3 — Orientación correcta: avanzar
            cmd.data = "forward"

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = HighLevelNav()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
