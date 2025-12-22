#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, String, Int8MultiArray, Bool

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from hexapod_pkg import hw_config as cfg

# ==================================================
# CONFIGURACIÓN GPS (DEBE COINCIDIR CON EL WORLD)
# ==================================================
def gps_to_local_xy(lat, lon, lat_ref, lon_ref):
    """Convierte lat/lon a coordenadas locales ENU (X=East, Y=North)"""
    meters_per_deg = 111320.0
    dlat = lat - lat_ref
    dlon = lon - lon_ref

    x = dlon * meters_per_deg * math.cos(math.radians(lat_ref))
    y = dlat * meters_per_deg
    return x, y


def normalize_angle(a):
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a


class HighLevelNav(Node):
    def __init__(self):
        super().__init__("high_level_nav")

        self.declare_parameter("topic_ir_left", cfg.TOPIC_GZ_IR1)
        self.declare_parameter("topic_ir_right", cfg.TOPIC_GZ_IR2)
        self.declare_parameter("topic_estimate_heading", cfg.TOPIC_HEADING_COMPASS)
        self.declare_parameter("topic_ultrasonic_range", cfg.TOPIC_ULTRASONIC_RANGE)
        self.declare_parameter("topic_estimate_xy", cfg.TOPIC_XY_ODOM_CURRENT_POSITION)

        topic_ir_left = self.get_parameter("topic_ir_left").value
        topic_ir_right = self.get_parameter("topic_ir_right").value
        topic_estimate_heading = self.get_parameter("topic_estimate_heading").value
        topic_ultrasonic_range = self.get_parameter("topic_ultrasonic_range").value
        topic_estimate_xy = self.get_parameter("topic_estimate_xy").value


        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ================= SUBS =================
        self.create_subscription(
            PointStamped,
            topic_estimate_xy,
            self.cb_gps,
            qos
        )

        self.create_subscription(
            Float32,
            topic_estimate_heading,
            self.cb_heading,
            qos
        )

        self.create_subscription(
            Int8MultiArray,
            "/green_cube/grid",
            self.cb_grid,
            10
        )

        self.create_subscription(
            Float32,
            topic_ultrasonic_range,
            self.cb_ultra,
            qos
        )

        self.create_subscription(
            Bool,
            topic_ir_left,
            self.cb_ir_left,
            qos
        )

        self.create_subscription(
            Bool,
            topic_ir_right,
            self.cb_ir_right,
            qos
        )

        # ================= PUB =================
        self.cmd_pub = self.create_publisher(String,cfg.TOPIC_CMD_GZ_ROBOT, 10)

        # ================= OBJETIVO =================
        self.goal = self.read_goal_from_console()

        # ================= ESTADO =================
        self.x = self.y = self.heading = None
        self.grid = None
        self.N = None
        self.ultra_dist = None

        self.ir_left = False
        self.ir_right = False

        self.state = "NAV"
        self.prev_state = "NAV"
        self.heading_ref = None
        self.last_recover_time = 0.0

        # ================= PARÁMETROS =================
        self.heading_tol = 6.0
        self.dist_tol = 0.5

        self.heading_step = 2.0
        self.recover_period = 0.4

        self.ultra_clear_dist = 0.8

        self.clearance_angle = 25.0
        self.inflate_rows = 4
        self.inflate_cols = 3

        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info(
            f"Objetivo fijado en XY local: {self.goal}"
        )

    # ==================================================
    # === LECTURA DE OBJETIVO DESDE CONSOLA ===
    # ==================================================
    def read_goal_from_console(self):
        while True:
            try:
                raw = input(
                    "\nIngrese objetivo GPS (lat, lon): "
                )
                lat_str, lon_str = raw.split(",")
                lat = float(lat_str.strip())
                lon = float(lon_str.strip())

                gx, gy = gx, gy = gps_to_local_xy(
                                                    lat, lon,
                                                    -25.33074682110693,
                                                    -57.517799631180665
                                                )

                self.get_logger().info(
                    f"Objetivo GPS recibido: ({lat}, {lon})"
                )
                self.get_logger().info(
                    f"Convertido a XY local: ({gx:.2f}, {gy:.2f})"
                )

                return gx, gy

            except Exception as e:
                print("❌ Formato inválido. Use: lat, lon")
                print(e)

    # ================= CALLBACKS =================
    def cb_gps(self, msg):
        self.x = msg.point.x
        self.y = msg.point.y

    def cb_heading(self, msg):
        self.heading = float(msg.data)

    def cb_grid(self, msg):
        self.N = msg.layout.dim[0].size
        self.grid = list(msg.data)

    def cb_ultra(self, msg):
        self.ultra_dist = float(msg.data)

    def cb_ir_left(self, msg):
        self.ir_left = bool(msg.data)

    def cb_ir_right(self, msg):
        self.ir_right = bool(msg.data)

    # ================= PERCEPCIÓN =================
    def obstacle_detected(self):
        if self.grid is None:
            return False

        N = self.N
        mid = N // 2

        for r in range(N - self.inflate_rows, N):
            for c in range(mid - self.inflate_cols, mid + self.inflate_cols + 1):
                if self.grid[r * N + c] == 1:
                    return True

        if self.ultra_dist and self.ultra_dist < self.ultra_clear_dist:
            return True

        return False

    def grid_density(self):
        if self.grid is None:
            return 0, 0

        N = self.N
        mid = N // 2

        left = right = 0
        for r in range(mid, N):
            for c in range(0, mid):
                left += self.grid[r * N + c]
            for c in range(mid, N):
                right += self.grid[r * N + c]

        return left, right

    # ================= LOOP =================
    def loop(self):
        if None in (self.x, self.y, self.heading):
            return

        cmd = String()

        # === REFLEJO IR LATERAL ===
        if self.ir_left or self.ir_right:
            self.state = "CRAB_RIGHT" if self.ir_left else "CRAB_LEFT"
            cmd.data = "lateral_right" if self.ir_left else "lateral_left"
            self.cmd_pub.publish(cmd)
            return

        gx, gy = self.goal
        dx = gx - self.x
        dy = gy - self.y

        dist = math.hypot(dx, dy)
        if dist < self.dist_tol:
            cmd.data = "stop"
            self.cmd_pub.publish(cmd)
            return

        heading_goal = normalize_angle(
            math.degrees(math.atan2(dy, dx))
        )

        err = normalize_angle(heading_goal - self.heading)
        cmd.data = (
            "turn_left" if err > self.heading_tol else
            "turn_right" if err < -self.heading_tol else
            "forward"
        )

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(HighLevelNav())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
