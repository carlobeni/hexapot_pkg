#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from rclpy.qos import qos_profile_sensor_data


class LocalNavPlanner(Node):
    def __init__(self):
        super().__init__("local_nav_planner")

        # -------- SUBSCRIPCIONES --------
        self.sub_grid = self.create_subscription(
            String, "/local_obstacles_grid", self.cb_grid, 10)

        self.sub_global = self.create_subscription(
            String, "/hl_global_cmd", self.cb_global, 10)

        self.sub_ultra = self.create_subscription(
            Float32, "/ultrasonic/range", self.cb_ultra,
            qos_profile_sensor_data)

        # -------- PUBLICACIÓN --------
        self.pub_cmd = self.create_publisher(String, "/hl_cmd", 10)

        # -------- ESTADO --------
        self.grid = None
        self.global_cmd = "stop"
        self.ultra = -1.0

        # -------- PARÁMETROS --------
        self.wall_thresh = 0.6
        self.ultra_critical = 0.6
        self.escape_steps = 0
        self.escape_dir = None

        # -------- BUG MODE --------
        self.bug_mode = False
        self.bug_phase = 0        # 0: back, 1: turn, 2: forward
        self.bug_steps = 0
        self.bug_dir = "turn_left"

        self.timer = self.create_timer(0.05, self.loop)

    # ================= CALLBACKS =================

    def cb_grid(self, msg):
        try:
            rows = msg.data.split("|")
            grid = []
            for r in rows:
                _, values = r.split(":")
                grid.append([float(v) for v in values.split(",")])
            self.grid = grid
        except Exception:
            self.get_logger().warn("Formato inválido en /local_obstacles_grid")

    def cb_global(self, msg):
        self.global_cmd = msg.data

    def cb_ultra(self, msg):
        self.ultra = msg.data

    # ================= LÓGICA =================

    def grid_empty(self):
        """YOLO no ve nada relevante"""
        return sum(sum(row) for row in self.grid) < 0.1

    def loop(self):
        if self.grid is None:
            return

        cmd = String()

        # ============================
        # 🐞 BUG MODE ACTIVO
        # ============================
        if self.bug_mode:
            self.bug_steps -= 1

            if self.bug_phase == 0:
                cmd.data = "backward"
                if self.bug_steps <= 0:
                    self.bug_phase = 1
                    self.bug_steps = 10

            elif self.bug_phase == 1:
                cmd.data = self.bug_dir
                if self.bug_steps <= 0:
                    self.bug_phase = 2
                    self.bug_steps = 12

            else:
                cmd.data = "forward"
                if self.bug_steps <= 0:
                    self.bug_phase = 0
                    self.bug_steps = 8

            # Salida del bug
            if self.ultra == -1 or self.ultra > self.ultra_critical:
                self.bug_mode = False

            self.pub_cmd.publish(cmd)
            return

        # ============================
        # 🐞 ENTRADA A BUG MODE
        # ============================
        if (
            self.ultra != -1
            and self.ultra < self.ultra_critical
            and self.grid_empty()
        ):
            self.get_logger().warn("🐞 BUG MODE ACTIVADO")
            self.bug_mode = True
            self.bug_phase = 0
            self.bug_steps = 8
            self.bug_dir = "turn_left"
            cmd.data = "backward"
            self.pub_cmd.publish(cmd)
            return

        # ============================
        # 1️⃣ ESCAPE ULTRASÓNICO NORMAL
        # ============================
        if self.escape_steps > 0:
            self.escape_steps -= 1
            cmd.data = self.escape_dir
            self.pub_cmd.publish(cmd)
            return

        if self.ultra != -1 and self.ultra < self.ultra_critical:
            left_free = sum(self.grid[4][0:2])
            right_free = sum(self.grid[4][3:5])
            self.escape_dir = "turn_left" if left_free < right_free else "turn_right"
            self.escape_steps = 15
            cmd.data = self.escape_dir
            self.pub_cmd.publish(cmd)
            return

        # ============================
        # 2️⃣ NAVEGACIÓN NORMAL VISUAL
        # ============================
        front_col = [self.grid[r][2] for r in range(5)]
        wall_ahead = all(v > self.wall_thresh for v in front_col)

        bottom = self.grid[4]
        left_score = bottom[0] + bottom[1]
        center_score = bottom[2]
        right_score = bottom[3] + bottom[4]

        if wall_ahead:
            cmd.data = "turn_left" if left_score < right_score else "turn_right"
        else:
            if self.global_cmd == "forward" and center_score < self.wall_thresh:
                cmd.data = "forward"
            elif self.global_cmd == "turn_left" and left_score < self.wall_thresh:
                cmd.data = "turn_left"
            elif self.global_cmd == "turn_right" and right_score < self.wall_thresh:
                cmd.data = "turn_right"
            else:
                if center_score <= min(left_score, right_score):
                    cmd.data = "forward"
                elif left_score < right_score:
                    cmd.data = "turn_left"
                else:
                    cmd.data = "turn_right"

        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = LocalNavPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

