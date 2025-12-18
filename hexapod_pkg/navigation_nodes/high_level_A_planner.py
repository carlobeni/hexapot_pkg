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


# ===============================
#    A* PATH PLANNER 2D
# ===============================
class AStarPlanner:
    def __init__(self, step=2):
        self.step = step  # distancia entre nodos

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def get_neighbors(self, node):
        s = self.step
        x, y = node
        return [
            (x+s, y), (x-s, y), (x, y+s), (x, y-s),
            (x+s, y+s), (x+s, y-s), (x-s, y+s), (x-s, y-s)
        ]

    def a_star(self, start, goal):
        open_set = {start}
        came_from = {}
        g = {start: 0}
        f = {start: self.heuristic(start, goal)}

        while open_set:
            current = min(open_set, key=lambda n: f[n])

            if self.heuristic(current, goal) < self.step:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            open_set.remove(current)

            for nb in self.get_neighbors(current):
                tentative_g = g[current] + self.heuristic(current, nb)

                if nb not in g or tentative_g < g[nb]:
                    g[nb] = tentative_g
                    f[nb] = tentative_g + self.heuristic(nb, goal)
                    came_from[nb] = current
                    open_set.add(nb)

        return []


# ============================================================
#       HIGH LEVEL NAVIGATION WITH A* WAYPOINTS
# ============================================================
class HighLevelNav(Node):
    def __init__(self):
        super().__init__("high_level_nav")

        self.sub_gps = self.create_subscription(
            PointStamped, "/localization/local_stimate_xy", self.cb_gps, 10)

        self.sub_heading = self.create_subscription(
            Float32, "/localization/heading_deg", self.cb_heading,
            qos_profile_sensor_data
        )


        self.cmd_pub = self.create_publisher(String, "/hl_global_cmd", 10)

        self.goal = (8.0, 6.0)

        self.x = None
        self.y = None
        self.heading = None

        self.heading_tol = 5
        self.dist_tol = 0.3

        self.step = 2
        self.planner = AStarPlanner(step=self.step)

        self.path = []
        self.current_wp_index = 0

        # NODO LENTO: 2 Hz
        self.timer = self.create_timer(0.5, self.loop)

    def cb_gps(self, msg):
        self.x = msg.point.x
        self.y = msg.point.y

    def cb_heading(self, msg):
        self.heading = float(msg.data)

    def plan_path_if_needed(self):
        if not self.path:
            start = (round(self.x, 2), round(self.y, 2))
            goal = self.goal
            self.get_logger().info(f"Planificando A* desde {start} a {goal}...")
            self.path = self.planner.a_star(start, goal)
            self.current_wp_index = 0
            self.get_logger().info(f"Ruta generada ({len(self.path)} nodos): {self.path}")

    def loop(self):
        if None in (self.x, self.y, self.heading):
            return

        self.plan_path_if_needed()

        if not self.path:
            cmd = String()
            cmd.data = "stop"
            self.cmd_pub.publish(cmd)
            return

        if self.current_wp_index >= len(self.path):
            cmd = String()
            cmd.data = "stop"
            self.cmd_pub.publish(cmd)
            return

        wx, wy = self.path[self.current_wp_index]

        dx = wx - self.x
        dy = wy - self.y
        dist = math.hypot(dx, dy)

        desired_heading = math.degrees(math.atan2(dy, dx))
        err = normalize_angle_deg(desired_heading - self.heading)

        cmd = String()

        if dist < self.dist_tol:
            self.current_wp_index += 1
            return

        if err > self.heading_tol:
            cmd.data = "turn_left"
        elif err < -self.heading_tol:
            cmd.data = "turn_right"
        else:
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
