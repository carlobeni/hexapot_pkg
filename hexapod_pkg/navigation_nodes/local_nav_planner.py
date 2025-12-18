#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class LocalNavVision(Node):
    def __init__(self):
        super().__init__('local_nav_vision')

        # -------- ROS ----------
        self.bridge = CvBridge()

        self.sub_img = self.create_subscription(
            Image, '/camera/image_raw', self.cb_img, 10)

        self.sub_global = self.create_subscription(
            String, '/hl_cmd_global', self.cb_global, 10)

        self.pub_cmd = self.create_publisher(
            String, '/hl_cmd', 10)

        # -------- Parámetros (calibrables en runtime) ----------
        self.declare_parameter('roi_start', 0.55)       # % inferior
        self.declare_parameter('edge_thresh', 0.03)     # densidad
        self.declare_parameter('smooth_alpha', 0.7)     # filtro
        self.declare_parameter('debug_view', False)

        # -------- Estado ----------
        self.global_cmd = "stop"

        self.e_left_f = 0.0
        self.e_front_f = 0.0
        self.e_right_f = 0.0

        self.state = "CLEAR"   # CLEAR | AVOID

    # =========================================================
    # CALLBACKS
    # =========================================================
    def cb_global(self, msg):
        self.global_cmd = msg.data

    def cb_img(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        h, w = gray.shape

        roi_start = self.get_parameter('roi_start').value
        y0 = int(h * roi_start)

        roi = gray[y0:h, :]

        # Regiones
        left  = roi[:, :w//3]
        front = roi[:, w//3:2*w//3]
        right = roi[:, 2*w//3:]

        # Densidad de bordes
        e_left  = self.edge_density(left)
        e_front = self.edge_density(front)
        e_right = self.edge_density(right)

        # Filtro exponencial
        alpha = self.get_parameter('smooth_alpha').value
        self.e_left_f  = alpha*self.e_left_f  + (1-alpha)*e_left
        self.e_front_f = alpha*self.e_front_f + (1-alpha)*e_front
        self.e_right_f = alpha*self.e_right_f + (1-alpha)*e_right

        self.decide()

        # Debug
        if self.get_parameter('debug_view').value:
            self.debug_visual(img, roi, w, h)

    # =========================================================
    # PERCEPCIÓN
    # =========================================================
    def edge_density(self, img):
        edges = cv2.Canny(img, 50, 150)
        return np.sum(edges > 0) / edges.size

    # =========================================================
    # FSM + DECISIÓN
    # =========================================================
    def decide(self):
        T = self.get_parameter('edge_thresh').value

        # Clasificación del obstáculo
        if self.e_front_f > T or (self.e_left_f > T and self.e_right_f > T):
            obstacle = "front"
        elif self.e_left_f > T:
            obstacle = "left"
        elif self.e_right_f > T:
            obstacle = "right"
        else:
            obstacle = "none"

        cmd = String()

        # ---------- FSM ----------
        if self.state == "CLEAR":
            if obstacle == "none":
                cmd.data = self.global_cmd
            else:
                self.state = "AVOID"
                cmd.data = self.avoid_cmd(obstacle)

        elif self.state == "AVOID":
            if obstacle == "none":
                self.state = "CLEAR"
                cmd.data = self.global_cmd
            else:
                cmd.data = self.avoid_cmd(obstacle)

        self.pub_cmd.publish(cmd)

        # Log útil (no spamea demasiado)
        self.get_logger().info(
            f"STATE={self.state} "
            f"E[L,F,R]=({self.e_left_f:.3f}, "
            f"{self.e_front_f:.3f}, "
            f"{self.e_right_f:.3f}) "
            f"CMD={cmd.data}"
        )

    def avoid_cmd(self, obstacle):
        if obstacle == "front":
            return "turn_left"
        elif obstacle == "left":
            return "turn_right"
        elif obstacle == "right":
            return "turn_left"
        return "stop"

    # =========================================================
    # DEBUG VISUAL
    # =========================================================
    def debug_visual(self, img, roi, w, h):
        vis = img.copy()
        y0 = h - roi.shape[0]

        cv2.line(vis, (w//3, y0), (w//3, h), (0,255,0), 2)
        cv2.line(vis, (2*w//3, y0), (2*w//3, h), (0,255,0), 2)
        cv2.rectangle(vis, (0, y0), (w, h), (255,0,0), 2)

        edges = cv2.Canny(roi, 50, 150)

        cv2.imshow("LocalNavVision - ROI", roi)
        cv2.imshow("LocalNavVision - Edges", edges)
        cv2.imshow("LocalNavVision - View", vis)
        cv2.waitKey(1)


# =============================================================
def main():
    rclpy.init()
    node = LocalNavVision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
