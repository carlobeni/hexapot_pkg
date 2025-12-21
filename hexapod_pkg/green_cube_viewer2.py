#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int8MultiArray, MultiArrayDimension
from cv_bridge import CvBridge
import cv2
import numpy as np


class GreenCubeGrid(Node):
    def __init__(self):
        super().__init__('green_cube_grid')

        self.bridge = CvBridge()

        # ================= PARÁMETROS =================
        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('grid_size', 15)

        self.declare_parameter('h_min', 2)
        self.declare_parameter('h_max', 180)
        self.declare_parameter('s_min', 105)
        self.declare_parameter('v_min', 34)

        self.sub = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.cb_img,
            10
        )

        self.pub_grid = self.create_publisher(
            Int8MultiArray,
            '/green_cube/grid',
            10
        )

        self.get_logger().info("GreenCubeGrid (mask → grid) listo")

    def cb_img(self, msg):
        # ================= IMAGEN =================
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        H, W, _ = img.shape

        # ================= GRID =================
        N = self.get_parameter('grid_size').value
        cell_w = W / N
        cell_h = H / N

        grid = np.zeros((N, N), dtype=np.int8)

        # ================= HSV → MASK =================
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower = np.array([
            self.get_parameter('h_min').value,
            self.get_parameter('s_min').value,
            self.get_parameter('v_min').value
        ])
        upper = np.array([
            self.get_parameter('h_max').value,
            255, 255
        ])

        mask = cv2.inRange(hsv, lower, upper)

        # Limpieza mínima (opcional pero recomendada)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # ================= MASK → GRID DIRECTO =================
        for r in range(N):
            for c in range(N):
                y0 = int(r * cell_h)
                y1 = int((r + 1) * cell_h)
                x0 = int(c * cell_w)
                x1 = int((c + 1) * cell_w)

                cell = mask[y0:y1, x0:x1]

                # Celda ocupada si hay píxeles blancos
                if cv2.countNonZero(cell) > 0:
                    grid[r, c] = 1

        # ================= PUBLICAR =================
        msg_out = Int8MultiArray()
        msg_out.layout.dim = [
            MultiArrayDimension(label='rows', size=N, stride=N),
            MultiArrayDimension(label='cols', size=N, stride=1),
        ]
        msg_out.data = grid.flatten(order='C').tolist()

        self.pub_grid.publish(msg_out)

        # ================= DEBUG VISUAL =================
        for i in range(1, N):
            cv2.line(img, (0, int(i * cell_h)), (W, int(i * cell_h)), (255, 0, 0), 1)
            cv2.line(img, (int(i * cell_w), 0), (int(i * cell_w), H), (255, 0, 0), 1)

        cv2.imshow("GREEN CUBE GRID", img)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)


def main():
    rclpy.init()
    rclpy.spin(GreenCubeGrid())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
