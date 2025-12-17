#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLO
import numpy as np


class ObstacleViewer(Node):
    def __init__(self):
        super().__init__('obstacle_viewer')

        self.bridge = CvBridge()

        # -------- PARÁMETROS --------
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('conf_thresh', 0.25)
        self.declare_parameter('frame_skip', 2)
        self.declare_parameter('debug_view', True)
        self.declare_parameter('grid_rows', 5)
        self.declare_parameter('grid_cols', 5)

        self.image_topic = self.get_parameter('image_topic').value
        self.grid_rows = self.get_parameter('grid_rows').value
        self.grid_cols = self.get_parameter('grid_cols').value

        # -------- ROS --------
        self.sub_img = self.create_subscription(
            Image, self.image_topic, self.cb_img, 10)

        self.pub_grid = self.create_publisher(
            String, '/local_obstacles_grid', 10)

        # -------- YOLO --------
        self.model = YOLO(self.get_parameter('model').value)
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(device)

        self.frame_id = 0
        self.get_logger().info(f"ObstacleViewer listo ({device})")

    def cb_img(self, msg):
        self.frame_id += 1
        if self.frame_id % self.get_parameter('frame_skip').value != 0:
            return

        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w, _ = img.shape

        grid = np.zeros((self.grid_rows, self.grid_cols), dtype=np.float32)

        cell_w = w / self.grid_cols
        cell_h = h / self.grid_rows

        with torch.no_grad():
            result = self.model(
                img,
                conf=self.get_parameter('conf_thresh').value,
                verbose=False
            )[0]

        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w - 1, x2), min(h - 1, y2)

            for r in range(self.grid_rows):
                for c in range(self.grid_cols):
                    cx1 = int(c * cell_w)
                    cy1 = int(r * cell_h)
                    cx2 = int((c + 1) * cell_w)
                    cy2 = int((r + 1) * cell_h)

                    ix1 = max(x1, cx1)
                    iy1 = max(y1, cy1)
                    ix2 = min(x2, cx2)
                    iy2 = min(y2, cy2)

                    if ix1 < ix2 and iy1 < iy2:
                        inter_area = (ix2 - ix1) * (iy2 - iy1)
                        cell_area = cell_w * cell_h
                        occ = inter_area / cell_area
                        grid[r, c] = min(1.0, grid[r, c] + occ)

            if self.get_parameter('debug_view').value:
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # -------- PUBLICAR GRID --------
        rows_str = []
        for r in range(self.grid_rows):
            row = ",".join(f"{grid[r, c]:.2f}" for c in range(self.grid_cols))
            rows_str.append(f"r{r}:{row}")

        msg = " | ".join(rows_str)
        self.pub_grid.publish(String(data=msg))

        # -------- DEBUG VISUAL --------
        if self.get_parameter('debug_view').value:
            for r in range(1, self.grid_rows):
                y = int(r * cell_h)
                cv2.line(img, (0, y), (w, y), (255, 0, 0), 1)
            for c in range(1, self.grid_cols):
                x = int(c * cell_w)
                cv2.line(img, (x, 0), (x, h), (255, 0, 0), 1)

            cv2.imshow("OBSTACLE GRID", img)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = ObstacleViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()