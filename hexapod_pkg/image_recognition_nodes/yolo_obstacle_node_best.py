#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os
from rclpy.qos import qos_profile_sensor_data
from hexapod_pkg import hw_config as cfg
import numpy as np

class YoloDebugNode(Node):
    def __init__(self):
        super().__init__('yolo_debug_node')

        self.bridge = CvBridge()

        # -------- PARÁMETROS --------
        self.declare_parameter('topic_image', 'camera/image_raw')
        self.declare_parameter('model', 'yolov8s.pt')
        self.declare_parameter('conf_thresh', 0.6)
        self.declare_parameter('frame_skip', 2)
        self.declare_parameter('debug_view', True)
        self.declare_parameter('topic_obstacle_occupation_grid', cfg.TOPIC_OBSTACLE_OCCUPATION_GRID)

        topic_image = self.get_parameter('topic_image').value
        topic_obstacle_occupation_grid = self.get_parameter("topic_obstacle_occupation_grid").value

        # -------- SUB / PUB --------
        self.sub_img = self.create_subscription(
            Image, topic_image, self.cb_img, qos_profile_sensor_data)

        self.pub_debug = self.create_publisher(
            String, '/yolo/debug_detections', 10)

        # -------- YOLO --------
        pkg_share = get_package_share_directory("hexapod_pkg")
        model_path = os.path.join(
            pkg_share,
            "utils",
            "cubitos.pt"
        )

        self.model = YOLO(model_path)
        self.model.to('cuda')

        self.frame_id = 0

        self.get_logger().info(f"Escuchando imágenes en: {topic_image}")
        self.get_logger().info(f"YOLO CUDA activo: {torch.cuda.is_available()}")

    def cb_img(self, msg):
        self.frame_id += 1
        if self.frame_id % self.get_parameter('frame_skip').value != 0:
            return

        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        with torch.no_grad():
            result = self.model(
                img,
                conf=self.get_parameter('conf_thresh').value,
                device='cuda',
                verbose=False
            )[0]

        detections = []

        for box in result.boxes:
            cls_id = int(box.cls[0])
            cls_name = self.model.names[cls_id]
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            detections.append(
                f"{cls_name} conf={conf:.2f} bbox=({x1},{y1},{x2},{y2})"
            )

            if self.get_parameter('debug_view').value:
                cv2.rectangle(img, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.putText(
                    img, f"{cls_name} {conf:.2f}",
                    (x1, y1-5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0,255,0), 1
                )

        # Publicar detecciones
        if detections:
            msg_out = " | ".join(detections)
        else:
            msg_out = "no detections"

        self.pub_debug.publish(String(data=msg_out))

        if self.get_parameter('debug_view').value:
            cv2.imshow("YOLO DEBUG", img)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = YoloDebugNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()