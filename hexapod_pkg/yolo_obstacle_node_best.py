#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

import cv2
import numpy as np
import torch
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os


class YoloDebugNode(Node):
    def __init__(self):
        super().__init__('yolo_debug_node')

        # -------- PARÁMETROS --------
        self.declare_parameter(
            'image_topic',
            '/carlobeni/test/webcam/image/compressed'
        )
        self.declare_parameter('model', 'best_custom.pt')
        self.declare_parameter('conf_thresh', 0.6)
        self.declare_parameter('frame_skip', 2)
        self.declare_parameter('debug_view', True)

        image_topic = self.get_parameter('image_topic').value

        # -------- SUB / PUB --------
        self.sub_img = self.create_subscription(
            CompressedImage,
            image_topic,
            self.cb_img,
            qos_profile_sensor_data
        )

        self.pub_debug = self.create_publisher(
            String,
            '/yolo/debug_detections',
            10
        )

        # -------- YOLO --------
        pkg_share = get_package_share_directory("hexapod_pkg")
        model_path = os.path.join(
            pkg_share,
            "utils",
            self.get_parameter('model').value
        )

        print(model_path)

        self.model = YOLO(model_path)
        self.model.to('cuda')

        self.frame_id = 0

        self.get_logger().info(f"Escuchando imágenes comprimidas en: {image_topic}")
        self.get_logger().info(f"YOLO CUDA activo: {torch.cuda.is_available()}")

    # ======================
    # Callback imagen
    # ======================
    def cb_img(self, msg: CompressedImage):
        self.frame_id += 1
        if self.frame_id % self.get_parameter('frame_skip').value != 0:
            return

        # ---------- DECOMPRESIÓN JPEG ----------
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if img is None:
            self.get_logger().warn("No se pudo decodificar la imagen JPEG")
            return

        # ---------- YOLO ----------
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
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    img,
                    f"{cls_name} {conf:.2f}",
                    (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1
                )

        # ---------- PUBLICAR TEXTO ----------
        if detections:
            msg_out = " | ".join(detections)
        else:
            msg_out = "no detections"

        self.pub_debug.publish(String(data=msg_out))

        # ---------- DEBUG VISUAL ----------
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
