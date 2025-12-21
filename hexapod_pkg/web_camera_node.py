#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data


class WebcamNode(Node):
    def __init__(self):
        super().__init__('webcam_node')

        # ======================
        # Parámetros (ajustables)
        # ======================
        self.camera_index = 0        # CAMBIA a 1, 2, 3 si no funciona
        self.width = 640
        self.height = 640
        self.fps = 15.0

        # ======================
        # Publisher
        # ======================
        self.image_pub = self.create_publisher(
            Image,
            '/webcam/image_raw',
            qos_profile_sensor_data
        )

        # ======================
        # OpenCV Camera (V4L2)
        # ======================
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().fatal(
                f"No se pudo abrir la cámara /dev/video{self.camera_index}"
            )
            raise RuntimeError("Camera open failed")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.bridge = CvBridge()

        # ======================
        # Timer
        # ======================
        self.timer = self.create_timer(
            1.0 / self.fps,
            self.timer_callback
        )

        self.get_logger().info(
            f"Webcam node iniciado usando /dev/video{self.camera_index}"
        )

    # ======================
    # Callback principal
    # ======================
    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("Frame no capturado")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_optical_frame'

        self.image_pub.publish(msg)

    # ======================
    # Cleanup seguro
    # ======================
    def destroy_node(self):
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = WebcamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
