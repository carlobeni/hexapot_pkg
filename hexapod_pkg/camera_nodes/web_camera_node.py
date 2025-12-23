#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from rclpy.qos import qos_profile_sensor_data


class WebcamNode(Node):
    def __init__(self):
        super().__init__('webcam_node')

        # ======================
        # Parámetros
        # ======================
        self.camera_index = 1
        self.width = 640
        self.height = 480
        self.fps = 15.0
        self.jpeg_quality = 80  # 60–90 recomendado

        # ======================
        # Publisher (compressed)
        # ======================
        self.image_pub = self.create_publisher(
            CompressedImage,
            'carlobeni/test/webcam/image/compressed',
            qos_profile_sensor_data
        )

        # ======================
        # OpenCV Camera
        # ======================
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().fatal(
                f"No se pudo abrir /dev/video{self.camera_index}"
            )
            raise RuntimeError("Camera open failed")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*'MJPG')
        )

        self.bridge = CvBridge()

        # ======================
        # Timer
        # ======================
        self.timer = self.create_timer(
            1.0 / self.fps,
            self.timer_callback
        )

        self.get_logger().info(
            f"Webcam compressed iniciado (/dev/video{self.camera_index})"
        )

    # ======================
    # Callback
    # ======================
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Frame no capturado")
            return

        # JPEG encode
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        success, encoded = cv2.imencode('.jpg', frame, encode_param)

        if not success:
            self.get_logger().warn("JPEG encode falló")
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_optical_frame'
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()

        self.image_pub.publish(msg)

    # ======================
    # Cleanup
    # ======================
    def destroy_node(self):
        if self.cap.isOpened():
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
