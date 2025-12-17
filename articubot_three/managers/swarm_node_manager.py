#!/usr/bin/env python3
import math
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3Stamped
from cv_bridge import CvBridge


class SwarmNodeManager(Node):

    def __init__(self):
        super().__init__('swarm_node_manager')

        # ==============================
        # PARÁMETROS
        # ==============================
        self.declare_parameter('image_source_topic', '/camera/image_raw')
        self.declare_parameter('compass_topic', '/imu/compass')
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('mag_topic', '/imu/mag')
        self.declare_parameter('cmd_vel_out_topic', '/cmd_vel')

        self.image_topic = self.get_parameter('image_source_topic').value
        self.compass_topic = self.get_parameter('compass_topic').value
        self.gps_topic = self.get_parameter('gps_topic').value
        self.mag_topic = self.get_parameter('mag_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_out_topic').value

        # ==============================
        # SUBSCRIPCIONES
        # ==============================
        self.sub_image = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )

        self.sub_compass = self.create_subscription(
            Vector3Stamped, self.compass_topic, self.compass_callback, 10
        )

        self.sub_gps = self.create_subscription(
            NavSatFix, self.gps_topic, self.gps_callback, 10
        )

        self.sub_mag = self.create_subscription(
            Vector3Stamped, self.mag_topic, self.mag_callback, 10
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # ==============================
        # VARIABLES
        # ==============================
        self.bridge = CvBridge()
        self.heading = 0.0
        self.last_position = None

        self.get_logger().info("SwarmNodeManager iniciado")

    # =====================================================
    # CALLBACKS SENSORES
    # =====================================================
    def compass_callback(self, msg):
        self.heading = math.atan2(msg.vector.y, msg.vector.x)

    def gps_callback(self, msg):
        self.last_position = (msg.latitude, msg.longitude)

    def mag_callback(self, msg):
        pass  # disponible si se requiere fusión futura

    # =====================================================
    # VISIÓN + ALGORITMO SWARM
    # =====================================================
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape
        cx_img = w // 2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Colores a detectar (HSV)
        color_ranges = {
            "red":    [(0,120,70), (10,255,255)],
            "green":  [(40,70,70), (80,255,255)],
            "blue":   [(100,150,50), (140,255,255)],
            "yellow": [(20,100,100), (30,255,255)],
            "purple": [(130,50,50), (160,255,255)],
        }

        detected = []

        for color, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for c in cnts:
                area = cv2.contourArea(c)
                if area < 300:
                    continue

                (x, y), radius = cv2.minEnclosingCircle(c)
                angle = math.atan2((x - cx_img), w / 2)
                distance = 1.0 / max(radius, 1.0)  # escala relativa

                detected.append((angle, distance))

        if not detected:
            self.stop_robot()
            return

        self.compute_and_publish_cmd(detected)

    # =====================================================
    # SWARM CORE
    # =====================================================
    def compute_and_publish_cmd(self, detections):
        vx = 0.0
        vy = 0.0

        for angle, dist in detections:
            vx += math.cos(angle) * dist
            vy += math.sin(angle) * dist

        vx /= len(detections)
        vy /= len(detections)

        target_angle = math.atan2(vy, vx)
        error_angle = self.normalize_angle(target_angle)

        cmd = Twist()
        cmd.linear.x = 0.2 * max(0.0, 1.0 - abs(error_angle))
        cmd.angular.z = 1.5 * error_angle

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = SwarmNodeManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
