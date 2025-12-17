#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import mediapipe as mp
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


# =========================================================
# === CONSTANTES DE LANDMARKS
# =========================================================
PALMA_MANO = [0, 1, 2, 5, 9, 13, 17]


# =========================================================
# === FUNCIONES GEOMÉTRICAS
# =========================================================
def distance(a, b):
    return np.linalg.norm(
        np.array([a.x - b.x, a.y - b.y, a.z - b.z])
    )


def palm_center(landmarks):
    pts = [landmarks[i] for i in PALMA_MANO]
    c = np.mean([[p.x, p.y, p.z] for p in pts], axis=0)

    class P:
        pass

    p = P()
    p.x, p.y, p.z = c
    return p


# =========================================================
# === DETECTORES DE GESTOS
# =========================================================
def detect_al_pelo(landmarks, max_angle_deg=30):
    palma_pts = np.array([[landmarks[i].x, landmarks[i].y] for i in PALMA_MANO])
    pc = palma_pts.mean(axis=0)

    def dist(i):
        p = landmarks[i]
        return np.linalg.norm(np.array([p.x, p.y]) - pc)

    # Pulgar extendido
    if not (dist(4) > dist(3) > dist(2) and dist(4) > 0.10):
        return False

    # Pulgar arriba de la palma
    if landmarks[4].y >= np.min(palma_pts[:, 1]):
        return False

    # Ángulo del pulgar
    x2, y2 = landmarks[2].x, landmarks[2].y
    x4, y4 = landmarks[4].x, landmarks[4].y
    dx = x4 - x2
    dy = y2 - y4

    if dy <= 0:
        return False

    angle = np.degrees(np.arctan(abs(dx) / dy))
    return angle <= max_angle_deg


def detect_victory(landmarks):
    pc = palm_center(landmarks)
    return (
        distance(landmarks[8], pc) > 0.12 and
        distance(landmarks[12], pc) > 0.12 and
        distance(landmarks[16], pc) < 0.10 and
        distance(landmarks[20], pc) < 0.10
    )


def detect_rock(landmarks):
    palma_pts = np.array([[landmarks[i].x, landmarks[i].y] for i in PALMA_MANO])
    pc = palma_pts.mean(axis=0)

    def dist(i):
        p = landmarks[i]
        return np.linalg.norm(np.array([p.x, p.y]) - pc)

    # Índice y meñique extendidos
    if not (dist(8) > 0.12 and dist(20) > 0.12):
        return False

    # Medio y anular doblados
    if not (dist(12) < 0.10 and dist(16) < 0.10):
        return False

    # Pulgar: punta fuera de la palma
    palm_radius = np.max([dist(i) for i in PALMA_MANO])
    return dist(4) > palm_radius


def hand_expression(landmarks):
    if detect_rock(landmarks):
        return "Rock"
    if detect_al_pelo(landmarks):
        return "Al pelo"
    if detect_victory(landmarks):
        return "Victoria"
    return None


# =========================================================
# === NODO ROS 2
# =========================================================
class HandGestureNode(Node):
    def __init__(self):
        super().__init__("hand_gesture_node")

        self.publisher = self.create_publisher(String, "/social_cmd", 10)
        self.subscription = self.create_subscription(
            Image,
            "/image_raw",
            self.image_callback,
            10
        )

        self.bridge = CvBridge()
        self.last_gesture = None
        self.timestamp = 0

        # -------- MediaPipe --------
        BaseOptions = mp.tasks.BaseOptions
        HandLandmarker = mp.tasks.vision.HandLandmarker
        HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
        VisionRunningMode = mp.tasks.vision.RunningMode

        pkg_share = get_package_share_directory("phantom_uttubot")
        model_path = os.path.join(
            pkg_share,
            "scripts",
            "hand_landmarker.task"
        )

        options = HandLandmarkerOptions(
            base_options=BaseOptions(
                model_asset_path=model_path
            ),
            running_mode=VisionRunningMode.VIDEO,
            num_hands=1
        )

        self.detector = HandLandmarker.create_from_options(options)

        self.get_logger().info("Nodo de gestos iniciado (suscrito a /image_raw)")


    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        # MediaPipe requiere timestamp monótono en ms
        self.timestamp += 33

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=rgb
        )

        result = self.detector.detect_for_video(
            mp_image,
            self.timestamp
        )

        gesture = None

        if result.hand_landmarks:
            gesture = hand_expression(result.hand_landmarks[0])

        # ===============================
        # === PUBLICACIÓN DE EVENTOS
        # ===============================
        if gesture is not None:
            if gesture != self.last_gesture:
                msg_out = String()
                msg_out.data = gesture
                self.publisher.publish(msg_out)
                self.get_logger().info(f"/social_cmd → {gesture}")
                self.last_gesture = gesture

# =========================================================
# === MAIN
# =========================================================
def main():
    rclpy.init()
    node = HandGestureNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

