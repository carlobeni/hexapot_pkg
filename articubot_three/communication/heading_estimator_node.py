#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import hw_config as cfg

# =============================
# CONSTANTES
# =============================
RAD_TO_DEG = 57.29578
DEG_TO_RAD = math.pi / 180.0

# Declinación magnética (Paraguay)
MAG_DECLINATION_DEG = -15.24

# =============================
# PARÁMETROS KALMAN (idénticos al script base)
# =============================
Q_angle = 0.02
Q_gyro  = 0.0015
R_angle = 0.005

# =============================
# ESTADOS KALMAN
# =============================
bias = 0.0
P_00 = 0.0
P_01 = 0.0
P_10 = 0.0
P_11 = 0.0
angle = 0.0


def kalman_heading(mag_angle, gyro_rate, dt):
    """
    mag_angle : heading por magnetómetro (deg)
    gyro_rate : velocidad angular z (deg/s)
    dt        : periodo de muestreo (s)
    """
    global angle, bias, P_00, P_01, P_10, P_11

    # Predicción
    angle += dt * (gyro_rate - bias)

    P_00 += -dt * (P_10 + P_01) + Q_angle * dt
    P_01 += -dt * P_11
    P_10 += -dt * P_11
    P_11 += Q_gyro * dt

    # Innovación
    y = mag_angle - angle
    S = P_00 + R_angle
    K0 = P_00 / S
    K1 = P_10 / S

    # Corrección
    angle += K0 * y
    bias  += K1 * y

    P_00 -= K0 * P_00
    P_01 -= K0 * P_01
    P_10 -= K1 * P_00
    P_11 -= K1 * P_01

    return angle


class HeadingEstimator(Node):

    def __init__(self):
        super().__init__("heading_estimator")

        self.qos_sensors = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # =============================
        # SUBSCRIBERS (DESDE LA PI)
        # =============================
        self.sub_imu = self.create_subscription(
            Imu,
            cfg.TOPIC_IMU_GIR_ACC,
            self.cb_imu,
            self.qos_sensors
        )

        self.sub_mag = self.create_subscription(
            MagneticField,
            cfg.TOPIC_IMU_MAG,
            self.cb_mag,
            self.qos_sensors
        )

        # =============================
        # PUBLISHERS (HEADING)
        # =============================
        self.pub_heading_mag = self.create_publisher(
            Float64,
            cfg.TOPIC_HEADING_COMPASS,
            self.qos_sensors
        )

        self.pub_heading_kalman = self.create_publisher(
            Float64,
            cfg.TOPIC_HEADING_COMPASS_KALMAN,
            self.qos_sensors
        )

        self.last_time = self.get_clock().now()
        self.last_mag_heading = None

        self.get_logger().info("Heading estimator iniciado (PC)")

    # --------------------------------------------------

    def cb_mag(self, msg):
        """
        Heading por magnetómetro (sin fusión)
        """
        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y

        heading_mag = math.atan2(my, mx) * RAD_TO_DEG
        if heading_mag < 0.0:
            heading_mag += 360.0

        self.last_mag_heading = heading_mag

        out = Float64()
        out.data = heading_mag
        self.pub_heading_mag.publish(out)

    # --------------------------------------------------

    def cb_imu(self, msg):
        """
        Heading fusionado (Kalman: gyro + mag)
        """
        if self.last_mag_heading is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0:
            return

        # Gyro Z en deg/s
        gyro_z_deg = msg.angular_velocity.z * RAD_TO_DEG

        # Kalman
        heading = kalman_heading(self.last_mag_heading, gyro_z_deg, dt)

        # Ajuste a norte verdadero
        heading += MAG_DECLINATION_DEG

        # Normalización
        if heading < 0.0:
            heading += 360.0
        elif heading >= 360.0:
            heading -= 360.0

        out = Float64()
        out.data = heading
        self.pub_heading_kalman.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = HeadingEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
