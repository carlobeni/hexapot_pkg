#!/usr/bin/env python3
# compute_heading.py

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32

import math
import random
import csv
import os
from datetime import datetime


class HeadingEstimatorFast(Node):

    def __init__(self):
        super().__init__("compute_heading")

        # ================= PARÁMETROS =================
        self.declare_parameter("imu_topic", "/sensor/raw_data/imu")
        self.declare_parameter("mag_topic", "/sensor/raw_data/magnetometer")
        self.declare_parameter("output_topic", "/localization/heading_deg")

        self.declare_parameter("declination_deg", -15.5)
        self.declare_parameter("alpha", 0.95)
        self.declare_parameter("heading_noise_deg", 0.0)

        imu_topic = self.get_parameter("imu_topic").value
        mag_topic = self.get_parameter("mag_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.declination_rad = math.radians(
            self.get_parameter("declination_deg").value
        )
        self.alpha = float(self.get_parameter("alpha").value)

        noise_deg = self.get_parameter("heading_noise_deg").value
        self.heading_noise_rad = math.radians(noise_deg)

        # ================= ESTADO =================
        self.last_mag = None

        # ================= ROS =================
        self.create_subscription(
            Imu, imu_topic, self.imu_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            MagneticField, mag_topic, self.mag_cb, qos_profile_sensor_data
        )

        self.pub = self.create_publisher(
            Float32, output_topic, qos_profile_sensor_data
        )

        # ================= CSV =================
        log_dir = os.path.expanduser(
            "~/ros2_projects/ros2_hex_ws/src/hexapod_pkg/logs"
        )
        os.makedirs(log_dir, exist_ok=True)

        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_path = os.path.join(log_dir, f"heading_log.csv")

        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "time_s",
            "yaw_imu_deg",
            "mag_heading_deg",
            "fused_heading_deg"
        ])

        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info("HeadingEstimator READY")
        self.get_logger().info(
            "Convención: X=NORTE, Y=ESTE | CCW positivo | yaw sobre +Z"
        )
        self.get_logger().info(f"CSV → {self.csv_path}")

    # ================= CALLBACKS =================
    def mag_cb(self, msg: MagneticField):
        self.last_mag = msg.magnetic_field

    def imu_cb(self, msg: Imu):
        if self.last_mag is None:
            return

        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9 - self.start_time

        # --------- YAW IMU (rad, CCW +Z) ---------
        yaw_imu = self.quaternion_to_yaw(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )

        # --------- HEADING MAGNÉTICO ---------
        mag_heading = self.compute_mag_heading(self.last_mag)

        # --------- FUSIÓN ---------
        heading = self.alpha * yaw_imu + (1.0 - self.alpha) * mag_heading
        heading += self.declination_rad
        heading += random.gauss(0.0, self.heading_noise_rad)
        heading = self.wrap_pi(heading)

        # --------- A GRADOS [-180,180] ---------
        yaw_imu_deg = math.degrees(yaw_imu)
        mag_heading_deg = math.degrees(mag_heading)
        fused_deg = math.degrees(heading)

        self.pub.publish(Float32(data=float(fused_deg)))

        # --------- CSV ---------
        self.csv_writer.writerow([
            f"{t:.3f}",
            f"{yaw_imu_deg:.2f}",
            f"{mag_heading_deg:.2f}",
            f"{fused_deg:.2f}",
        ])
        self.csv_file.flush()

    # ================= MAG HEADING =================
    def compute_mag_heading(self, mag):
        """
        Heading absoluto:
        X = Norte
        Y = Este
        CCW positivo (yaw estándar)
        """
        north = mag.x
        east = mag.y
        return math.atan2(east, north)

    # ================= UTILIDADES =================
    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w*z + x*y)
        cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def wrap_pi(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HeadingEstimatorFast()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
