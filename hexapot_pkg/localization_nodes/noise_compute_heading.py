#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32
import random
import math


class HeadingEstimatorFast(Node):

    def __init__(self):
        super().__init__('heading_estimator_fast')

        # ================= PARÁMETROS =================
        self.declination_deg = -15.5       # declinación magnética
        self.heading_gain   = 1.125        # corrección de escala
        self.alpha          = 0.95         # 1=solo IMU, 0=solo MAG

        self.swap_xy        = False
        self.invert_mag_x   = False
        self.invert_mag_y   = False

        self.declination_rad = math.radians(self.declination_deg)

        # ================= RUIDO ARTIFICIAL =================
        self.heading_noise_deg = 1.5
        self.heading_noise_rad = math.radians(self.heading_noise_deg)

        # ================= ESTADO =====================
        self.last_mag = None

        # ================= ROS ========================
        self.create_subscription(
            Imu,
            'sensor/raw_data/imu',
            self.imu_cb,
            qos_profile_sensor_data
        )

        self.create_subscription(
            MagneticField,
            'sensor/raw_data/magnetometer',
            self.mag_cb,
            qos_profile_sensor_data
        )

        self.pub = self.create_publisher(
            Float32,
            '/localization/heading_deg',
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f"Heading FAST | decl={self.declination_deg}° alpha={self.alpha}"
        )

    # =================================================
    #                     CALLBACKS
    # =================================================
    def mag_cb(self, msg: MagneticField):
        self.last_mag = msg.magnetic_field

    def imu_cb(self, msg: Imu):
        if self.last_mag is None:
            return

        q = msg.orientation
        yaw_imu = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        mag_heading = self.compute_mag_heading(
            q.x, q.y, q.z, q.w,
            self.last_mag.x,
            self.last_mag.y,
            self.last_mag.z
        )

        # ---- FUSIÓN COMPLEMENTARIA ----
        fused = self.alpha * yaw_imu + (1.0 - self.alpha) * mag_heading
        fused = self.normalize(fused)

        # ---- GANANCIA ----
        fused *= self.heading_gain
        fused = self.normalize(fused)

        # ---- RUIDO ARTIFICIAL DE HEADING ----
        noise = random.gauss(0.0, self.heading_noise_rad)
        fused += noise
        fused = self.normalize(fused)

        # ---- A GRADOS ----
        deg = math.degrees(fused)
        if deg < 0:
            deg += 360.0

        out = Float32()
        out.data = float(deg)
        self.pub.publish(out)

    # =================================================
    #               MAGNETIC HEADING
    # =================================================
    def compute_mag_heading(self, qx, qy, qz, qw, mx, my, mz):

        # --- Correcciones de ejes ---
        if self.swap_xy:
            mx, my = my, mx
        if self.invert_mag_x:
            mx = -mx
        if self.invert_mag_y:
            my = -my

        # --- Rotación cuerpo → mundo (solo XY) ---
        r00 = 1 - 2*(qy*qy + qz*qz)
        r01 = 2*(qx*qy - qz*qw)
        r10 = 2*(qx*qy + qz*qw)
        r11 = 1 - 2*(qx*qx + qz*qz)

        mwx = r00 * mx + r01 * my
        mwy = r10 * mx + r11 * my

        heading = math.atan2(mwy, mwx)
        heading = self.normalize(heading)

        # declinación
        heading += self.declination_rad
        return self.normalize(heading)

    # =================================================
    #                 UTILIDADES
    # =================================================
    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w*z + x*y)
        cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize(a):
        if a > math.pi:
            a -= 2*math.pi
        elif a < -math.pi:
            a += 2*math.pi
        return a


# =====================================================
def main(args=None):
    rclpy.init(args=args)
    node = HeadingEstimatorFast()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

