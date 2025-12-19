#!/usr/bin/env python3
# dds_monitor_pc.py

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray, String
from sensor_msgs.msg import (
    Range,
    NavSatFix,
    CompressedImage,
    Imu,
    MagneticField,
)

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)

import hw_config as cfg


class MonitorNode(Node):

    def __init__(self):
        super().__init__("dds_monitor_pc")
        cfg.check_domain_id(self.get_logger())

        # =====================================================
        # QoS PROFILES
        # =====================================================
        self.qos_sensors = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.qos_commands = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # =====================================================
        # FLAGS (se mantienen, aunque no se usan aquí)
        # =====================================================
        self.declare_parameter("monitor_cmd_serial", True)
        self.declare_parameter("monitor_ir", True)
        self.declare_parameter("monitor_ultrasonic", True)
        self.declare_parameter("monitor_gps", True)
        self.declare_parameter("monitor_imu", True)
        self.declare_parameter("monitor_camera", True)

        # =====================================================
        # DATA STORAGE
        # data[topic] = (timestamp_or_id, value)
        # =====================================================
        self.data = {}

        # ================= Sensors =================
        self._sub(Int32MultiArray, cfg.TOPIC_IR, self.qos_sensors)
        self._sub(Range, cfg.TOPIC_ULTRASONIC, self.qos_sensors)
        self._sub(NavSatFix, cfg.TOPIC_GPS, self.qos_sensors)

        # ---- IMU NUEVA ----
        self._sub(Imu, cfg.TOPIC_IMU_GIR_ACC, self.qos_sensors)
        self._sub(MagneticField, cfg.TOPIC_IMU_MAG, self.qos_sensors)

        self._sub(CompressedImage, cfg.TOPIC_CAMERA, self.qos_sensors)

        # ================= Commands =================
        self._sub(
            String,
            cfg.TOPIC_CMD_SERIAL,
            self.qos_commands,
            is_command=True
        )

        self.timer = self.create_timer(0.2, self.print_table)

    # -----------------------------------------------------

    def _sub(self, msg_type, topic, qos, is_command=False):
        self.data[topic] = ("-", "---")

        def cb(msg):
            if is_command:
                # ===== COMMAND =====
                if ":" in msg.data:
                    msg_id, value = msg.data.split(":", 1)
                    self.data[topic] = (msg_id.strip(), value.strip())
                else:
                    self.data[topic] = ("?", msg.data)
            else:
                # ===== SENSOR =====
                if hasattr(msg, "header"):
                    stamp = msg.header.stamp
                    timestamp = f"{stamp.sec}.{stamp.nanosec:09d}"
                else:
                    timestamp = "N/A"

                value = self._format(msg)
                self.data[topic] = (timestamp, value)

        self.create_subscription(msg_type, topic, cb, qos)

    # -----------------------------------------------------

    def _format(self, msg):

        # ---------- IR ----------
        if isinstance(msg, Int32MultiArray):
            return str(msg.data)

        # ---------- ULTRASONIC ----------
        if isinstance(msg, Range):
            return f"{msg.range:.3f} m"

        # ---------- GPS ----------
        if isinstance(msg, NavSatFix):
            return f"{msg.latitude:.6f}, {msg.longitude:.6f}"

        # ---------- IMU ----------
        if isinstance(msg, Imu):
            q = msg.orientation
            av = msg.angular_velocity
            la = msg.linear_acceleration

            INDENT = " " * 63  # alinear con columna VALUE

            return (
                f"ori     = [{q.x:.2f}, {q.y:.2f}, {q.z:.2f}, {q.w:.2f}]\n"
                f"{INDENT}ang_vel = [{av.x:.2f}, {av.y:.2f}, {av.z:.2f}]\n"
                f"{INDENT}lin_acc = [{la.x:.2f}, {la.y:.2f}, {la.z:.2f}]"
            )

        # ---------- MAG ----------
        if isinstance(msg, MagneticField):
            m = msg.magnetic_field
            return f"x={m.x:.2f}, y={m.y:.2f}, z={m.z:.2f}"

        # ---------- CAMERA ----------
        if isinstance(msg, CompressedImage):
            return "frame"

        return "?"

    # -----------------------------------------------------

    def print_table(self):
        print("\033[2J\033[H", end="")
        print("=== MONITOR (CONTROL DEVICE) ===")
        print(f"ROS_DOMAIN_ID = {cfg.ROS_DOMAIN_ID}\n")

        # -------- SENSORS --------
        print("---- SUBSCRIBERS (SENSORS) ----")
        print("{:<35} | {:<22} | {:<70}".format("TOPIC", "TIMESTAMP", "VALUE"))
        print("-" * 135)

        for topic, (ts, value) in self.data.items():
            if topic == cfg.TOPIC_CMD_SERIAL:
                continue
            print("{:<35} | {:<22} | {:<70}".format(topic, ts, value))

        # -------- COMMANDS --------
        print("\n---- SUBSCRIBERS (COMMANDS) ----")
        print("{:<35} | {:<22} | {:<70}".format("TOPIC", "ID", "VALUE"))
        print("-" * 135)

        if cfg.TOPIC_CMD_SERIAL in self.data:
            msg_id, value = self.data[cfg.TOPIC_CMD_SERIAL]
            print("{:<35} | {:<22} | {:<70}".format(
                cfg.TOPIC_CMD_SERIAL, msg_id, value
            ))

        print("\nCtrl+C to exit")


def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
