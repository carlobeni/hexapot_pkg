#!/usr/bin/env python3
# dds_monitor_pc.py

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from sensor_msgs.msg import (
    Range,
    NavSatFix,
    Image,
    Imu,
    MagneticField,
)

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)

from hexapod_pkg import hw_config as cfg

class MonitorNode(Node):

    def __init__(self):
        super().__init__("dds_monitor_pc")

        # =====================================================
        # DECLARACIÓN DE PARÁMETROS (SOLO TIPOS ROS VÁLIDOS)
        # =====================================================
        self.declare_parameter("topic_imu", cfg.TOPIC_PI_PHONE_IMU_GIR_ACC)
        self.declare_parameter("topic_mag", cfg.TOPIC_PI_PHONE_IMU_MAG)
        self.declare_parameter("topic_gps", cfg.TOPIC_PI_PHONE_GPS)
        self.declare_parameter("topic_ultrasonic", cfg.TOPIC_PI_ULTRASONIC)
        self.declare_parameter("topic_ir1", cfg.TOPIC_PI_IR1)
        self.declare_parameter("topic_ir2", cfg.TOPIC_PI_IR2)
        self.declare_parameter("topic_camera", cfg.TOPIC_PI_PHONE_CAMERA)
        self.declare_parameter("topic_cmd_serial", cfg.TOPIC_CMD_SERIAL)

        # =====================================================
        # LEER PARÁMETROS
        # =====================================================
        self.topic_imu        = self.get_parameter("topic_imu").value
        self.topic_mag        = self.get_parameter("topic_mag").value
        self.topic_gps        = self.get_parameter("topic_gps").value
        self.topic_ultrasonic = self.get_parameter("topic_ultrasonic").value
        self.topic_ir1         = self.get_parameter("topic_ir1").value
        self.topic_ir2         = self.get_parameter("topic_ir2").value
        self.topic_camera     = self.get_parameter("topic_camera").value
        self.topic_cmd_serial = self.get_parameter("topic_cmd_serial").value

        # =====================================================
        # QoS
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
        # DATA STORAGE
        # data[topic] = (timestamp_or_id, value)
        # =====================================================
        self.data = {}

        # =====================================================
        # SUBSCRIBERS (solo si el topic fue definido)
        # =====================================================
        self._try_sub(Imu,            self.topic_imu,        self.qos_sensors)
        self._try_sub(MagneticField,  self.topic_mag,        self.qos_sensors)
        self._try_sub(NavSatFix,      self.topic_gps,        self.qos_sensors)
        self._try_sub(Range,          self.topic_ultrasonic, self.qos_sensors)
        self._try_sub(Bool,self.topic_ir1,         self.qos_sensors)
        self._try_sub(Bool,self.topic_ir2,         self.qos_sensors)
        self._try_sub(Image,self.topic_camera,     self.qos_sensors)

        self._try_sub(
            String,
            self.topic_cmd_serial,
            self.qos_commands,
            is_command=True
        )

        self.timer = self.create_timer(0.2, self.print_table)

        self.get_logger().info("DDS Monitor PC READY")

    # -----------------------------------------------------
    def _try_sub(self, msg_type, topic, qos, is_command=False):
        if not topic:
            return

        self.data[topic] = ("-", "---")

        def cb(msg):
            if is_command:
                if ":" in msg.data:
                    msg_id, value = msg.data.split(":", 1)
                    self.data[topic] = (msg_id.strip(), value.strip())
                else:
                    self.data[topic] = ("?", msg.data)
            else:
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

        if isinstance(msg, Bool):
            return str(msg.data)

        if isinstance(msg, Range):
            return f"{msg.range:.3f}"

        if isinstance(msg, NavSatFix):
            return f"{msg.latitude:.6f}, {msg.longitude:.6f}"

        if isinstance(msg, Imu):
            q = msg.orientation
            av = msg.angular_velocity
            la = msg.linear_acceleration

            IND = " " * 55
            return (
                f"ori     = [{q.x:.2f}, {q.y:.2f}, {q.z:.2f}, {q.w:.2f}]\n"
                f"{IND}ang_vel = [{av.x:.2f}, {av.y:.2f}, {av.z:.2f}]\n"
                f"{IND}lin_acc = [{la.x:.2f}, {la.y:.2f}, {la.z:.2f}]"
            )

        if isinstance(msg, MagneticField):
            m = msg.magnetic_field
            return f"x={m.x:.2f}, y={m.y:.2f}, z={m.z:.2f}"

        if isinstance(msg, Image):
            return "frame"

        return "?"

    # -----------------------------------------------------
    def print_table(self):
        print("\033[2J\033[H", end="")
        print("=== DDS MONITOR (PC) ===\n")

        print("---- SENSORS ----")
        print("{:<35} | {:<22} | {:<70}".format("TOPIC", "TIMESTAMP", "VALUE"))
        print("-" * 80)

        for topic, (ts, value) in self.data.items():
            if topic == self.topic_cmd_serial:
                continue
            print("{:<35} | {:<22} | {:<70}".format(topic, ts, value))

        print("\n---- COMMANDS ----")
        print("{:<35} | {:<22} | {:<70}".format("TOPIC", "ID", "VALUE"))
        print("-" * 80)

        if self.topic_cmd_serial in self.data:
            msg_id, value = self.data[self.topic_cmd_serial]
            print("{:<35} | {:<22} | {:<70}".format(
                self.topic_cmd_serial, msg_id, value
            ))

        print("\nCtrl+C to exit")


# ======================================================
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
