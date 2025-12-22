#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import threading
import tkinter as tk
from tkinter import ttk

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, Bool, String

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from hexapod_pkg import hw_config as cfg


class RobotHealthMonitor(Node):

    def __init__(self):
        super().__init__("robot_health_monitor")


        self.declare_parameter("topic_gps", cfg.TOPIC_GZ_GPS)
        self.declare_parameter("topic_imu", cfg.TOPIC_GZ_IMU_GIR_ACC)
        self.declare_parameter("topic_ir_left", cfg.TOPIC_GZ_IR1)
        self.declare_parameter("topic_ir_right", cfg.TOPIC_GZ_IR2)
        self.declare_parameter("topic_mag", cfg.TOPIC_GZ_IMU_MAG)
        self.declare_parameter("topic_ultrasonic", cfg.TOPIC_GZ_ULTRASONIC)

        self.declare_parameter("topic_gps_to_xy", cfg.TOPIC_GPS_TO_XY)
        self.declare_parameter("topic_estimate_heading", cfg.TOPIC_ESTIMATE_HEADING)
        self.declare_parameter("topic_ultrasonic_range", cfg.TOPIC_ULTRASONIC_RANGE)
        self.declare_parameter("topic_estimate_xy", cfg.TOPIC_ESTIMATE_XY)

        # topic de comando
        self.declare_parameter("cmd_serial_topic", cfg.TOPIC_CMD_SERIAL)

        topic_gps = self.get_parameter("topic_gps").value
        topic_imu = self.get_parameter("topic_imu").value
        topic_ir_left = self.get_parameter("topic_ir_left").value
        topic_ir_right = self.get_parameter("topic_ir_right").value
        topic_mag = self.get_parameter("topic_mag").value
        topic_ultrasonic = self.get_parameter("topic_ultrasonic").value

        topic_gps_to_xy = self.get_parameter("topic_gps_to_xy").value
        topic_estimate_heading = self.get_parameter("topic_estimate_heading").value
        topic_ultrasonic_range = self.get_parameter("topic_ultrasonic_range").value
        topic_estimate_xy = self.get_parameter("topic_estimate_xy").value

        cmd_robot_topic = self.get_parameter("cmd_robot_topic").value
        cmd_serial_topic = self.get_parameter("cmd_serial_topic").value



        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ==============================
        # TOPICS A MONITOREAR (SALUD)
        # ==============================
        self.health_topics = {
            topic_gps: "GPS",
            topic_imu: "IMU",
            topic_ir_left: "IR Left",
            topic_ir_right: "IR Right",
            topic_mag: "Magnetometer",
            topic_ultrasonic: "Ultrasonic",
            cmd_robot_topic: "Robot Command",
            cmd_serial_topic: "Serial Command",
        }

        # ==============================
        # ESTADO DEL ROBOT
        # ==============================
        self.state = {
            "gps_xy": None,
            "heading": None,
            "local_estimate": None,
            "ultrasonic": None,
            "ir_left": None,
            "ir_right": None,
            "hl_cmd": None,
        }

        # ==============================
        # CONTROL MASTER
        # ==============================
        self.current_mode = "teleop"

        self.mode_pub = self.create_publisher(
            String,
            "/control_master/mode",
            10
        )

        self.create_timer(0.2, self.publish_mode)

        # ==============================
        # SUBSCRIPCIONES
        # ==============================
        self.create_subscription(
            PointStamped,
            topic_gps_to_xy,
            self.cb_gps_xy,
            qos
        )

        self.create_subscription(
            Float32,
            topic_estimate_heading,
            self.cb_heading,
            qos
        )

        self.create_subscription(
            PointStamped,
            topic_estimate_xy,
            self.cb_local_estimate,
            qos
        )

        self.create_subscription(
            Float32,
            topic_ultrasonic_range,
            self.cb_ultrasonic,
            qos
        )

        self.create_subscription(
            Bool,
            topic_ir_left,
            self.cb_ir_left,
            qos
        )

        self.create_subscription(
            Bool,
            topic_ir_right,
            self.cb_ir_right,
            qos
        )

        self.create_subscription(
            String,
            "/hl_cmd",
            self.cb_hl_cmd,
            10
        )

        # ==============================
        # GUI
        # ==============================
        self.root = tk.Tk()
        self.root.title("Robot Master Monitor")
        self.root.geometry("650x580")

        self.build_ui()

        self.timer = self.create_timer(1.0, self.update_ui)

        self.get_logger().info("Robot Health + Control Master iniciado")

    # ==============================
    # CALLBACKS
    # ==============================
    def cb_gps_xy(self, msg):
        self.state["gps_xy"] = (msg.point.x, msg.point.y)

    def cb_heading(self, msg):
        self.state["heading"] = msg.data

    def cb_local_estimate(self, msg):
        self.state["local_estimate"] = (msg.point.x, msg.point.y)

    def cb_ultrasonic(self, msg):
        self.state["ultrasonic"] = msg.data

    def cb_ir_left(self, msg):
        self.state["ir_left"] = msg.data

    def cb_ir_right(self, msg):
        self.state["ir_right"] = msg.data

    def cb_hl_cmd(self, msg):
        self.state["hl_cmd"] = msg.data

    # ==============================
    # CONTROL MASTER
    # ==============================
    def publish_mode(self):
        msg = String()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)

    def on_mode_change(self):
        self.current_mode = self.mode_var.get()
        self.get_logger().info(f"Modo activo → {self.current_mode}")

    # ==============================
    # UI
    # ==============================
    def build_ui(self):
        self.labels = {}

        style = ttk.Style()
        style.configure("TLabel", font=("Courier", 10))
        style.configure("TRadiobutton", font=("Courier", 10))

        def section(title):
            lbl = ttk.Label(
                self.root,
                text=title,
                font=("Courier", 11, "bold")
            )
            lbl.pack(pady=(10, 4), anchor="w", padx=10)

        # -------- SENSOR HEALTH --------
        section("SENSOR HEALTH")
        for topic, label in self.health_topics.items():
            l = ttk.Label(self.root, text=f"{label:<15}: ---")
            l.pack(anchor="w", padx=20)
            self.labels[topic] = l

        # -------- ROBOT STATE --------
        section("ROBOT STATE")
        for key in ["gps_xy", "heading", "local_estimate", "ultrasonic", "ir_left", "ir_right"]:
            l = ttk.Label(self.root, text=f"{key:<18}: ---")
            l.pack(anchor="w", padx=20)
            self.labels[key] = l

        # -------- HL CMD --------
        section("HIGH LEVEL COMMAND")
        self.hl_label = ttk.Label(
            self.root,
            text="hl_cmd            : ---",
            font=("Courier", 10)
        )
        self.hl_label.pack(anchor="w", padx=20)

        # -------- CONTROL MODE --------
        # section("CONTROL MODE (MASTER)")
        # self.mode_var = tk.StringVar(value=self.current_mode)

        # for mode in ["teleop", "navigation", "social"]:
        #     rb = ttk.Radiobutton(
        #         self.root,
        #         text=mode,
        #         value=mode,
        #         variable=self.mode_var,
        #         command=self.on_mode_change
        #     )
        #     rb.pack(anchor="w", padx=20)

        self.mode_label = ttk.Label(
             self.root,
            text=f"{'mode':<18}: {self.current_mode}",
             font=("Courier", 10, "bold")
        )
        self.mode_label.pack(anchor="w", padx=20)

        self.time_label = ttk.Label(self.root, text="")
        self.time_label.pack(pady=10, anchor="e", padx=10)

    # ==============================
    # UPDATE UI
    # ==============================
    def update_ui(self):
        topic_info = self.get_topic_names_and_types()
        topic_map = {name: types for name, types in topic_info}

        # SENSOR HEALTH
        for topic, label in self.health_topics.items():
            if topic in topic_map:
                msg_type = topic_map[topic][0]
                self.labels[topic].config(
                    text=f"{label:<15}: Available | {msg_type}"
                )
            else:
                self.labels[topic].config(
                    text=f"{label:<15}: Missing"
                )

        def fmt(v):
            return "N/A" if v is None else str(v)

        # ROBOT STATE
        self.labels["gps_xy"].config(
            text=f"{'gps_xy':<18}: {fmt(self.state['gps_xy'])}"
        )
        self.labels["heading"].config(
            text=f"{'heading (deg)':<18}: {fmt(self.state['heading'])}"
        )
        self.labels["local_estimate"].config(
            text=f"{'local_estimate':<18}: {fmt(self.state['local_estimate'])}"
        )
        self.labels["ultrasonic"].config(
            text=f"{'ultrasonic (m)':<18}: {fmt(self.state['ultrasonic'])}"
        )
        self.labels["ir_left"].config(
            text=f"{'ir_left':<18}: {fmt(self.state['ir_left'])}"
        )
        self.labels["ir_right"].config(
            text=f"{'ir_right':<18}: {fmt(self.state['ir_right'])}"
        )

        self.hl_label.config(
            text=f"{'hl_cmd':<18}: {fmt(self.state['hl_cmd'])}",
            foreground="black"
        )

        # self.mode_label.config(
        #     text=f"{'mode':<18}: {self.current_mode}"
        # )

        self.time_label.config(
            text=f"Updated: {time.strftime('%H:%M:%S')}"
        )

        self.root.update_idletasks()
        self.root.update()

    # ==============================
    # MAIN
    # ==============================
def main():
    rclpy.init()
    node = RobotHealthMonitor()

    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True
    )
    ros_thread.start()

    try:
        node.root.mainloop()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
