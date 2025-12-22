#!/usr/bin/env python3
# sensor_ultrasonic.py
import rclpy
from rclpy.node import Node


from sensor_msgs.msg import Range
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from hexapod_pkg import hw_config as cfg

class UltrasonicRangeFast(Node):

    def __init__(self):
        super().__init__('sensor_ultrasonic')

        # ================= PARÁMETROS =================
        self.declare_parameter("topic_ultrasonic_raw", cfg.TOPIC_GZ_ULTRASONIC)
        self.declare_parameter("topic_ultrasonic_range", cfg.TOPIC_ULTRASONIC_RANGE)

        topic_ultrasonic_raw = self.get_parameter("topic_ultrasonic_raw").value
        topic_ultrasonic_range = self.get_parameter("topic_ultrasonic_range").value



        # ================= PARÁMETROS =================
        self.min_valid = 0.02    # 2 cm
        self.max_valid = 4.50    # 4.5 m
        self.no_echo_value = 1.4500000476837158  # valor cuando no hay detección
        self.no_echo_tol   = 0.05  # tolerancia numérica

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        

        # ================= ROS ========================
        self.create_subscription(
            Range,
            topic_ultrasonic_raw,
            self.range_cb,
            qos
        )

        self.pub = self.create_publisher(
            Float32,
            topic_ultrasonic_range,
            qos
        )

        self.get_logger().info(
            "UltrasonicRangeFast | valid=[2cm–4.5m], no_echo -> -1"
        )

    # =================================================
    #                   CALLBACK
    # =================================================
    def range_cb(self, msg: Range):

        r = msg.range
        out = Float32()

        # --- Caso: sin detección ---
        if abs(r - self.no_echo_value) < self.no_echo_tol:
            out.data = -1.0

        # --- Caso: detección válida ---
        elif self.min_valid <= r <= self.max_valid:
            out.data = float(r)

        # --- Caso: valor inválido ---
        else:
            out.data = -1.0

        self.pub.publish(out)


# =====================================================
def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicRangeFast()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
