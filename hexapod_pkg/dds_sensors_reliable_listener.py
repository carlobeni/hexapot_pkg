#!/usr/bin/env python3
# dds_sensors_reliable_listener.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import hexapod_pkg.hexapod_pkg.hw_config as cfg

class SensorsReliableListener(Node):
    def __init__(self):
        super().__init__("dds_sensors_reliable_listener")
        cfg.check_domain_id(self.get_logger())

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(NavSatFix, cfg.TOPIC_GPS, self.cb, qos)
    def cb(self, msg):
        pass

def main():
    rclpy.init()
    rclpy.spin(SensorsReliableListener())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
