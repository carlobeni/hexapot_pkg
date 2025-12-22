#!/usr/bin/env python3
import sys
import termios
import tty
import select
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from hexapod_pkg import hw_config as cfg


class TeleOpHexapod(Node):

    def __init__(self):
        super().__init__("tele_op_hexapod_real")

        self.cmd_pub = self.create_publisher(String, cfg.TOPIC_CMD_REAL_ROBOT, 10)

        self.active_cmd = None
        self.last_key_time = 0.0
        self.key_timeout = 0.15  # s
        self.stop_requested = False

        self.timer = self.create_timer(0.05, self.loop)

        self.input_thread = threading.Thread(
            target=self.keyboard_loop, daemon=True
        )
        self.input_thread.start()

        self.get_logger().info(
            "TeleOp activo (mantener tecla): w = forward; a = turn_left; d = turn_right; s = backward; q = lateral_left; e = lateral_right; x = STOP"
        )

    # ---------- Teclado raw ----------
    def keyboard_loop(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    key = sys.stdin.read(1).lower()
                    self.last_key_time = time.time()

                    if key == "x":
                        self.stop_requested = True
                        self.active_cmd = None
                        continue

                    self.stop_requested = False
                    
                    # ---- MODOS ----
                    if key == "1":
                        self.active_cmd = "mode_1"
                    elif key == "2":
                        self.active_cmd = "mode_2"
                    elif key == "3":
                        self.active_cmd = "mode_3"

                    # ---- MOVIMIENTO ----
                    if key == "w":
                        self.active_cmd = "forward"
                    elif key == "a":
                        self.active_cmd = "turn_left"
                    elif key == "d":
                        self.active_cmd = "turn_right"
                    elif key == "s":
                        self.active_cmd = "backward"
                    elif key == "q":
                        self.active_cmd = "lateral_left"
                    elif key == "e":
                        self.active_cmd = "lateral_right"
                    else:
                        self.active_cmd = None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    # ---------- Loop ----------
    def loop(self):
        # STOP tiene prioridad
        if self.stop_requested:
            msg = String()
            msg.data = "stop"
            self.cmd_pub.publish(msg)
            self.stop_requested = False
            return

        # Si se soltó la tecla → no publicar
        if self.active_cmd is None:
            return

        if (time.time() - self.last_key_time) > self.key_timeout:
            self.active_cmd = None
            return

        msg = String()
        msg.data = self.active_cmd
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = TeleOpHexapod()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
