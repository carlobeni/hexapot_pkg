#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class TeleOpHexapod(Node):
    def __init__(self):
        super().__init__("tele_op_hexapod")

        # Publicador
        self.cmd_pub = self.create_publisher(String, "/hl_cmd", 10)

        # Comando actual (manual)
        self.current_cmd = "stop"

        # Timer de publicación
        self.timer = self.create_timer(0.05, self.loop)

        # Hilo para leer teclado
        self.input_thread = threading.Thread(
            target=self.console_input_loop, daemon=True)
        self.input_thread.start()

        self.get_logger().info(
            "Control manual activo:\n"
            " w = forward\n"
            " a = turn_left\n"
            " d = turn_right\n"
            " s = backward\n"
            " q = lateral_left\n"
            " e = lateral_right\n"
            " x = stop\n"
        )


    # ---------- Lectura de consola ----------
    def console_input_loop(self):
        while rclpy.ok():
            try:
                key = input().strip().lower()

                if key == "w":
                    self.current_cmd = "forward"
                elif key == "a":
                    self.current_cmd = "turn_left"
                elif key == "d":
                    self.current_cmd = "turn_right"
                elif key == "s":
                    self.current_cmd = "backward"
                elif key == "q":
                    self.current_cmd = "lateral_left"
                elif key == "e":
                    self.current_cmd = "lateral_right"
                elif key == "x":
                    self.current_cmd = "stop"
                else:
                    print("Comando inválido (w/a/d/s)")
            except EOFError:
                break

    # ---------- Loop principal ----------
    def loop(self):
        cmd = String()
        cmd.data = self.current_cmd
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    node = TeleOpHexapod()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
