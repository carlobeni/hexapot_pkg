#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from hexapod_pkg import hw_config as cfg

class TeleOpHexapod(Node):
    def __init__(self):
        super().__init__("tele_op_hexapod")

        # Publicador
        self.cmd_pub = self.create_publisher(String,cfg.TOPIC_CMD_REAL_ROBOT, 10)

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


class TeleOpHexapod(Node):

    def __init__(self):
        super().__init__("tele_op_hexapod_real")

        self.cmd_pub = self.create_publisher(
            String, cfg.TOPIC_CMD_REAL_ROBOT, 10
        )

        self.last_cmd_sent = None
        self.stop_event = threading.Event()

        self.input_thread = threading.Thread(
            target=self.keyboard_loop, daemon=True
        )
        self.input_thread.start()

        self.get_logger().info(
            "Control manual activo:"
            " w = forward"
            " a = turn_left"
            " d = turn_right"
            " s = backward"
            " q = lateral_left"
            " e = lateral_right"
            " x = stop"
        )

    # ---------- Publicar comando ----------
    def send_cmd(self, cmd: str):
        if cmd == self.last_cmd_sent:
            return

        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)

        self.last_cmd_sent = cmd
        self.get_logger().info(f"CMD enviado: {cmd}")

    # ---------- Teclado raw ----------
    def keyboard_loop(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)
            while rclpy.ok() and not self.stop_event.is_set():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).lower()

                    if key == "\x03":  # Ctrl+C
                        self.stop_event.set()
                        rclpy.shutdown()
                        break

                    if key == "x":
                        self.send_cmd("stop")
                    elif key == "1":
                        self.send_cmd("mode_1")
                    elif key == "2":
                        self.send_cmd("mode_2")
                    elif key == "3":
                        self.send_cmd("mode_3")
                    elif key == "w":
                        self.send_cmd("forward")
                    elif key == "s":
                        self.send_cmd("backward")
                    elif key == "a":
                        self.send_cmd("turn_left")
                    elif key == "d":
                        self.send_cmd("turn_right")
                    elif key == "q":
                        self.send_cmd("lateral_left")
                    elif key == "e":
                        self.send_cmd("lateral_right")

        finally:
            # CRÍTICO: restaurar terminal siempre
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)

    node = TeleOpHexapod()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_event.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
