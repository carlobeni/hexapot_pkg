#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class MoveRFLeg(Node):
    def __init__(self):
        super().__init__("move_rf_leg")

        # Publicador al controlador
        self.pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10
        )

        # Suscripción para obtener posiciones actuales
        self.joint_states_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_states_callback,
            10
        )

        # Lista exacta de tus 18 articulaciones
        self.joints = [
            "j_c1_rf", "j_thigh_rf", "j_tibia_rf",
            "j_c1_rm", "j_thigh_rm", "j_tibia_rm",
            "j_c1_rr", "j_thigh_rr", "j_tibia_rr",
            "j_c1_lf", "j_thigh_lf", "j_tibia_lf",
            "j_c1_lm", "j_thigh_lm", "j_tibia_lm",
            "j_c1_lr", "j_thigh_lr", "j_tibia_lr",
        ]

        self.current_positions = {j: 0.0 for j in self.joints}
        self.received_states = False

        # Enviar comando luego de 1 segundo
        self.timer = self.create_timer(1.0, self.send_once)

        self.already_sent = False

    def joint_states_callback(self, msg: JointState):
        # Guardar posiciones actuales
        for name, pos in zip(msg.name, msg.position):
            if name in self.current_positions:
                self.current_positions[name] = pos
        self.received_states = True

    def send_once(self):
        if self.already_sent:
            return

        if not self.received_states:
            self.get_logger().info("Esperando /joint_states...")
            return

        # ========================
        # Posiciones deseadas RF
        # ========================
        #desired_rf = {
            #"j_c1_rf": 0.0,
            #"j_thigh_rf": 0,
            #"j_tibia_rf": 0.785398
        #}

        desired_rf = {
            "j_c1_rf": -math.pi/4, "j_thigh_rf": -math.pi/4, "j_tibia_rf": -math.pi/4,
            "j_c1_rm": 0.0, "j_thigh_rm": -math.pi/4, "j_tibia_rm":-math.pi/4,
            "j_c1_rr": +math.pi/4, "j_thigh_rr": -math.pi/4, "j_tibia_rr": -math.pi/4,
            "j_c1_lf": +math.pi/4, "j_thigh_lf": -math.pi/4, "j_tibia_lf": -math.pi/4,
            "j_c1_lm": 0.0, "j_thigh_lm": -math.pi/4, "j_tibia_lm": -math.pi/4,
            "j_c1_lr": -math.pi/4, "j_thigh_lr": -math.pi/4, "j_tibia_lr": -math.pi/4,
        }

        # Armar lista de posiciones finales para los 18 joints
        final_positions = []
        for joint in self.joints:
            if joint in desired_rf:
                final_positions.append(desired_rf[joint])
            else:
                final_positions.append(self.current_positions[joint])

        # Construir mensaje
        traj = JointTrajectory()
        traj.joint_names = self.joints

        point = JointTrajectoryPoint()
        point.positions = final_positions
        point.time_from_start.sec = 2

        traj.points.append(point)

        # Publicar
        self.pub.publish(traj)
        self.get_logger().info("Trajectoria enviada a RF")

        self.already_sent = True


def main(args=None):
    rclpy.init(args=args)
    node = MoveRFLeg()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
