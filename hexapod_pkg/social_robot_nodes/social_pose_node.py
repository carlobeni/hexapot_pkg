#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import math


# ============================================================
#     BASE DE MOVIMIENTO (idéntica filosofía a tus gaits)
# ============================================================
class BaseMotion:
    def __init__(self, node, joints, update_hz=20):
        self.node = node
        self.joints = joints

        self.pub = node.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10
        )

        self.current = {j: 0.0 for j in joints}
        self.target  = {}
        self.start   = {}
        self.received = False

        self.T = 0.6          # duración del movimiento (rápido)
        self.t0 = None
        self.active = False

        node.create_subscription(JointState, "/joint_states", self.joint_cb, 10)
        self.timer = node.create_timer(1.0 / update_hz, self.update)

    def joint_cb(self, msg):
        for n, p in zip(msg.name, msg.position):
            if n in self.current:
                self.current[n] = p
        self.received = True

    def set_target(self, target_dict):
        if not self.received:
            return

        self.start = dict(self.current)
        self.target = dict(target_dict)
        self.t0 = self.node.get_clock().now()
        self.active = True

    def interpolate(self, a, b, s):
        return a + (b - a) * s

    def publish_pose(self, P, t):
        traj = JointTrajectory()
        traj.joint_names = self.joints

        pt = JointTrajectoryPoint()
        pt.positions = [P.get(j, self.current[j]) for j in self.joints]
        pt.time_from_start.nanosec = int(t * 1e9)

        traj.points.append(pt)
        self.pub.publish(traj)

    def update(self):
        if not self.active or not self.received:
            return

        now = self.node.get_clock().now()
        elapsed = (now - self.t0).nanoseconds / 1e9
        s = min(elapsed / self.T, 1.0)

        # easing suave (igual que tus sway implícitos)
        s = 0.5 - 0.5 * math.cos(math.pi * s)

        P = {}
        for j in self.joints:
            q0 = self.start[j]
            q1 = self.target.get(j, q0)
            P[j] = self.interpolate(q0, q1, s)

        self.publish_pose(P, max(self.T - elapsed, 0.05))

        if elapsed >= self.T:
            self.active = False


# ============================================================
#     MOVIMIENTO SOCIAL: PATAS DELANTERAS
# ============================================================
class SocialFrontLegsMotion(BaseMotion):
    def __init__(self, node):
        joints = [
            "j_c1_rf","j_thigh_rf","j_tibia_rf",
            "j_c1_rm","j_thigh_rm","j_tibia_rm",
            "j_c1_rr","j_thigh_rr","j_tibia_rr",
            "j_c1_lf","j_thigh_lf","j_tibia_lf",
            "j_c1_lm","j_thigh_lm","j_tibia_lm",
            "j_c1_lr","j_thigh_lr","j_tibia_lr"
        ]
        super().__init__(node, joints, update_hz=20)

    # ======================================================
    # EDITÁ ÁNGULOS ACÁ
    # ======================================================
    def pose_rock(self):
        return {
            "j_c1_rf": 0.0, "j_thigh_rf": 0.0, "j_tibia_rf": 0.0,
            "j_c1_rm": 0.0, "j_thigh_rm": -0.9, "j_tibia_rm": 1.,
            "j_c1_rr": 0.0, "j_thigh_rr": 0.0, "j_tibia_rr": 0.0,
            "j_c1_lf": 0.0, "j_thigh_lf": 0.0, "j_tibia_lf": 0.0,
            "j_c1_lm": 0.0, "j_thigh_lm": -0.9, "j_tibia_lm": 1.2,
            "j_c1_lr": 0.0, "j_thigh_lr": 0.0, "j_tibia_lr": 0.0,
        }

    def pose_neutral(self):
        return {
            "j_c1_rf": 0.0, "j_thigh_rf": 0.0, "j_tibia_rf": 0.0,
            "j_c1_rm": 0.0, "j_thigh_rm": 0.0, "j_tibia_rm": 0.0,
            "j_c1_rr": 0.0, "j_thigh_rr": 0.0, "j_tibia_rr": 0.0,
            "j_c1_lf": 0.0, "j_thigh_lf": 0.0, "j_tibia_lf": 0.0,
            "j_c1_lm": 0.0, "j_thigh_lm": 0.0, "j_tibia_lm": 0.0,
            "j_c1_lr": 0.0, "j_thigh_lr": 0.0, "j_tibia_lr": 0.0,
        }


# ============================================================
#     NODO PRINCIPAL
# ============================================================
class SocialMotionNode(Node):
    def __init__(self):
        super().__init__("social_motion_node")

        self.motion = SocialFrontLegsMotion(self)
        self.last_cmd = None

        self.create_subscription(
            String,
            "/social_cmd",
            self.cb_social,
            10
        )

        self.get_logger().info("Nodo social fluido iniciado")

    def cb_social(self, msg):
        cmd = msg.data.strip()

        if cmd == self.last_cmd:
            return

        if cmd == "Rock":
            self.motion.set_target(self.motion.pose_rock())
        else:
            self.motion.set_target(self.motion.pose_neutral())

        self.last_cmd = cmd


def main(args=None):
    rclpy.init(args=args)
    node = SocialMotionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
