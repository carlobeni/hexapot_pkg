#!/usr/bin/env python3
# gz_hexapod_inv_kinematics.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time


# ============================================================
#     CLASE BASE DE GAIT (para evitar copiar 200 líneas)
# ============================================================
class BaseGait:
    def __init__(self, node, joints, update_hz=20):
        self.node = node
        self.pub = node.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10
        )
        self.current = {j: 0.0 for j in joints}
        self.joints = joints
        self.received = False

        node.create_subscription(JointState, "/joint_states", self.joint_cb, 10)

        self.timer = node.create_timer(1.0/update_hz, self.update)
        self.enabled = False

    def joint_cb(self, msg):
        for n, p in zip(msg.name, msg.position):
            if n in self.current:
                self.current[n] = p
        self.received = True

    def publish_pose(self, P, t):
        traj = JointTrajectory()
        traj.joint_names = self.joints
        pt = JointTrajectoryPoint()
        pt.time_from_start.nanosec = int(t*1e9)
        pt.positions = [P.get(j, self.current[j]) for j in self.joints]
        traj.points.append(pt)
        self.pub.publish(traj)

    def enable(self):
        self.enabled = True
        #self.node.get_logger().info(f"[GAIT] Activado: {type(self).__name__}")

    def disable(self):
        self.enabled = False
        #self.node.get_logger().info(f"[GAIT] Desactivado: {type(self).__name__}")

    def update(self):
        pass


# ============================================================
#     GAIT RECTILÍNEO 
# ============================================================
class TripodForwardGait(BaseGait):
    def __init__(self, node):
        joints = [
            "j_c1_rf","j_thigh_rf","j_tibia_rf",
            "j_c1_rm","j_thigh_rm","j_tibia_rm",
            "j_c1_rr","j_thigh_rr","j_tibia_rr",
            "j_c1_lf","j_thigh_lf","j_tibia_lf",
            "j_c1_lm","j_thigh_lm","j_tibia_lm",
            "j_c1_lr","j_thigh_lr","j_tibia_lr"
        ]
        super().__init__(node, joints, update_hz=60)

        self.tripod_A = ["lf", "rm", "lr"]
        self.tripod_B = ["rf", "lm", "rr"]

        # Parámetros de marcha (SIMÉTRICOS)
        self.step_length = 0.45
        self.step_height = 0.7

        self.T = 0.45
        self.start_time = node.get_clock().now()

    def mirror(self, leg, c1):
        # ÚNICA función: espejo geométrico
        if leg.startswith("l"):
            return -c1
        return c1

    def leg_pose(self, leg, phi, support=True):
        if support:
            c1 = -self.step_length * math.sin(math.pi * (phi - 0.5))
            tibia = 0.0
        else:
            c1 = self.step_length * math.sin(math.pi * (phi - 0.5))
            tibia = self.step_height * math.sin(math.pi * phi)

        c1 = self.mirror(leg, c1)

        return {
            f"j_c1_{leg}": c1,
            f"j_thigh_{leg}": 0.0,
            f"j_tibia_{leg}": tibia
        }

    def update(self):
        if not self.enabled or not self.received:
            return

        now = self.node.get_clock().now()
        t = (now - self.start_time).nanoseconds / 1e9
        phi = (t % self.T) / self.T

        P = {}

        for leg in self.tripod_A:
            P.update(self.leg_pose(leg, phi, support=(phi < 0.5)))

        for leg in self.tripod_B:
            P.update(self.leg_pose(
                leg,
                (phi + 0.5) % 1.0,
                support=(phi >= 0.5)
            ))

        self.publish_pose(P, 1.0 / 60.0)


# ============================================================
#     GAIT RECTILÍNEO TRASERO
# ============================================================
class TripodBackwardGait(TripodForwardGait):
    def __init__(self, node):
        super().__init__(node)
        # invertir avance longitudinal
        self.step_length = -self.step_length


# ============================================================
#     GIRO ANTI HORARIO 
# ============================================================
class RotateCCW(BaseGait):
    def __init__(self, node):
        joints = [
            "j_c1_rf","j_thigh_rf","j_tibia_rf",
            "j_c1_rm","j_thigh_rm","j_tibia_rm",
            "j_c1_rr","j_thigh_rr","j_tibia_rr",
            "j_c1_lf","j_thigh_lf","j_tibia_lf",
            "j_c1_lm","j_thigh_lm","j_tibia_lm",
            "j_c1_lr","j_thigh_lr","j_tibia_lr"
        ]
        super().__init__(node, joints, update_hz=60)

        self.tripod_A = ["lf", "rm", "lr"]
        self.tripod_B = ["rf", "lm", "rr"]

        self.spread = {"A": -0.25, "B": +0.25}
        self.push = +0.40

        self.step_height = 1.15

        self.T = 0.6   # ↓ más chico = más rápido
        self.start_time = node.get_clock().now()

    def apply_pose(self, leg, c1, tibia):
        return {
            f"j_c1_{leg}": c1,
            f"j_thigh_{leg}": 0.0,
            f"j_tibia_{leg}": tibia
        }

    def update(self):
        if not self.enabled or not self.received:
            return

        now = self.node.get_clock().now()
        t = (now - self.start_time).nanoseconds / 1e9
        phi = (t % self.T) / self.T          # 0..1
        alpha = 0.5 * (1 - math.cos(2 * math.pi * phi))  # suavizado

        P = {}

        # === Trípode A en apoyo, B en swing ===
        if phi < 0.5:
            # apoyo A
            for l in self.tripod_A:
                c1 = self.spread["A"] + self.push
                P.update(self.apply_pose(l, c1, 0.0))

            # swing B
            for l in self.tripod_B:
                tibia = self.step_height * math.sin(math.pi * alpha)
                P.update(self.apply_pose(l, 0.0, tibia))

        # === Trípode B en apoyo, A en swing ===
        else:
            for l in self.tripod_B:
                c1 = self.spread["B"] + self.push
                P.update(self.apply_pose(l, c1, 0.0))

            for l in self.tripod_A:
                tibia = self.step_height * math.sin(math.pi * alpha)
                P.update(self.apply_pose(l, 0.0, tibia))

        self.publish_pose(P, 1.0 / 60.0)


# ============================================================
#     GIRO HORARIO (invertido)
# ============================================================
class RotateCW(RotateCCW):
    def __init__(self, node):
        super().__init__(node)
        self.push = -0.40
        self.spread = {"A": +0.25, "B": -0.25}

# ============================================================
#     GAIT LATERAL REAL (SHEAR / CANGREJO)
# ============================================================
class LateralCrabBase(BaseGait):
    def __init__(self, node):
        joints = [
            "j_c1_rf","j_thigh_rf","j_tibia_rf",
            "j_c1_rm","j_thigh_rm","j_tibia_rm",
            "j_c1_rr","j_thigh_rr","j_tibia_rr",
            "j_c1_lf","j_thigh_lf","j_tibia_lf",
            "j_c1_lm","j_thigh_lm","j_tibia_lm",
            "j_c1_lr","j_thigh_lr","j_tibia_lr"
        ]
        super().__init__(node, joints, update_hz=90)

        # Orientación lateral fija (CRÍTICA)
        self.base_c1 = {
            "rf": -math.pi/4,
            "rm":  0.0,
            "rr": +math.pi/4,
            "lf": +math.pi/4,
            "lm":  0.0,
            "lr": -math.pi/4,
        }

        self.thigh_support = -0.75
        self.tibia = -0.85
        self.c1_yaw_trim = -0.08   # rad ≈ 3.5°

        # amplitudes REALES
        self.push_amp = 0.35
        self.pull_amp = 0.30

        self.T = 0.8
        self.start_time = node.get_clock().now()

    def pose(self, leg, thigh):
        return {
            f"j_c1_{leg}": self.base_c1[leg],
            f"j_thigh_{leg}": thigh,
            f"j_tibia_{leg}": self.tibia
        }

    def support(self, leg):
        return self.pose(leg, self.thigh_support)

    def push(self, leg, s):
        return self.pose(leg, self.thigh_support + self.push_amp * s)

    def pull(self, leg, s):
        return self.pose(leg, self.thigh_support - self.pull_amp * s)
    
class LateralLeftGait(LateralCrabBase):
    def update(self):
        if not self.enabled or not self.received:
            return

        t = (self.node.get_clock().now() - self.start_time).nanoseconds / 1e9
        phi = (t % self.T) / self.T
        s = math.sin(math.pi * (phi * 2 if phi < 0.5 else (phi - 0.5) * 2))

        P = {}
        yaw_trim = -self.c1_yaw_trim   # corrige giro horario

        if phi < 0.5:
            P.update(self.pose("rf", self.thigh_support + self.push_amp * s))
            P.update(self.pose("rr", self.thigh_support + self.push_amp * s))
            P.update(self.pose("lm", self.thigh_support - self.pull_amp * s))

            P["j_c1_rf"] += yaw_trim
            P["j_c1_rr"] += yaw_trim

            for leg in ["lf", "lr", "rm"]:
                P.update(self.support(leg))

        else:
            P.update(self.pose("rm", self.thigh_support + self.push_amp * s))
            P.update(self.pose("lf", self.thigh_support - self.pull_amp * s))
            P.update(self.pose("lr", self.thigh_support - self.pull_amp * s))

            P["j_c1_rm"] += yaw_trim

            for leg in ["lm", "rf", "rr"]:
                P.update(self.support(leg))

        self.publish_pose(P, 1.0 / 90.0)

class LateralRightGait(LateralCrabBase):
    def update(self):
        if not self.enabled or not self.received:
            return

        t = (self.node.get_clock().now() - self.start_time).nanoseconds / 1e9
        phi = (t % self.T) / self.T
        s = math.sin(math.pi * (phi * 2 if phi < 0.5 else (phi - 0.5) * 2))

        P = {}
        yaw_trim = +self.c1_yaw_trim   # corrige giro antihorario

        if phi < 0.5:
            P.update(self.pose("rf", self.thigh_support - self.pull_amp * s))
            P.update(self.pose("rr", self.thigh_support - self.pull_amp * s))
            P.update(self.pose("lm", self.thigh_support + self.push_amp * s))

            P["j_c1_lm"] += yaw_trim

            for leg in ["lf", "lr", "rm"]:
                P.update(self.support(leg))

        else:
            P.update(self.pose("rm", self.thigh_support - self.pull_amp * s))
            P.update(self.pose("lf", self.thigh_support + self.push_amp * s))
            P.update(self.pose("lr", self.thigh_support + self.push_amp * s))

            P["j_c1_lf"] += yaw_trim
            P["j_c1_lr"] += yaw_trim

            for leg in ["lm", "rf", "rr"]:
                P.update(self.support(leg))

        self.publish_pose(P, 1.0 / 90.0)

# ============================================================
#       NODO PRINCIPAL: PUENTE DE MOVIMIENTO
# ============================================================
class HLHexapodBridge(Node):
    def __init__(self):
        super().__init__("gz_hexapod_inv_kinematics")

        self.create_subscription(String, "/hl_cmd", self.cb_cmd, 10)

        # Instancia de gaits
        self.g_forward = TripodForwardGait(self)
        self.g_backward = TripodBackwardGait(self)
        self.g_ccw     = RotateCCW(self)
        self.g_cw      = RotateCW(self)
        self.g_left  = LateralLeftGait(self)
        self.g_right = LateralRightGait(self)

        self.active = None

    def disable_all(self):
        for g in (self.g_forward, self.g_backward, self.g_ccw, self.g_cw, self.g_left, self.g_right):
            g.disable()

    def cb_cmd(self, msg):
        cmd = msg.data.strip()
        #self.get_logger().info(f"[HL_CMD] {cmd}")

        if cmd == "forward":
            self.disable_all()
            self.g_forward.enable()
            self.active = self.g_forward

        elif cmd == "backward":
            self.disable_all()
            self.g_backward.enable()
            self.active = self.g_backward

        elif cmd == "turn_right":   # CCW
            self.disable_all()
            self.g_ccw.enable()
            self.active = self.g_ccw

        elif cmd == "turn_left":  # CW
            self.disable_all()
            self.g_cw.enable()
            self.active = self.g_cw

        elif cmd == "lateral_left":
            self.disable_all()
            self.g_left.enable()
            self.active = self.g_left

        elif cmd == "lateral_right":
            self.disable_all()
            self.g_right.enable()
            self.active = self.g_right

        elif cmd == "stop":
            self.disable_all()
            self.active = None

        else:
            #self.get_logger().warn("Comando no conocido.")
            pass


def main(args=None):
    rclpy.init(args=args)
    node = HLHexapodBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
