#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from hexapod_pkg import hw_config as cfg

import csv
import os
from datetime import datetime

class SwarmNav(Node):
    def __init__(self):
        super().__init__("swarm_nav")

        # ================= PARÁMETROS =================
        self.declare_parameter("topic_obstacle_occupation_grid", cfg.TOPIC_OBSTACLE_OCCUPATION_GRID)
        self.declare_parameter("topic_cmd_robot", cfg.TOPIC_CMD_GZ_ROBOT)
        self.declare_parameter("center_tol_cols", 1)
        self.declare_parameter("search_cmd", "turn_left")
        
        # Parámetros de Seguridad y Evasión
        self.declare_parameter("danger_rows_count", 10)
        self.declare_parameter("min_active_cells_proximity", 10)
        # Tiempo de giro forzado al encontrar una pelota (en ciclos de 0.05s)
        # 20 ciclos = 1 segundo de giro ignorando detecciones
        self.declare_parameter("evasion_duration_cycles", 20) 

        topic_grid = self.get_parameter("topic_obstacle_occupation_grid").value
        topic_cmd = self.get_parameter("topic_cmd_robot").value
        
        self.center_tol = self.get_parameter("center_tol_cols").value
        self.search_cmd = self.get_parameter("search_cmd").value
        self.danger_rows = self.get_parameter("danger_rows_count").value
        self.min_prox_cells = self.get_parameter("min_active_cells_proximity").value
        self.evasion_limit = self.get_parameter("evasion_duration_cycles").value

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        # ================= CONFIGURACIÓN CSV =================
        home = os.path.expanduser("~")
        self.csv_dir = os.path.join(home, "csv_logs")
        if not os.path.exists(self.csv_dir):
            os.makedirs(self.csv_dir)

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = os.path.join(self.csv_dir, f"swarm_log_{timestamp_str}.csv")
        self.init_csv()

        # ================= SUBS / PUB =================
        self.create_subscription(Int8MultiArray, topic_grid, self.cb_grid, qos)
        self.cmd_pub = self.create_publisher(String, topic_cmd, 10)

        # ================= ESTADO =================
        self.grid = None
        self.N = None
        self.is_object_near = False
        self.evasion_timer = 0  # Contador para el giro forzado

        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info("SwarmNav con Modo Evasión iniciado.")

    def init_csv(self):
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_stamp', 'target_col', 'object_near', 'evading'])

    def cb_grid(self, msg):
        self.N = msg.layout.dim[0].size
        self.grid = list(msg.data)

    def find_centroid_column(self):
        if self.grid is None: return None
        N = self.N
        total_weight = 0
        weighted_sum_cols = 0
        
        # 1. Centroide
        for c in range(N):
            col_density = sum(1 for r in range(N) if self.grid[r * N + c] == 1)
            if col_density > 0:
                weighted_sum_cols += c * col_density
                total_weight += col_density

        # 2. Zona de Peligro (Proximidad)
        active_near_cells = 0
        for r in range(N - self.danger_rows, N):
            for c in range(N):
                if self.grid[r * N + c] == 1:
                    active_near_cells += 1
        
        self.is_object_near = active_near_cells >= self.min_prox_cells
        target_col = int(weighted_sum_cols / total_weight) if total_weight > 0 else None
        
        # Log CSV reducido
        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            time_now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            writer.writerow([time_now, target_col, self.is_object_near, self.evasion_timer > 0])
            
        return target_col

    def loop(self):
        if self.grid is None: return
        cmd = String()

        # --- LÓGICA DE EVASIÓN FORZADA ---
        if self.evasion_timer > 0:
            cmd.data = self.search_cmd
            self.cmd_pub.publish(cmd)
            self.evasion_timer -= 1
            return

        # Si no estamos evadiendo, procesamos la matriz normalmente
        col = self.find_centroid_column()

        # --- DISPARADOR DE EVASIÓN ---
        if self.is_object_near:
            self.get_logger().info("¡Pelota alcanzada! Iniciando búsqueda de nueva pelota...")
            self.evasion_timer = self.evasion_limit
            cmd.data = self.search_cmd
            self.cmd_pub.publish(cmd)
            return

        # --- BUSQUEDA NORMAL (Sin detecciones) ---
        if col is None:
            cmd.data = self.search_cmd
            self.cmd_pub.publish(cmd)
            return

        # --- SEGUIMIENTO ---
        center = self.N // 2
        err = col - center

        if abs(err) <= self.center_tol:
            cmd.data = "forward"
        elif err < 0:
            cmd.data = "turn_left"
        else:
            cmd.data = "turn_right"

        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = SwarmNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()