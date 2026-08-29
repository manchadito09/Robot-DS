#!/usr/bin/env python3
"""
Script de exploración autónoma inteligente con evitación de obstáculos.
Lee el lidar en tiempo real y evita colisiones.

Uso:
  python3 explore.py    # Exploración automática

Presiona Ctrl+C para detener.
"""

import rclpy
import math
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class SmartExploreNode(Node):
    def __init__(self):
        super().__init__('smart_explorer')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

        self.min_distance = 0.4  # Distancia mínima a obstáculos (metros)
        self.front_distance = 0.5  # Distancia frontal segura
        self.last_scan = None
        self.obstacle_detected = False
        self.get_logger().info('Explorador inteligente iniciado. Leyendo lidar...')

    def on_scan(self, msg):
        """Procesa datos del lidar."""
        self.last_scan = msg
        ranges = msg.ranges

        # Detectar distancia mínima
        valid_ranges = [r for r in ranges if 0.1 < r < 20.0]
        if valid_ranges:
            min_dist = min(valid_ranges)

            # Detectar si hay obstáculo frontal (centro del lidar)
            front_idx = len(ranges) // 2
            front_range = ranges[front_idx] if 0.1 < ranges[front_idx] < 20.0 else 20.0

            if min_dist < self.min_distance:
                self.obstacle_detected = True
            else:
                self.obstacle_detected = False

    def send_velocity(self, linear_x, angular_z, duration=0.5):
        """Envía comando de velocidad."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        end_time = time.time() + duration
        while time.time() < end_time:
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)

        # Parar
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        time.sleep(0.1)

    def explore(self):
        """Exploración inteligente con evitación de obstáculos."""
        self.get_logger().info('Iniciando exploración inteligente...')

        move_count = 0
        max_moves = 100  # Máximo de movimientos antes de considerar terminado

        while move_count < max_moves:
            move_count += 1

            # Si hay obstáculo, retroceder y girar
            if self.obstacle_detected:
                self.get_logger().info(f'[{move_count}] ⚠️  Obstáculo detectado. Retrocediendo...')
                self.send_velocity(-0.2, 0.0, duration=1.0)  # Retroceder
                self.send_velocity(0.0, 1.2, duration=1.5)   # Girar
            else:
                # Avanzar
                self.get_logger().info(f'[{move_count}] → Avanzando...')
                self.send_velocity(0.3, 0.0, duration=2.0)

                # A veces girar para explorar
                if move_count % 5 == 0:
                    self.get_logger().info(f'[{move_count}] ↻ Girando para explorar...')
                    self.send_velocity(0.0, 0.8, duration=1.0)

        self.get_logger().info('Exploración completada.')

    def get_robot_pose(self):
        """Obtiene posición actual del robot."""
        try:
            from tf2_ros import Buffer, TransformListener
            from rclpy.time import Time
            from rclpy.duration import Duration

            tf_buffer = Buffer()
            tf_listener = TransformListener(tf_buffer, self)
            t = tf_buffer.lookup_transform('map', 'base_footprint', Time(), timeout=Duration(seconds=1.0))
            x = t.transform.translation.x
            y = t.transform.translation.y
            return x, y
        except:
            return None


def main():
    rclpy.init()
    node = SmartExploreNode()

    try:
        # Esperar a recibir primer scan
        node.get_logger().info('Esperando datos del lidar...')
        time.sleep(2)

        if node.last_scan is None:
            node.get_logger().error('No se reciben datos del lidar. ¿Está funcionando?')
            return

        node.get_logger().info('✅ Lidar detectado. Iniciando exploración...')
        node.explore()

        node.get_logger().info('Deteniendo robot...')
        node.send_velocity(0.0, 0.0, duration=0.5)

    except KeyboardInterrupt:
        node.get_logger().info('Exploración cancelada por el usuario.')
        node.send_velocity(0.0, 0.0, duration=0.5)
    except Exception as e:
        print(f'Error: {e}')
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
