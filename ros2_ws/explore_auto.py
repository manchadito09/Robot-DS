#!/usr/bin/env python3
"""
Explorador automático inteligente con detección de obstáculos.
Funciona dentro del contexto de SLAM ya lanzado.
"""

import rclpy
import time
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class AutoExplorer(Node):
    def __init__(self):
        super().__init__('auto_explorer')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

        self.min_dist = 100
        self.front_dist = 100
        self.left_dist = 100
        self.right_dist = 100
        self.get_logger().info('🤖 Explorador automático iniciado')

    def on_scan(self, msg):
        """Procesa datos del lidar."""
        ranges = msg.ranges
        n = len(ranges)

        # Distancia mínima
        valid = [r for r in ranges if 0.1 < r < 30]
        self.min_dist = min(valid) if valid else 100

        # Frontal (centro)
        front_idx = n // 2
        self.front_dist = ranges[front_idx] if 0.1 < ranges[front_idx] < 30 else 100

        # Izquierda
        left_idx = int(n * 0.25)
        self.left_dist = ranges[left_idx] if 0.1 < ranges[left_idx] < 30 else 100

        # Derecha
        right_idx = int(n * 0.75)
        self.right_dist = ranges[right_idx] if 0.1 < ranges[right_idx] < 30 else 100

    def move(self, linear, angular, duration):
        """Envía comando de movimiento."""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular

        start = time.time()
        while time.time() - start < duration:
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.05)

        # Parar
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        for _ in range(5):
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.05)

    def explore(self):
        """Exploración inteligente con evitación de obstáculos."""
        self.get_logger().info('Esperando datos del lidar...')
        time.sleep(2)

        self.get_logger().info('✅ Iniciando exploración automática')

        step = 0
        while step < 200:
            step += 1

            # Log actual
            self.get_logger().info(
                f'[{step}] F:{self.front_dist:.2f} L:{self.left_dist:.2f} R:{self.right_dist:.2f}m'
            )

            # Lógica de movimiento
            if self.front_dist < 0.5:
                # Obstáculo frontal - retroceder y girar
                self.get_logger().info('⚠️  Obstáculo frontal! Retrocediendo...')
                self.move(-0.25, 0.0, 1.0)  # Atrás
                self.move(0.0, 1.5, 1.2)    # Girar izquierda
            elif self.front_dist < 0.8:
                # Cerca de obstáculo - avanzar lentamente
                self.get_logger().info('🚶 Avanzando lentamente')
                self.move(0.15, 0.0, 0.8)
            else:
                # Libre - avanzar rápido
                self.get_logger().info('🏃 Avanzando rápido')
                self.move(0.35, 0.0, 1.5)

                # A veces girar para explorar
                if step % 8 == 0:
                    self.get_logger().info('↻ Girando para explorar')
                    self.move(0.0, 0.8, 0.8)

        self.get_logger().info('✅ Exploración completada!')


def main():
    rclpy.init()

    try:
        explorer = AutoExplorer()
        explorer.explore()
        explorer.move(0.0, 0.0, 0.5)  # Parar
    except KeyboardInterrupt:
        print('\n⏹️  Cancelado por usuario')
    except Exception as e:
        print(f'❌ Error: {e}')
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
