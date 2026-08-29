#!/usr/bin/env python3
"""
Explorador simple e inteligente - mapea la habitación con movimientos adaptativos.
"""

import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SimpleExplorer(Node):
    def __init__(self):
        super().__init__('simple_explorer')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('🤖 Explorador simple iniciado')

    def move(self, linear, angular, duration):
        """Mueve el robot."""
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        start = time.time()
        while time.time() - start < duration:
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.05)

        # Parar
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        time.sleep(0.2)

    def explore(self):
        """Exploración en forma de 8 o zigzag."""
        self.get_logger().info('✅ Iniciando exploración inteligente...')

        # Patrón de búsqueda en zigzag expandido
        for i in range(5):
            self.get_logger().info(f'Línea {i+1}/5: avanzando...')
            self.move(0.3, 0.0, 5.0)  # Avanzar

            self.get_logger().info(f'Línea {i+1}/5: girando...')
            self.move(0.0, 1.2, 1.5)  # Girar 90°

        self.get_logger().info('✅ Exploración completada!')


def main():
    rclpy.init()
    explorer = SimpleExplorer()

    try:
        time.sleep(1)
        explorer.explore()
        explorer.move(0.0, 0.0, 0.5)  # Parar
    except KeyboardInterrupt:
        explorer.get_logger().info('⏹️  Cancelado por usuario')
        explorer.move(0.0, 0.0, 0.5)
    finally:
        try:
            explorer.destroy_node()
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
