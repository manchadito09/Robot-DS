#!/usr/bin/env python3
# encoding: utf-8
# Herramienta de waypoints con nombre para el JetRover (Nav2).
# Guarda sitios del mapa con un nombre y manda el robot a ellos por su nombre.
#
# Uso (con la navegacion ya lanzada):
#   python3 ~/ros2_ws/waypoint_tool.py pick cocina      # marca "cocina" haciendo clic en el mapa (RViz "Publish Point")
#   python3 ~/ros2_ws/waypoint_tool.py save cocina      # guarda la pos ACTUAL del robot como "cocina"
#   python3 ~/ros2_ws/waypoint_tool.py list             # lista los puntos guardados
#   python3 ~/ros2_ws/waypoint_tool.py go cocina        # manda el robot a "cocina"
#   python3 ~/ros2_ws/waypoint_tool.py del cocina       # borra "cocina"
#
# Opciones:
#   --map <nombre>   mapa al que pertenecen los puntos (por defecto: map_01)
import os
import sys
import math
import time
import argparse

import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
from action_msgs.srv import CancelGoal
from tf2_ros import Buffer, TransformListener

MAPS_DIR = os.path.expanduser('~/ros2_ws/src/slam/maps')
MAP_FRAME = 'map'
BASE_FRAME = 'base_footprint'


def wp_path(map_name):
    return os.path.join(MAPS_DIR, '%s.waypoints.yaml' % map_name)


def load_wp(map_name):
    path = wp_path(map_name)
    if not os.path.exists(path):
        return {}
    with open(path, 'r') as f:
        data = yaml.safe_load(f) or {}
    return data.get('waypoints', {})


def save_wp(map_name, waypoints):
    path = wp_path(map_name)
    with open(path, 'w') as f:
        yaml.safe_dump({'map': map_name, 'waypoints': waypoints},
                       f, allow_unicode=True, sort_keys=True)


def yaw_to_quat(yaw):
    return {'z': math.sin(yaw / 2.0), 'w': math.cos(yaw / 2.0)}


def quat_to_yaw(z, w):
    return 2.0 * math.atan2(z, w)


class WaypointTool(Node):
    def __init__(self):
        super().__init__('waypoint_tool')
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self._clicked = None
        self.create_subscription(PointStamped, '/clicked_point', self._on_click, 1)
        # spin_thread=True: un hilo propio mantiene lleno el buffer de TF y
        # procesa las suscripciones, asi el lookup con timeout puede esperar a
        # una transformacion coherente (map->odom de AMCL y odom->base del EKF
        # llegan con timestamps distintos y si no se espera da ExtrapolationException).
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

    def _on_click(self, msg):
        self._clicked = (msg.point.x, msg.point.y)

    def wait_click(self, timeout=120.0):
        """Espera un clic de la herramienta 'Publish Point' de RViz. Devuelve (x, y) o None."""
        self._clicked = None
        deadline = timeout
        while deadline > 0:
            if self._clicked is not None:
                return self._clicked
            time.sleep(0.2)
            deadline -= 0.2
        return None

    def get_pose(self, timeout=10.0):
        """Devuelve (x, y, yaw) del robot en el frame del mapa, o None si no hay TF."""
        deadline = timeout
        while deadline > 0:
            try:
                t = self.tf_buffer.lookup_transform(
                    MAP_FRAME, BASE_FRAME, Time(), timeout=Duration(seconds=0.3))
                tr = t.transform.translation
                rot = t.transform.rotation
                return tr.x, tr.y, quat_to_yaw(rot.z, rot.w)
            except Exception:
                time.sleep(0.2)
                deadline -= 0.5
        return None

    def send_goal(self, x, y, yaw, timeout=10.0):
        msg = PoseStamped()
        msg.header.frame_id = MAP_FRAME
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        q = yaw_to_quat(yaw)
        msg.pose.orientation.z = q['z']
        msg.pose.orientation.w = q['w']
        # Esperar a que Nav2 (bt_navigator) descubra el publicador, si no el
        # mensaje se pierde porque el nodo se cierra antes de que conecten.
        deadline = timeout
        while self.goal_pub.get_subscription_count() < 1 and deadline > 0:
            time.sleep(0.2)
            deadline -= 0.2
        if self.goal_pub.get_subscription_count() < 1:
            return False
        # publicar varias veces y dejar tiempo a que salga por la red
        for _ in range(5):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.goal_pub.publish(msg)
            time.sleep(0.2)
        return True

    def stop(self, timeout=4.0):
        """Para el robot: cancela TODOS los goals de Nav2 y manda velocidad cero.
        Util cuando se 'vuelve loco' (goal inalcanzable, dentro de un obstaculo...)."""
        cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        cancelled = False
        cli = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        if cli.wait_for_service(timeout_sec=timeout):
            # request vacio (goal_info a cero) = cancela TODOS los goals activos
            fut = cli.call_async(CancelGoal.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
            cancelled = fut.done() and fut.result() is not None
        # mandar velocidad cero repetidamente para asegurar que frena de verdad
        zero = Twist()
        for _ in range(20):
            cmd_pub.publish(zero)
            time.sleep(0.05)
        return cancelled


def main():
    parser = argparse.ArgumentParser(description='Waypoints con nombre para el JetRover')
    parser.add_argument('cmd', choices=['pick', 'save', 'list', 'go', 'del', 'stop'])
    parser.add_argument('name', nargs='?', default=None)
    parser.add_argument('--map', default='map_01')
    args = parser.parse_args()

    waypoints = load_wp(args.map)

    if args.cmd == 'list':
        if not waypoints:
            print('No hay puntos guardados para el mapa "%s".' % args.map)
        else:
            print('Puntos en el mapa "%s":' % args.map)
            for n, p in sorted(waypoints.items()):
                print('  - %-15s x=%.2f  y=%.2f  yaw=%.1f deg'
                      % (n, p['x'], p['y'], math.degrees(p['yaw'])))
        return

    if args.cmd == 'del':
        if not args.name:
            sys.exit('Falta el nombre: python3 waypoint_tool.py del <nombre>')
        if args.name in waypoints:
            del waypoints[args.name]
            save_wp(args.map, waypoints)
            print('Borrado "%s".' % args.name)
        else:
            print('No existe "%s".' % args.name)
        return

    if args.cmd != 'stop' and not args.name:
        sys.exit('Falta el nombre: python3 waypoint_tool.py %s <nombre>' % args.cmd)

    rclpy.init()
    node = WaypointTool()
    exit_code = 0
    try:
        if args.cmd == 'stop':
            ok = node.stop()
            if ok:
                print('Robot detenido (goal de Nav2 cancelado).')
            else:
                print('Robot detenido (velocidad a cero). Si seguia moviendose, '
                      '¿esta Nav2 lanzado?')
        elif args.cmd == 'pick':
            print('Marca "%s": en RViz pulsa el boton "Publish Point" y haz clic '
                  'en el mapa donde quieras la ubicacion...' % args.name)
            click = node.wait_click()
            if click is None:
                sys.exit('No llego ningun clic (topic /clicked_point). '
                         '¿Usaste la herramienta "Publish Point" de RViz?')
            x, y = click
            waypoints[args.name] = {'x': round(x, 3), 'y': round(y, 3), 'yaw': 0.0}
            save_wp(args.map, waypoints)
            print('Guardado "%s" en x=%.2f y=%.2f (orientacion 0; cambiala con "go" '
                  'si hace falta).' % (args.name, x, y))

        elif args.cmd == 'save':
            pose = node.get_pose()
            if pose is None:
                sys.exit('No pude leer la posicion del robot (TF %s->%s).\n'
                         '¿Esta la navegacion lanzada y el robot localizado '
                         '(2D Pose Estimate puesto en RViz)?' % (MAP_FRAME, BASE_FRAME))
            x, y, yaw = pose
            waypoints[args.name] = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 4)}
            save_wp(args.map, waypoints)
            print('Guardado "%s": x=%.2f y=%.2f yaw=%.1f deg'
                  % (args.name, x, y, math.degrees(yaw)))

        elif args.cmd == 'go':
            if args.name not in waypoints:
                sys.exit('No existe "%s". Usa "list" para ver los puntos.' % args.name)
            p = waypoints[args.name]
            ok = node.send_goal(p['x'], p['y'], p['yaw'])
            if not ok:
                sys.exit('Nadie esta escuchando /goal_pose: ¿esta la navegacion '
                         '(Nav2/bt_navigator) lanzada y el robot localizado?')
            print('Enviado el robot a "%s" (x=%.2f y=%.2f). Sigue el progreso en RViz.'
                  % (args.name, p['x'], p['y']))
    except SystemExit as e:
        # sys.exit('mensaje') -> imprimir el mensaje nosotros (os._exit lo saltaria)
        if isinstance(e.code, str):
            print(e.code, file=sys.stderr)
            exit_code = 1
        elif e.code:
            exit_code = e.code
    finally:
        # Salida limpia: el hilo del TransformListener lanzaria un traceback
        # (ExternalShutdownException) al hacer rclpy.shutdown(); con os._exit
        # cerramos el proceso de golpe tras imprimir el resultado.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(exit_code)


if __name__ == '__main__':
    main()
