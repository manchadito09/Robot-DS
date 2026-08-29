#!/usr/bin/env python3
# scan_vs_map.py - read-only. Does what the lidar SEES agree with where AMCL THINKS it is?
#
# The robot keeps driving into the Cristal room glass, and it is mislocalized when it does -- "close
# but not exactly right", always at the same spot. Glass is the obvious suspect (a lidar shoots
# straight through it), but "obvious" has been wrong three times on this robot already. So measure.
#
# For every beam: take its endpoint, put it on the map, and ask how far it is from the nearest
# mapped wall. If AMCL's pose is right and the map is honest, endpoints land ON walls (a few cm).
# Endpoints far from any wall are beams the map cannot explain -- and those are what drag AMCL away
# from the truth.
#
#   Park the robot in the corridor, FACING THE GLASS, standing still, then:
#     python3 ~/ros2_ws/scan_vs_map.py
#
# Reads /scan, /amcl_pose and the map; uses TF for the lidar's mounting (it is yawed 180 deg).
import math
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

MAPD = os.path.expanduser('~/ros2_ws/src/slam/maps')
GOOD = 0.15      # an endpoint this close to a mapped wall is "explained"
LOST = 0.50      # this far from any wall = the map cannot explain this beam at all


def load_map():
    y = yaml.safe_load(open(f'{MAPD}/map_01.yaml'))
    f = open(f'{MAPD}/map_01.pgm', 'rb')
    assert f.readline().strip() == b'P5'
    l = f.readline()
    while l.startswith(b'#'):
        l = f.readline()
    w, h = map(int, l.split())
    int(f.readline())
    img = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
    import cv2
    free = (img > 100).astype(np.uint8)          # everything that is NOT a wall
    dist = cv2.distanceTransform(free, cv2.DIST_L2, 5) * y['resolution']   # m to nearest wall
    return dist, y['resolution'], y['origin'][0], y['origin'][1], h, w


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    dist, res, ox, oy, H, W = load_map()
    rclpy.init()
    node = Node('scan_vs_map')
    tfb = Buffer()
    TransformListener(tfb, node)
    scans = []
    node.create_subscription(LaserScan, '/scan', lambda m: scans.append(m), qos_profile_sensor_data)

    print('waiting for /scan and TF (map -> lidar) ...')
    t0 = time.time()
    tf = None
    while time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.2)
        if scans:
            try:
                tf = tfb.lookup_transform('map', scans[-1].header.frame_id, rclpy.time.Time())
                break
            except Exception:
                pass
    if tf is None or not scans:
        sys.exit('no /scan or no TF map->lidar. Is navigation up and the robot localized?')

    m = scans[-1]
    px, py = tf.transform.translation.x, tf.transform.translation.y
    pth = yaw_of(tf.transform.rotation)
    print(f'lidar in map: x={px:.2f} y={py:.2f} yaw={math.degrees(pth):.1f} deg\n')

    rng = np.array(m.ranges, dtype=np.float32)
    ang = m.angle_min + np.arange(len(rng)) * m.angle_increment
    ok = np.isfinite(rng) & (rng > m.range_min) & (rng < min(m.range_max, 11.9))
    rng, ang = rng[ok], ang[ok]

    ex = px + rng * np.cos(pth + ang)             # endpoint in the map frame
    ey = py + rng * np.sin(pth + ang)
    c = ((ex - ox) / res).astype(int)
    r = (H - 1 - ((ey - oy) / res)).astype(int)
    inside = (c >= 0) & (c < W) & (r >= 0) & (r < H)
    d = np.full(len(rng), 9.9, dtype=np.float32)
    d[inside] = dist[r[inside], c[inside]]        # m from the endpoint to the nearest mapped wall

    n = len(rng)
    good = (d <= GOOD).sum()
    lost = (d >= LOST).sum()
    print(f'{n} beams used\n')
    print(f'  land ON a mapped wall  (< {GOOD:.2f} m) : {good:4d}  ({100*good/n:4.1f}%)   <- explained')
    print(f'  land in EMPTY map space (> {LOST:.2f} m) : {lost:4d}  ({100*lost/n:4.1f}%)   <- the map cannot explain these')
    print(f'  median miss: {np.median(d):.2f} m')
    print()
    if 100 * good / n > 80:
        print('  VERDICT: the lidar AGREES with the map here. Localization is not the problem.')
    elif 100 * lost / n > 25:
        print('  VERDICT: a LOT of beams land where the map says empty floor. AMCL is being pulled')
        print('           away from the truth by beams it cannot explain. This is the poisoning.')
    else:
        print('  VERDICT: mixed. Look at the per-direction table below.')

    print('\n  WHERE the beams disagree (robot-relative; 0 = straight ahead):')
    print('  sector        beams   land on wall   median miss')
    for lo, hi, name in [(-180, -135, 'behind R'), (-135, -90, 'right-back'), (-90, -45, 'right'),
                         (-45, 0, 'right-front'), (0, 45, 'left-front'), (45, 90, 'left'),
                         (90, 135, 'left-back'), (135, 180, 'behind L')]:
        a = np.degrees(ang)
        sel = (a >= lo) & (a < hi)
        if sel.sum() == 0:
            continue
        g = 100 * (d[sel] <= GOOD).sum() / sel.sum()
        flag = '  <-- 🔴' if g < 50 else ''
        print(f'  {name:12s} {sel.sum():5d}      {g:5.1f}%        {np.median(d[sel]):.2f} m{flag}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
