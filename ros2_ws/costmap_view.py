#!/usr/bin/env python3
# costmap_view.py - read-only. DRAW what TEB actually sees.
#
# The robot stopped being blind to the glass and started being scared of it: it now halts at the
# glass corner and takes the long way round a gap it obviously fits through. Both say the same
# thing -- the local costmap looks fuller than the world is. Guessing which layer is filling it has
# already cost us a day, so: draw it.
#
# The local costmap is a 3x3 m rolling window centred on the robot, and it is EXACTLY what the
# controller plans in. Whatever is black here is a wall as far as TEB is concerned, whether or not
# anything is really there.
#
#   Park the robot where it gets stuck, then:
#     python3 ~/ros2_ws/costmap_view.py
#
# It also draws the STATIC MAP around the same spot, so you can tell the two apart:
#   a wall in both  -> real (or at least mapped)
#   wall only in the costmap -> a live sensor put it there: a lidar return, a camera mark, or a
#                               GHOST (camera_layer has clearing: False -- its marks never expire)
#   wall only in the map     -> AMCL has slid: the map wall is not where the costmap thinks it is
import math
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

MAPD = os.path.expanduser('~/ros2_ws/src/slam/maps')
LETHAL = 90        # cost >= this = TEB treats it as a wall
INFLATED = 40      # cost in between = expensive but passable


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def load_static():
    y = yaml.safe_load(open(f'{MAPD}/map_01.yaml'))
    f = open(f'{MAPD}/map_01.pgm', 'rb')
    assert f.readline().strip() == b'P5'
    l = f.readline()
    while l.startswith(b'#'):
        l = f.readline()
    w, h = map(int, l.split())
    int(f.readline())
    img = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
    return img, y['resolution'], y['origin'][0], y['origin'][1], h, w


def main():
    smap, sres, sox, soy, SH, SW = load_static()
    rclpy.init()
    node = Node('costmap_view')
    tfb = Buffer()
    TransformListener(tfb, node)
    grids = []
    qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: grids.append(m), qos)

    t0 = time.time()
    pose = None
    while time.time() - t0 < 12:
        rclpy.spin_once(node, timeout_sec=0.2)
        if grids:
            try:
                tr = tfb.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                pose = (tr.transform.translation.x, tr.transform.translation.y,
                        yaw_of(tr.transform.rotation))
                break
            except Exception:
                pass
    if not grids or pose is None:
        sys.exit('no /local_costmap/costmap or no TF map->base_footprint. Is navigation up?')

    g = grids[-1]
    W, H, res = g.info.width, g.info.height, g.info.resolution
    ox, oy = g.info.origin.position.x, g.info.origin.position.y
    data = np.array(g.data, dtype=np.int16).reshape(H, W)

    # where the robot is inside the rolling window (the window is in the odom frame)
    try:
        tro = tfb.lookup_transform(g.header.frame_id, 'base_footprint', rclpy.time.Time())
        rx, ry = tro.transform.translation.x, tro.transform.translation.y
        rth = yaw_of(tro.transform.rotation)
    except Exception:
        sys.exit('no TF to the costmap frame (%s)' % g.header.frame_id)
    rc = int((rx - ox) / res)
    rr = int((ry - oy) / res)

    print(f'robot in map: x={pose[0]:.2f} y={pose[1]:.2f} yaw={math.degrees(pose[2]):.0f} deg')
    print(f'local costmap: {W}x{H} cells @ {res:.2f} m = {W*res:.1f} x {H*res:.1f} m\n')

    S = 2                                   # cells per character
    print('  WHAT TEB SEES (the local costmap).  R = robot, arrow = where it is pointing')
    print('  # wall (TEB will not cross)   + expensive   . free   ? unknown\n')
    rows = []
    for r in range(H - 1, -1, -S):          # costmap row 0 is the BOTTOM
        line = '  '
        for c in range(0, W, S):
            blk = data[max(0, r - S + 1):r + 1, c:c + S]
            if blk.size == 0:
                continue
            if abs(r - rr) <= S and abs(c - rc) <= S:
                line += 'R'
                continue
            if (blk >= LETHAL).any():
                line += '#'
            elif (blk >= INFLATED).any():
                line += '+'
            elif (blk < 0).all():
                line += '?'
            else:
                line += '.'
        rows.append(line)
    for l in rows:
        print(l)

    # the same patch of the STATIC map, for comparison
    print('\n  THE MAP ALONE (no sensors) around the same spot:\n')
    half_m = (W * res) / 2.0
    for dy in np.arange(half_m, -half_m, -S * res):
        line = '  '
        for dx in np.arange(-half_m, half_m, S * res):
            wx, wy = pose[0] + dx, pose[1] + dy
            c = int((wx - sox) / sres); r = SH - 1 - int((wy - soy) / sres)
            if not (0 <= r < SH and 0 <= c < SW):
                line += ' '
            elif abs(dx) < S * res and abs(dy) < S * res:
                line += 'R'
            elif smap[r, c] < 100:
                line += '#'
            elif smap[r, c] > 230:
                line += '.'
            else:
                line += '?'
        print(line)

    n_leth = int((data >= LETHAL).sum())
    print(f'\n  lethal cells in the window: {n_leth} of {W*H}  ({100*n_leth/(W*H):.0f}%)')
    print('\n  Read it like this:')
    print('    wall in BOTH pictures       -> real, or at least mapped. TEB is right to stop.')
    print('    wall ONLY in the costmap    -> a sensor put it there. A lidar return, a camera mark,')
    print('                                   or a GHOST (camera_layer never clears its marks).')
    print('    wall ONLY in the map        -> AMCL has slid: the two pictures are out of register,')
    print('                                   and TEB is dodging a wall that is not where it thinks.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
