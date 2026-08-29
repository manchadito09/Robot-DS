#!/usr/bin/env python3
# pose_check.py - read-only. Is the robot REALLY where AMCL thinks it is?
#
# WHY THIS EXISTS
#
# Navigation starts AMCL already parked at the 'Base' waypoint, so nobody has to tell the robot
# where it is -- no RViz "2D Pose Estimate", no tapping the map on the phone. A visitor should never
# have to localize a robot.
#
# But AMCL BELIEVES that pose whether or not it is true. Push the robot into a corner, start
# navigation, and AMCL is confidently, precisely wrong -- and it says nothing. Worse, the web app
# reported "localized: true" the whole time, because all that ever meant was "an /amcl_pose message
# arrived". AMCL always sends one. Even a wrong one. So demo_check said READY and the robot set off
# planning against the walls of a different room.
#
# So: check the belief against the world. For every lidar beam, take its endpoint, put it on the map
# at the pose AMCL claims, and ask how far it is from the nearest mapped wall. If the belief is
# right, the endpoints land ON walls. If the robot is somewhere else, they land in open space.
#
# This is the same measurement scan_vs_map.py makes (it is what caught the glass wall: 31.5% of
# beams explained when the map was lying, 93.2% once it was fixed). This one just answers yes/no.
#
#   python3 ~/ros2_ws/pose_check.py          # exit 0 = the robot is where it thinks it is
#
#   exit 0  AGREES     -- go
#   exit 1  DISAGREES  -- the robot is NOT where AMCL thinks. Park it at Base and relaunch.
#   exit 2  CANNOT TELL -- no /scan, or no TF map->lidar. Is navigation up? Is the battery in?
#                          (no battery -> no lidar -> nothing to check with)
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
WAYPOINTS = f'{MAPD}/map_01.waypoints.yaml'
GOOD = 0.15       # an endpoint this close to a mapped wall counts as "near" one (same as scan_vs_map)

# ON a wall, not just near one. This is the number the verdict is made on -- see the long comment
# further down. A lidar is good to a couple of centimetres, so a beam that sees real wall from a
# correct pose lands within about five. Being lost moves that spike; a room full of people only
# makes it smaller.
SHARP = 0.05
SHARP_PCT = float(os.environ.get('POSE_CHECK_MIN', '25'))   # % of beams that must land ON a wall

# The lidar is bolted on YAWED 180 DEGREES. The beam maths never cared -- it uses the same transform
# for the robot and the beams -- but printing the lidar's heading as if it were the robot's made this
# tool report -36 deg while the robot sat at Base facing 144, and cost an hour hunting a bug that
# was not there. Print what a human can check against the waypoint.
LIDAR_YAW_DEG = 180.0

# Below this share of explained beams, the robot is not where it thinks it is.
#
# Calibrate it, do not guess it: park the robot at Base, run this, and read the number it prints. A
# healthy pose on this map scores in the 90s (the corridor by the glass measured 93.2% once the map
# told the truth). A mislocalized one collapses -- the same spot scored 31.5% when AMCL was adrift.
# There is a wide, empty gap between those two, and 70 sits in the middle of it: high enough to
# catch a robot in the wrong room, low enough to survive a chair that moved since the map was made.
AGREE_PCT = float(os.environ.get('POSE_CHECK_MIN', '70'))

SCANS = 5         # judge on the median of a few sweeps, so one bad frame cannot fail the check


def load_map():
    y = yaml.safe_load(open(f'{MAPD}/map_01.yaml'))
    f = open(f'{MAPD}/map_01.pgm', 'rb')
    assert f.readline().strip() == b'P5'
    line = f.readline()
    while line.startswith(b'#'):
        line = f.readline()
    w, h = map(int, line.split())
    int(f.readline())
    img = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
    import cv2
    free = (img > 100).astype(np.uint8)                                    # everything NOT a wall
    dist = cv2.distanceTransform(free, cv2.DIST_L2, 5) * y['resolution']   # m to the nearest wall
    return dist, y['resolution'], y['origin'][0], y['origin'][1], h, w


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def base_waypoint():
    """Where Base is, so the printout can say how far the robot thinks it is from it.

    Not part of the verdict -- it is a sanity number for the human. Right after Navigation starts,
    AMCL believes it is at Base by construction, so "0 cm from Base" proves nothing on its own. It
    is the LIDAR that has to agree. But if the two disagree, seeing both numbers at once is what
    tells you whether the robot was parked wrong or the map is."""
    try:
        wps = (yaml.safe_load(open(WAYPOINTS)) or {}).get('waypoints') or {}
        b = wps.get('Base') or wps.get('base') or wps.get('Home')
        return (float(b['x']), float(b['y']), float(b.get('yaw', 0.0))) if b else None
    except Exception:
        return None


def beam_misses(m, tf, dist, res, ox, oy, H, W):
    """How far each beam's endpoint lands from the nearest mapped wall, in metres."""
    px, py = tf.transform.translation.x, tf.transform.translation.y
    pth = yaw_of(tf.transform.rotation)

    rng = np.array(m.ranges, dtype=np.float32)
    ang = m.angle_min + np.arange(len(rng)) * m.angle_increment
    ok = np.isfinite(rng) & (rng > m.range_min) & (rng < min(m.range_max, 11.9))
    rng, ang = rng[ok], ang[ok]
    if len(rng) < 20:                       # too few beams to say anything honest
        return None

    ex = px + rng * np.cos(pth + ang)       # endpoint, in the map frame
    ey = py + rng * np.sin(pth + ang)
    c = ((ex - ox) / res).astype(int)
    r = (H - 1 - ((ey - oy) / res)).astype(int)
    inside = (c >= 0) & (c < W) & (r >= 0) & (r < H)
    d = np.full(len(rng), 9.9, dtype=np.float32)
    d[inside] = dist[r[inside], c[inside]]
    return d


def main():
    dist, res, ox, oy, H, W = load_map()
    rclpy.init()
    node = Node('pose_check')
    tfb = Buffer()
    TransformListener(tfb, node)
    scans = []
    node.create_subscription(LaserScan, '/scan', lambda m: scans.append(m), qos_profile_sensor_data)

    # Wait for BOTH a sweep and the TF that says where the lidar is on the map. Either one missing
    # and there is nothing to compare -- which is not a pass and not a fail, it is "cannot tell".
    t0 = time.time()
    tf = None
    while time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.2)
        if scans and len(scans) >= SCANS:
            try:
                tf = tfb.lookup_transform('map', scans[-1].header.frame_id, rclpy.time.Time())
                break
            except Exception:
                pass
    if tf is None or len(scans) < SCANS:
        print('CANNOT TELL - no /scan, or no TF map->lidar.')
        print('              Is navigation running? Is the BATTERY in? (no battery -> no lidar)')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(2)

    pcts, sharps, meds = [], [], []
    for m in scans[-SCANS:]:
        d = beam_misses(m, tf, dist, res, ox, oy, H, W)
        if d is None:
            continue
        pcts.append(100.0 * (d <= GOOD).mean())
        sharps.append(100.0 * (d <= SHARP).mean())
        meds.append(float(np.median(d)))
    node.destroy_node()
    rclpy.shutdown()

    if not pcts:
        print('CANNOT TELL - the lidar is publishing, but the sweeps are empty.')
        sys.exit(2)

    pct = float(np.median(pcts))
    sharp = float(np.median(sharps))
    med = float(np.median(meds))

    # The pose we print is the ROBOT's, not the lidar's. The lidar is bolted on yawed 180 degrees,
    # so printing its heading meant this tool reported -36 deg while the robot sat at Base facing
    # 144 -- and sent a tired person hunting a localization bug that did not exist, at midnight,
    # two days before a demo. The maths never cared (the beams use the same transform either way).
    # Only the human reading it did.
    px, py = tf.transform.translation.x, tf.transform.translation.y
    yaw = math.degrees(yaw_of(tf.transform.rotation)) + LIDAR_YAW_DEG
    yaw = (yaw + 180) % 360 - 180
    print(f'the robot thinks it is at x={px:.2f} y={py:.2f} facing {yaw:.0f} deg')
    base = base_waypoint()
    if base:
        bx, by, byaw = base
        off = math.hypot(px - bx, py - by)
        dyaw = abs((yaw - math.degrees(byaw) + 180) % 360 - 180)
        print(f'Base is at        x={bx:.2f} y={by:.2f} facing {math.degrees(byaw):.0f} deg'
              f'   -> {off*100:.0f} cm and {dyaw:.0f} deg away')
    print()
    print(f'  beams landing ON a mapped wall  (<{SHARP*100:.0f} cm) : {sharp:.1f}%   '
          f'(need {SHARP_PCT:.0f}%)')
    print(f'  beams landing near one          (<{GOOD*100:.0f} cm) : {pct:.1f}%')
    print(f'  median miss                                : {med*100:.0f} cm')

    # WHY THE SHARP NUMBER AND NOT THE LOOSE ONE.
    #
    # The obvious test -- "what share of beams land near a mapped wall" -- cannot tell the two
    # failures apart, because BOTH push it down:
    #
    #   the robot is in the wrong place   -> the walls are not where it thinks
    #   the robot is in the right place, in a WORKING OFFICE -> half the beams hit people, chairs,
    #                                        bags and boxes that were not there when the map was made
    #
    # Rodrigo's office scored 64% with the robot sitting EXACTLY on Base (9 cm and 0.1 deg out), and
    # this tool called it MISLOCALIZED. It was not. It was Tuesday, and people were at their desks.
    #
    # What separates them is not how MANY beams find a wall, but how WELL the ones that do. A lidar
    # is accurate to a couple of centimetres. If the pose is right, the beams that see real wall land
    # ON it -- a hard spike under 5 cm -- no matter how many of their neighbours hit a colleague.
    # If the pose is wrong by even 20 cm, that spike is gone: every wall beam misses by 20 cm. The
    # clutter takes beams AWAY from the spike; being lost MOVES the spike.
    if sharp >= SHARP_PCT:
        print()
        print('AGREES - the robot really is where it thinks it is.')
        if pct < 60:
            print(f'         (only {pct:.0f}% of beams land near a wall, but the ones that do land ON '
                  'it -- that is a busy room, not a lost robot.)')
        sys.exit(0)

    print()
    print('DISAGREES - the robot is NOT where AMCL thinks it is.')
    print('            Even the beams that find a wall are missing it, so this is not clutter.')
    print('            Park the robot at Base and relaunch Navigation. It localizes itself.')
    print('            (If it IS at Base: the map may be stale -- run scan_vs_map.py to see where.)')
    sys.exit(1)


if __name__ == '__main__':
    main()
