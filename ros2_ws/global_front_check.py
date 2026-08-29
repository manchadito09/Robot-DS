#!/usr/bin/env python3
# global_front_check.py - read-only. Is the chair in the GLOBAL costmap just ahead of the
# robot? The global costmap is in the map frame, so we get the robot pose from TF
# (map->base_footprint), look at the cells 0.3-1.2 m in front along the robot's heading,
# and report the nearest lethal cell. If it's there, the planner can route around it.
import numpy as np
import rclpy
import time
import math
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

LETHAL = 90
NEAR, FAR = 0.30, 1.20
HALF_W = 0.35


def main():
    rclpy.init()
    node = rclpy.create_node("global_front_check")
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    got = []
    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    node.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                             lambda m: got.append(m), qos)
    t0 = time.time()
    while (not got or not tfbuf.can_transform("map", "base_footprint", rclpy.time.Time())) \
            and time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not got:
        raise SystemExit("no llega /global_costmap/costmap")
    try:
        tf = tfbuf.lookup_transform("map", "base_footprint", rclpy.time.Time())
    except Exception as e:
        raise SystemExit(f"sin TF map->base_footprint: {e}")

    rx = tf.transform.translation.x; ry = tf.transform.translation.y
    q = tf.transform.rotation
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    m = got[-1]
    W, H = m.info.width, m.info.height
    res = m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    grid = np.array(m.data, dtype=np.int16).reshape(H, W)

    # sample points in a lane ahead of the robot, transform to map, read the cell
    hits = []
    for d in np.arange(NEAR, FAR, res):
        for s in np.arange(-HALF_W, HALF_W, res):
            mx = rx + d*math.cos(yaw) - s*math.sin(yaw)
            my = ry + d*math.sin(yaw) + s*math.cos(yaw)
            col = int((mx - ox)/res); row = int((my - oy)/res)
            if 0 <= col < W and 0 <= row < H and grid[row, col] >= LETHAL:
                hits.append(d)
    print(f"robot en map ({rx:.2f},{ry:.2f}) yaw {math.degrees(yaw):.0f} deg")
    if hits:
        print(f"celdas de obstaculo en la franja {NEAR}-{FAR} m al frente: {len(hits)}")
        print(f"  mas cercano: {min(hits):.2f} m")
        print("  -> LA SILLA ESTA EN EL COSTMAP GLOBAL -> el planner puede rodearla.")
    else:
        print("  -> no hay obstaculo al frente en el GLOBAL (¿la camara no llega, o no marca?).")
    node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
