#!/usr/bin/env python3
# costmap_front_check.py - read-only. Is there an obstacle in the LOCAL costmap just in
# front of the robot? The local costmap is a rolling window centred on base_footprint, so
# we look at the cells 0.3-1.2 m ahead along +x and report the nearest lethal cell. With
# the lidar blind to the low chair, an obstacle here means the camera put it there.
import numpy as np
import rclpy
import time
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

LETHAL = 90            # cost >= this counts as an obstacle
NEAR, FAR = 0.30, 1.20
HALF_W = 0.30          # +-30 cm sideways (a ~60 cm lane ahead)


def main():
    rclpy.init()
    node = rclpy.create_node("costmap_front_check")
    got = []
    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL      # costmaps are latched
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    node.create_subscription(OccupancyGrid, "/local_costmap/costmap",
                             lambda m: got.append(m), qos)
    t0 = time.time()
    while not got and time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not got:
        raise SystemExit("no llega /local_costmap/costmap")

    m = got[-1]
    W, H = m.info.width, m.info.height
    res = m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    grid = np.array(m.data, dtype=np.int16).reshape(H, W)

    # robot is at (0,0) in the costmap frame (rolling window, base_footprint centred).
    # cell (col,row) -> world: x = ox + (col+0.5)*res, y = oy + (row+0.5)*res
    cols = ox + (np.arange(W) + 0.5) * res
    rows = oy + (np.arange(H) + 0.5) * res
    X, Y = np.meshgrid(cols, rows)                 # X[row,col], Y[row,col]

    lane = (X > NEAR) & (X < FAR) & (np.abs(Y) < HALF_W)
    obst = lane & (grid >= LETHAL)
    n = int(obst.sum())
    print(f"costmap {W}x{H} @ {res:.2f} m, franja {NEAR}-{FAR} m al frente (+-{HALF_W} m)")
    print(f"celdas de obstaculo en esa franja: {n}")
    if n:
        d = np.hypot(X[obst], Y[obst])
        print(f"  obstaculo mas cercano al frente: {d.min():.2f} m")
        print("  -> HAY un obstaculo bajo delante en el costmap (el lidar no lo ve -> es la camara).")
    else:
        print("  -> no hay obstaculo en esa franja (¿silla fuera de sitio o costmap sin la fuente?).")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
