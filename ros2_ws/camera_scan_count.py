#!/usr/bin/env python3
# camera_scan_count.py - read-only check of /camera_scan: how many bearings report an
# obstacle, and how near. Empty floor should give ~0 near hits.
import sys
import numpy as np
import rclpy
import time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

N = int(sys.argv[1]) if len(sys.argv) > 1 else 10


def main():
    rclpy.init()
    node = rclpy.create_node("camera_scan_count")
    got = []
    node.create_subscription(LaserScan, "/camera_scan", lambda m: got.append(m), qos_profile_sensor_data)
    t0 = time.time()
    while len(got) < N and time.time() - t0 < 25:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not got:
        raise SystemExit("nada en /camera_scan -- el nodo no publica")
    counts, allr = [], []
    for m in got[-N:]:
        r = np.array(m.ranges, dtype=np.float32)
        good = np.isfinite(r)
        counts.append(int(good.sum()))
        allr.append(r[good])
    tot = len(got[-1].ranges)
    print(f"scans leidos: {len(counts)}   rayos por scan: {tot}")
    print(f"rayos con obstaculo: min {min(counts)}  mediana {int(np.median(counts))}  max {max(counts)}")
    a = np.concatenate(allr) if any(counts) else np.empty(0)
    if len(a):
        print(f"distancias: min {a.min():.2f} m  mediana {np.median(a):.2f} m  max {a.max():.2f} m")
    else:
        print("distancias: (ninguna) -- vista limpia")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
