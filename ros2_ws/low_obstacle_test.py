#!/usr/bin/env python3
# low_obstacle_test.py - read-only. Chair in front: does the camera catch what the lidar
# misses? Reads /camera_scan (already base_footprint, front=0) and /scan (lidar, mounted
# yawed 180 deg so the robot's front is at +-180 deg). Also measures the HEIGHT of the
# camera points in the near strip, to show they sit below the lidar's 16.8 cm sweep.
import os
import numpy as np
import yaml
import rclpy
import time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2

FRONT = np.radians(30.0)
NEAR_M = 1.8
LIDAR_H = 0.168
CALIB = os.path.expanduser("~/ros2_ws/camera_calib.yaml")


def collect(node, topic, typ, n, to=15):
    got = []
    sub = node.create_subscription(typ, topic, lambda m: got.append(m), qos_profile_sensor_data)
    t0 = time.time()
    while len(got) < n and time.time() - t0 < to:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return got


def cam_front(m):
    r = np.array(m.ranges, dtype=np.float32)
    ang = m.angle_min + np.arange(len(r)) * m.angle_increment
    ok = np.isfinite(r) & (np.abs(ang) < FRONT) & (r < NEAR_M)
    return r[ok]


def lidar_front(m):
    r = np.array(m.ranges, dtype=np.float32)
    ang = np.degrees(m.angle_min + np.arange(len(r)) * m.angle_increment)
    ang = (ang + 360.0) % 360.0 - 180.0
    ang = (ang + 180.0 + 180.0) % 360.0 - 180.0     # undo the 180 deg mount -> front=0
    ok = np.isfinite(r) & (r > m.range_min) & (r < m.range_max) & (np.abs(ang) < 30) & (r < NEAR_M)
    return r[ok]


def main():
    with open(CALIB) as f:
        c = yaml.safe_load(f)
    R = np.array(c["R"], dtype=np.float32); t = np.array(c["t"], dtype=np.float32)

    rclpy.init()
    node = rclpy.create_node("low_obstacle_test")
    cam = collect(node, "/camera_scan", LaserScan, 6)
    lid = collect(node, "/scan", LaserScan, 4)
    cloud = collect(node, "/depth_cam/depth/points", PointCloud2, 2)

    cbest = [cam_front(m).min() for m in cam if len(cam_front(m))]
    lbest = [lidar_front(m).min() for m in lid if len(lidar_front(m))]
    print(f"CAMARA: ve la silla en {len(cbest)}/{len(cam)} barridos"
          + (f" | mas cerca {min(cbest):.2f} m" if cbest else " | no la ve"))
    print(f"LIDAR : ve algo en {len(lbest)}/{len(lid)} barridos"
          + (f" | mas cerca {min(lbest):.2f} m" if lbest else " | no la ve"))

    if cloud:
        pts = point_cloud2.read_points_numpy(cloud[-1], field_names=("x", "y", "z"), skip_nans=True)[::3]
        P = pts.astype(np.float32) @ R.T + t
        x, y, z = P[:, 0], P[:, 1], P[:, 2]
        rng = np.hypot(x, y); ang = np.arctan2(y, x)
        strip = (rng > 0.5) & (rng < 1.2) & (np.abs(ang) < FRONT) & (z > 0.04) & (z < 0.60)
        nz = int(strip.sum())
        print(f"\nEn la franja 0.5-1.2 m al frente: {nz} puntos de obstaculo")
        if nz:
            zz = z[strip]
            below = int((zz < LIDAR_H).sum())
            print(f"  altura: {zz.min():.3f} .. {zz.max():.3f} m (mediana {np.median(zz):.3f})")
            print(f"  POR DEBAJO de los {LIDAR_H} m del lidar: {below}/{nz} ({100*below/nz:.0f}%)")
            print("\n  VEREDICTO:", "LA CAMARA VE LO QUE EL LIDAR NO." if below/nz > 0.4
                  else "casi todo por encima del lidar (el lidar ya lo veia).")
    node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
