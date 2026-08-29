#!/usr/bin/env python3
# near_range_check.py - read-only. In the scan pose, how CLOSE and how FAR does the camera
# actually see the floor ahead? The nearest distance is the blind-spot edge: closer than
# that the robot is blind, and if the lidar clears the mark there, the robot forgets the
# obstacle right before hitting it.
import os
import numpy as np
import yaml
import rclpy
import time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

CALIB = os.path.expanduser("~/ros2_ws/camera_calib.yaml")
FRONT = np.radians(30.0)


def main():
    c = yaml.safe_load(open(CALIB))
    R = np.array(c["R"], dtype=np.float32); t = np.array(c["t"], dtype=np.float32)
    rclpy.init()
    n = rclpy.create_node("near_range_check")
    got = []
    n.create_subscription(PointCloud2, "/depth_cam/depth/points", lambda m: got.append(m), qos_profile_sensor_data)
    t0 = time.time()
    while not got and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not got:
        raise SystemExit("sin nube")
    pts = point_cloud2.read_points_numpy(got[-1], field_names=("x", "y", "z"), skip_nans=True)[::3]
    P = pts.astype(np.float32) @ R.T + t
    x, y, z = P[:, 0], P[:, 1], P[:, 2]
    rng = np.hypot(x, y); ang = np.arctan2(y, x)
    floor = (np.abs(ang) < FRONT) & (np.abs(z) < 0.05)   # near-flat floor points ahead
    r = rng[floor]
    print(f"calib usada: {c['tilt_deg']:.1f} deg, altura {c['height']:.3f} m")
    if len(r):
        print(f"la camara VE el suelo desde {r.min():.2f} m hasta {r.max():.2f} m (frente +-30 deg)")
        print(f"  robot: radio 0.16 m + inflado 0.20 m = 0.36 m de la base")
        gap = r.min() - 0.16
        print(f"  PUNTO CIEGO: nada visible entre el borde del robot (0.16 m) y {r.min():.2f} m"
              f"  -> franja ciega de {gap:.2f} m delante")
    else:
        print("no ve suelo en el cono frontal")
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
