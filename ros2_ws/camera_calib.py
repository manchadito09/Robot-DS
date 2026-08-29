#!/usr/bin/env python3
# camera_calib.py - work out where the depth camera REALLY is, by looking at bare floor.
#
# WHY: the camera rides on the arm, so its TF comes from the URDF + the servo angles --
# and that chain is wrong (measured: ~20 deg of tilt and 7 cm of height off). With a bad
# TF the floor gets projected 10-35 cm into the air, lands inside any obstacle height
# band, and the costmap sees a wall of phantom obstacles. That is what flooded Nav2 with
# "purple" before.
#
# So we ignore the arm TF and measure the pose directly: point the robot at 2-3 m of
# EMPTY FLOOR, run this, and it fits the floor plane and saves the true camera pose.
#
#   python3 ~/ros2_ws/camera_calib.py            # measure + save ~/ros2_ws/camera_calib.yaml
#   python3 ~/ros2_ws/camera_calib.py --check    # measure, print, save nothing
#
# The pose is only valid for ONE arm pose, so the arm must always be put in the same
# place first:  python3 ~/ros2_ws/arm_gesture.py camera_forward
import os
import sys
import math
import numpy as np
import yaml
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import time

CLOUD = "/depth_cam/depth/points"
OUT = os.path.expanduser("~/ros2_ws/camera_calib.yaml")
# The camera's sideways/forward offset from the robot centre is small and mechanical;
# only height and tilt are wrong, so we keep these from the URDF.
CAM_X, CAM_Y = 0.095, -0.012
# Fit the plane on NEAR points only. In the scan pose the camera also sees walls and
# furniture ahead, so the floor is never most of the view -- but the first metre in
# front of the robot is floor and nothing else.
NEAR_M = 1.2


def fit_plane(P, iters=120, thresh=0.02):
    """RANSAC plane fit -> (unit normal, offset d) with n.p + d = 0."""
    rng = np.random.default_rng(0)
    best = (None, None, 0)
    for _ in range(iters):
        s = P[rng.choice(len(P), 3, replace=False)]
        v = np.cross(s[1] - s[0], s[2] - s[0])
        ln = np.linalg.norm(v)
        if ln < 1e-6:
            continue
        v /= ln
        d = -v @ s[0]
        inl = int((np.abs(P @ v + d) < thresh).sum())
        if inl > best[2]:
            best = (v, d, inl)
    return best


def main():
    check = "--check" in sys.argv
    rclpy.init()
    n = rclpy.create_node("camera_calib")
    msgs = []
    n.create_subscription(PointCloud2, CLOUD, lambda m: msgs.append(m), qos_profile_sensor_data)
    t0 = time.time()
    while not msgs and time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not msgs:
        sys.exit("No point cloud on %s -- is the camera running? (bash ~/ros2_ws/camera_start.sh)" % CLOUD)

    m = msgs[-1]
    P = point_cloud2.read_points_numpy(m, field_names=("x", "y", "z"), skip_nans=True)[::5]
    near = P[P[:, 2] < NEAR_M]                             # optical z is distance ahead
    if len(near) < 2000:
        sys.exit("Only %d points within %.1f m -- is the robot facing open floor?" % (len(near), NEAR_M))
    normal, d, inliers = fit_plane(near)
    frac = inliers / len(near)
    print(f"  cloud: {len(P)} pts, {len(near)} within {NEAR_M} m, plane inliers {frac*100:.0f}%")
    if frac < 0.90:
        sys.exit("Only %.0f%% of the near view is one plane -- point the robot at EMPTY FLOOR." % (frac * 100))

    # floor normal must point 'up'; in an optical frame up is -y
    up_cam = normal if (normal @ np.array([0.0, -1.0, 0.0])) > 0 else -normal
    height = abs(d)

    # robot forward = the viewing direction flattened onto the floor plane
    view = np.array([0.0, 0.0, 1.0])                      # optical +z
    fwd_cam = view - (view @ up_cam) * up_cam
    fwd_cam /= np.linalg.norm(fwd_cam)
    left_cam = np.cross(up_cam, fwd_cam)                  # base y = z x x

    # rows are the base axes expressed in camera coords -> p_base = R @ p_cam + t
    R = np.vstack([fwd_cam, left_cam, up_cam])
    t = np.array([CAM_X, CAM_Y, height])

    tilt = math.degrees(math.asin(np.clip(-(R @ view)[2], -1, 1)))
    print(f"  MEASURED: height {height:.3f} m, looking {tilt:.1f} deg down")

    # sanity: the near floor must now land on z ~ 0 (the far view has walls in it)
    Z = (near @ R.T + t)[:, 2]
    print(f"  check: floor z after correction -> median {np.median(Z):+.3f} m, "
          f"p1 {np.percentile(Z,1):+.3f}, p99 {np.percentile(Z,99):+.3f}")

    if check:
        print("  (--check: nothing saved)")
    else:
        with open(OUT, "w") as f:
            yaml.safe_dump({"height": float(height), "tilt_deg": float(tilt),
                            "R": [[float(v) for v in row] for row in R],
                            "t": [float(v) for v in t],
                            "cloud_frame": m.header.frame_id}, f, sort_keys=False)
        print("  saved ->", OUT)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
