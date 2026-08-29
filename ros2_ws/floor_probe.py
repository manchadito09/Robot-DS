#!/usr/bin/env python3
# floor_probe.py - WHERE are the ghosts? Read-only. Stage EMPTY FLOOR, then run.
#
# camera_check.sh --floor says "27 obstacles on empty floor" (a healthy camera gives ~13) and stops
# there. That number alone does not say WHY, and the two causes want opposite fixes:
#
#   ghosts NEAR the robot   -> the floor-plane fit is wrong, or the arm is not in the scan pose.
#                              A bad fit tilts the whole floor and lifts even close points.
#   ghosts FAR from it      -> the fit is nearly right, and this is geometry. The camera looks down
#                              at the floor; a floor plane that is off by ONE DEGREE puts a point
#                              2.5 m away 4.4 cm in the air -- and MIN_H, the height at which
#                              camera_scan_node calls something an obstacle, is 4 cm. The far floor
#                              becomes a wall. Depth noise grows with range too, and it does the
#                              same thing.
#
# So print the hits by distance, and the heights of the points that got called obstacles. The shape
# of that answers it in one look.
#
#   python3 ~/ros2_ws/floor_probe.py        # robot facing EMPTY floor, nothing within 2.5 m
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

N = 12


def main():
    rclpy.init()
    node = rclpy.create_node("floor_probe")
    got = []
    node.create_subscription(LaserScan, "/camera_scan", lambda m: got.append(m),
                             qos_profile_sensor_data)
    t0 = time.time()
    while len(got) < N and time.time() - t0 < 25:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    if len(got) < 3:
        print('no /camera_scan. Is the camera up? bash ~/ros2_ws/camera_check.sh')
        return

    buckets = [(0.2, 0.75), (0.75, 1.25), (1.25, 1.75), (1.75, 2.25), (2.25, 2.6)]
    counts = {b: [] for b in buckets}
    totals = []
    for m in got:
        r = np.array(m.ranges, dtype=np.float32)
        r = r[np.isfinite(r)]
        totals.append(len(r))
        for b in buckets:
            counts[b].append(int(((r >= b[0]) & (r < b[1])).sum()))

    med_total = int(np.median(totals))
    print(f'\n{len(got)} scans.  hits per scan (median): {med_total}   '
          f'(a healthy camera on empty floor: ~13)\n')
    print('  distance ahead        ghosts per scan')
    print('  -------------------   ---------------')
    near = far = 0
    for b in buckets:
        c = int(np.median(counts[b]))
        bar = '#' * min(c, 50)
        print(f'  {b[0]:.2f} - {b[1]:.2f} m         {c:4d}  {bar}')
        if b[1] <= 1.25:
            near += c
        if b[0] >= 1.75:
            far += c

    print()
    if med_total <= 20:
        print('  CLEAN. The camera is not inventing obstacles. If the robot still cages itself,')
        print('  the marks are real ones that never get cleared -- see raytrace_min_range in')
        print('  nav2_params.yaml (camera_layer).')
    elif far > 2 * max(near, 1):
        print('  GHOSTS ARE FAR AWAY. The floor fit is close but not exact, and distance')
        print('  multiplies it: the far floor is being lifted over the 4 cm obstacle line.')
        print('  This is geometry, not a broken camera. The obstacle height has to allow for')
        print('  the error growing with range (or the scan has to stop looking so far).')
    elif near > far:
        print('  GHOSTS ARE CLOSE. That is not geometry -- the floor plane itself is wrong.')
        print('  Check the arm is in the scan pose (python3 ~/ros2_ws/arm_gesture.py scan),')
        print('  then rerun camera_calib.py facing empty floor.')
    else:
        print('  MIXED. Look at the shape above.')


if __name__ == '__main__':
    main()
