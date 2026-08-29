#!/usr/bin/env python3
# localization_watch.py - read-only. Why does the robot teleport on the map?
#
# It happens when the robot thrashes in a tight spot (forward/back, turning in place): the tracks
# slip, and either the odometry lies or AMCL jumps. Those are two different bugs with two different
# fixes, and you cannot tell them apart from memory. So: run this while you drive, and when the
# robot teleports, the trace says which one it was.
#
# The key number is the DISCREPANCY: how far AMCL says the robot moved between two of its updates,
# minus how far the odometry says it moved over the same window.
#
#   AMCL moved a lot, odom moved a little  -> AMCL TELEPORTED (it is the one lying)
#   AMCL and odom agree, both spinning     -> the odometry is lying and AMCL is following it
#
#   python3 ~/ros2_ws/localization_watch.py                 # print + log to /tmp/loc_watch.log
#   python3 ~/ros2_ws/localization_watch.py --quiet         # only the jumps, no heartbeat
#
# Ctrl-C to stop; it prints a summary of every jump it caught.
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

LOG = "/tmp/loc_watch.log"

# A jump is AMCL disagreeing with odometry by more than this over one AMCL update.
JUMP_M = 0.30            # metres of unexplained translation
JUMP_DEG = 20.0          # degrees of unexplained rotation


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angdiff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class Watch(Node):
    def __init__(self, quiet):
        super().__init__("localization_watch")
        self.quiet = quiet
        self.odom = None          # (x, y, yaw) latest
        self.imu_yaw = None
        self.last_amcl = None     # (x, y, yaw) at the previous AMCL update
        self.last_odom = None     # (x, y, yaw) at the previous AMCL update
        self.jumps = []
        self.t0 = time.time()
        self.last_beat = 0.0

        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_subscription(Imu, "/imu", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, 10)

        self.log("t(s)  | AMCL x     y     yaw   | odom yaw | imu yaw | cov(x,y,yaw)      | nota")
        self.log("-" * 100)

    def log(self, line):
        print(line, flush=True)
        with open(LOG, "a") as f:
            f.write(line + "\n")

    def on_odom(self, m):
        p = m.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_imu(self, m):
        self.imu_yaw = yaw_of(m.orientation)

    def on_amcl(self, m):
        if self.odom is None:
            return
        p = m.pose.pose
        amcl = (p.position.x, p.position.y, yaw_of(p.orientation))
        c = m.pose.covariance
        cov = (c[0], c[7], c[35])          # x, y, yaw variances
        t = time.time() - self.t0

        note = ""
        if self.last_amcl is not None:
            # how far each source says we moved since the previous AMCL update
            d_amcl = math.hypot(amcl[0] - self.last_amcl[0], amcl[1] - self.last_amcl[1])
            d_odom = math.hypot(self.odom[0] - self.last_odom[0], self.odom[1] - self.last_odom[1])
            a_amcl = abs(angdiff(amcl[2], self.last_amcl[2]))
            a_odom = abs(angdiff(self.odom[2], self.last_odom[2]))

            gap_m = d_amcl - d_odom            # translation AMCL cannot explain with odometry
            gap_a = math.degrees(a_amcl - a_odom)

            if gap_m > JUMP_M or gap_a > JUMP_DEG:
                note = (f"*** SALTO: AMCL se movio {d_amcl:.2f} m / {math.degrees(a_amcl):.0f} deg "
                        f"pero la odometria solo {d_odom:.2f} m / {math.degrees(a_odom):.0f} deg")
                self.jumps.append((t, gap_m, gap_a, d_odom, math.degrees(a_odom)))

        self.last_amcl = amcl
        self.last_odom = self.odom

        if note or not self.quiet:
            oy = math.degrees(self.odom[2])
            iy = math.degrees(self.imu_yaw) if self.imu_yaw is not None else float("nan")
            self.log(f"{t:6.1f} | {amcl[0]:6.2f} {amcl[1]:6.2f} {math.degrees(amcl[2]):6.1f} "
                     f"| {oy:8.1f} | {iy:7.1f} | {cov[0]:.3f},{cov[1]:.3f},{cov[2]:.3f} | {note}")

    def summary(self):
        self.log("")
        self.log("=" * 60)
        if not self.jumps:
            self.log("NO hubo saltos. AMCL siguio a la odometria todo el rato.")
            self.log("Si aun asi el robot acabo en el sitio equivocado, entonces MIENTE LA")
            self.log("ODOMETRIA: se fue poco a poco y AMCL la siguio. Mira el EKF/las orugas.")
            return
        self.log(f"SALTOS DE AMCL: {len(self.jumps)}")
        for t, gm, ga, d_odom, a_odom in self.jumps:
            self.log(f"  t={t:6.1f}s  sin explicar: {gm:+.2f} m / {ga:+.0f} deg   "
                     f"(la odometria decia {d_odom:.2f} m / {a_odom:.0f} deg)")
        self.log("")
        self.log("AMCL se teletransporto. NO es culpa de la odometria: es AMCL, que se cree")
        self.log("otra posicion del mapa (paredes parecidas / nube de particulas mal).")
        self.log(f"Traza completa en {LOG}")


def main():
    quiet = "--quiet" in sys.argv
    rclpy.init()
    node = Watch(quiet)
    print(f"Mirando /amcl_pose, /odom, /imu ... conduce el robot. Ctrl-C para el resumen.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.summary()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
