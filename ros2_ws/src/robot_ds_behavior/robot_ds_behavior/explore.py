#!/usr/bin/env python3
# explore.py - autonomous FRONTIER exploration so the whole floor gets mapped
# and every POI becomes reachable.
#
# How it works: SLAM publishes /map (an occupancy grid: -1 unknown, 0 free,
# 100 occupied). A "frontier" is a free cell touching an unknown cell -- i.e.
# the edge of what we know. We find the frontiers, cluster them, send the robot
# to the nearest cluster with Nav2, and repeat. When no frontiers remain, the
# reachable floor is fully mapped.
#
# This is the off-the-shelf explore_lite idea, written small (explore_lite has
# no Jazzy package and we can't apt-install on rosita). Reuses the same Nav2
# NavigateToPose action as guide.py.
#
# Usage (with Nav2 + SLAM running):
#     python3 explore.py
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from action_msgs.msg import GoalStatus
import tf2_ros

CELL = 0.5        # clustering bucket size (m)
MIN_PTS = 3       # ignore frontier clusters smaller than this (noise)
MIN_DIST = 0.5    # ignore frontiers closer than this (else goal = "already there", no driving)
SKIP_NEAR = 0.6   # don't revisit a target within this of one we already tried
MAX_ITERS = 80    # safety cap on number of goals
MAX_FAILS = 12    # stop after this many consecutive aborted goals (subido: en exploracion abortan a menudo)


class Explorer(Node):
    def __init__(self):
        super().__init__("explorer")
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.spin_ac = ActionClient(self, Spin, "spin")
        self.map = None
        self.create_subscription(OccupancyGrid, "map", self._map_cb, 1)
        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)
        self.visited = []   # targets we already tried, to avoid loops

    def _map_cb(self, msg):
        self.map = msg

    def robot_xy(self):
        try:
            t = self.tfbuf.lookup_transform("map", "base_footprint", rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return (0.0, 0.0)

    def frontier_points(self):
        # return world (x,y) of every FREE cell (0) that touches an UNKNOWN cell (-1).
        # Vectorizado con numpy (antes era un doble bucle Python sobre TODAS las
        # celdas -> se comia ~40% de CPU y ahogaba la Jetson durante el mapeo).
        m = self.map
        w, h, res = m.info.width, m.info.height, m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        grid = np.asarray(m.data, dtype=np.int16).reshape(h, w)
        free = grid == 0
        unknown = grid == -1
        nb = np.zeros_like(unknown)          # celda con algun vecino (4-conn) desconocido
        nb[1:, :] |= unknown[:-1, :]
        nb[:-1, :] |= unknown[1:, :]
        nb[:, 1:] |= unknown[:, :-1]
        nb[:, :-1] |= unknown[:, 1:]
        ys, xs = np.nonzero(free & nb)
        return [(ox + (x + 0.5) * res, oy + (y + 0.5) * res) for x, y in zip(xs.tolist(), ys.tolist())]

    def pick_target(self, pts, rxy):
        # bucket frontier points into CELL-sized cells, keep dense ones, return
        # the centroid of the cluster nearest to the robot
        buckets = {}
        for (px, py) in pts:
            k = (round(px / CELL), round(py / CELL))
            buckets.setdefault(k, []).append((px, py))
        cands = []
        for group in buckets.values():
            if len(group) < MIN_PTS:
                continue
            cx = sum(p[0] for p in group) / len(group)
            cy = sum(p[1] for p in group) / len(group)
            dist = math.hypot(cx - rxy[0], cy - rxy[1])
            if dist < MIN_DIST:                          # too close -> no real driving
                continue
            if any(math.hypot(cx - vx, cy - vy) < SKIP_NEAR for (vx, vy) in self.visited):
                continue                                 # already tried near here
            cands.append((cx, cy, dist))
        if not cands:
            return None
        cands.sort(key=lambda c: c[2])                   # nearest one beyond MIN_DIST
        return (cands[0][0], cands[0][1])

    def go(self, x, y):
        self.nav.wait_for_server()
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = float(x)
        g.pose.pose.position.y = float(y)
        g.pose.pose.orientation.w = 1.0
        fut = self.nav.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            return False
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut)
        return rfut.result().status == GoalStatus.STATUS_SUCCEEDED

    def spin_in_place(self, yaw):
        # gira en el sitio con la accion /spin de Nav2 para SEMBRAR el mapa
        # (asi slam_toolbox ve alrededor antes de que haya fronteras que perseguir)
        if not self.spin_ac.wait_for_server(timeout_sec=15.0):
            print("[explore] accion /spin no disponible, salto el giro inicial")
            return
        g = Spin.Goal()
        g.target_yaw = float(yaw)
        fut = self.spin_ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is not None and gh.accepted:
            rfut = gh.get_result_async()
            rclpy.spin_until_future_complete(self, rfut)

    def explore(self):
        print("[explore] waiting for the map...")
        while self.map is None:
            rclpy.spin_once(self, timeout_sec=0.5)
        # giro inicial ~360 grados para construir un mapa de partida con fronteras
        print("[explore] giro inicial 360 para sembrar el mapa...")
        self.spin_in_place(3.14)
        self.spin_in_place(3.14)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        fails = 0
        for i in range(MAX_ITERS):
            # refresh map + tf for a moment
            for _ in range(4):
                rclpy.spin_once(self, timeout_sec=0.1)
            rxy = self.robot_xy()
            pts = self.frontier_points()
            target = self.pick_target(pts, rxy)
            if target is None:
                print("[explore] no frontiers left -> floor fully mapped!")
                break
            print(f"[explore] {i + 1}: go to ({target[0]:.2f}, {target[1]:.2f})  frontier_cells={len(pts)}")
            self.visited.append(target)   # remember so we don't loop on it
            if self.go(*target):
                fails = 0
            else:
                fails += 1
                print(f"[explore]   goal aborted ({fails}/{MAX_FAILS})")
                if fails >= MAX_FAILS:
                    print("[explore] too many failures in a row, stopping.")
                    break
        print("[explore] done.")


def main():
    rclpy.init()
    node = Explorer()
    node.explore()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
