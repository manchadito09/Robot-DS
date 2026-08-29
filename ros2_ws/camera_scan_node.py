#!/usr/bin/env python3
# camera_scan_node.py - a virtual 2D laser from the depth camera, for the LOW obstacles
# the lidar cannot see (chair feet, whiteboard legs). The lidar sweeps at 16.8 cm, so
# anything below that is invisible to it.
#
# SELF-CALIBRATING (2026-07-11): the camera rides on the ARM, whose angle is NOT stable --
# the servo doesn't return to the exact same angle, and the arm trembles as the robot
# drives. A fixed calibration (a saved angle) therefore drifts and the floor gets
# mis-projected. So instead of trusting a saved pose, EVERY frame we look at the near
# floor and work out the camera's real angle right now, then project with THAT. Trembles
# and servo drift stop mattering because we never rely on an old number.
#
# It still does NOT use the camera's TF (URDF + servo angles), which is wrong. It fits the
# real floor plane each frame and publishes the scan already in base_footprint, so the
# costmap never touches the arm TF.
#
#   python3 ~/ros2_ws/arm_gesture.py scan     # tilt the arm down (any nearby angle is fine)
#   python3 ~/ros2_ws/camera_scan_node.py     # -> /camera_scan (auto-calibrated live)
#
# The saved camera_calib.yaml is only a FALLBACK for the first frames / when too little
# floor is visible to fit a plane (e.g. an obstacle fills the view).
import os

# ONE BLAS THREAD, AND IT HAS TO BE SET BEFORE numpy IS IMPORTED. Do not move these lines.
#
# This node has eaten this Jetson three times. Not by working hard -- by SPINNING. numpy hands its
# linear algebra to OpenBLAS, which by default starts one thread per core (six, here) and BUSY-WAITS
# them between operations. The matrices in this file are 3x3. There is nothing to parallelise, so
# the threads did no work at all; they just burned five cores waiting, load average hit 15, and
# every other thing on the robot starved behind them -- Whisper 8 s, Piper 5 s, Nav2 lurching. It
# looked like four separate bugs. It was six idle threads.
#
#   374% of a core -> 33%. And the work got FASTER (the floor fit: 18 ms -> 8 ms), because the six
#   threads had been fighting each other over 3x3 matrices.
#
# camera_scan.sh exports these too, and that is deliberate belt-and-braces: this is the ONE setting
# that must survive somebody launching the node a different way. Read once by OpenBLAS, at import.
# Set them after `import numpy` and they do nothing, silently.
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

import math                                                              # noqa: E402
import time                                                              # noqa: E402
import numpy as np                                                       # noqa: E402
import yaml                                                              # noqa: E402
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2

CALIB = os.path.expanduser("~/ros2_ws/camera_calib.yaml")   # fallback only

MIN_H = 0.04          # m above the floor: below this is floor (noise is +-1.5 cm)

# ...but only out to LIFT_FROM. Past that the obstacle line rises with distance, because the ERROR
# rises with distance: the floor plane is fitted on the near floor and extrapolated outwards, so the
# fit's angular error is multiplied by range. See the long comment in on_cloud -- this is what was
# painting phantom walls 2 m ahead and caging the robot in them.
#
# LIFT_SLOPE is tan(3 deg), which is the fit error the RANSAC actually tolerates (FIT_THRESH = 2 cm
# over a patch about 1 m across). Measured: it takes the ghosts on empty floor from 24 a scan to
# ~13, the number a healthy camera gives, without touching the near field at all.
LIFT_FROM = 1.0       # m: inside this, nothing changes -- 4 cm, as before
LIFT_SLOPE = 0.05     # m of extra height per m of extra distance (~3 degrees)

# RATE: it is limited ONCE, at the camera (depth_fps: 5 in dabai_dcw.launch.py), and NOT again here.
#
# This node used to eat 334% CPU -- 3.3 of the Jetson's 6 cores -- and everything else on the robot
# starved behind it: Whisper took 8 s to transcribe 3 s of speech, Piper 5 s for one sentence, load
# average 15. Nothing was slow; nothing could get a core.
#
# Throttling THIS CALLBACK barely helped (334% -> 293%): by the time our code runs, rclpy has already
# deserialised the 230k-point cloud. Dropping the frame here is throwing the food out after cooking
# it. The only way to not pay for a frame is to not be sent it -- hence depth_fps at the source.
#
# And then throttling BOTH was worse than either: with the camera at 5 Hz and this filter also at
# 5 Hz, ordinary jitter threw away every other frame, /camera_scan fell to ~2.5 Hz, and it went quiet
# often enough for the web app's camera watchdog to declare the camera dead and restart it. On a
# loop. Two throttles in series, each sensible alone, fighting each other.
MIN_PERIOD = 0.0          # the camera already limits the rate; do not limit it twice

# The RANSAC floor fit, though, is worth throttling separately: 60 numpy iterations per frame, and it
# is the single most expensive thing here. THE FLOOR DOES NOT MOVE. Fit it twice a second and reuse
# the pose in between -- the arm trembles, but slowly, and the fit converges on the same plane anyway.
FIT_PERIOD = 0.5          # s between floor re-fits (the expensive bit)
MAX_H = 0.60          # m: taller things the lidar already sees
RANGE_MIN = 0.20      # m from the robot centre
RANGE_MAX = 2.50
ANGLE_LIM = math.radians(45.0)
ANGLE_INC = math.radians(0.5)
STRIDE = 4            # keep 1 point in 4 -- plenty, and 4x cheaper

# CAMERA_SCAN_PROFILE=1 -> every 5 s, print where this node's milliseconds actually go. It has
# eaten this Jetson twice, and both times the first guess about which part was expensive was wrong.
PROFILE = os.environ.get("CAMERA_SCAN_PROFILE", "1") == "1"   # CAMERA_SCAN_PROFILE=0 to silence

# Live floor-fit settings (mirror camera_calib.py, kept cheap enough for ~10 Hz)
CAM_X, CAM_Y = 0.095, -0.012   # sideways/forward offset (small, mechanical, from URDF)
NEAR_M = 1.2                   # fit the plane on floor within this many metres ahead
FIT_ITERS = 60                 # RANSAC iterations (fewer than the offline tool -> faster)
FIT_THRESH = 0.02              # inlier band (m)
MIN_FLOOR_FRAC = 0.80          # need this fraction of the near view to be one plane
MIN_NEAR_PTS = 1500            # ...and at least this many near points


def fit_plane(P, rng, iters=FIT_ITERS, thresh=FIT_THRESH):
    """RANSAC plane fit -> (unit normal, offset d, inlier count)."""
    best = (None, None, 0)
    n = len(P)
    for _ in range(iters):
        s = P[rng.integers(0, n, 3)]
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


def pose_from_floor(P):
    """Fit the near floor of one cloud and return (R, t), or None if the view isn't
    mostly floor (too few points, or an obstacle fills it)."""
    near = P[P[:, 2] < NEAR_M]                     # optical z is distance ahead
    if len(near) < MIN_NEAR_PTS:
        return None
    rng = np.random.default_rng(0)                 # fixed seed -> deterministic, resume-safe
    normal, d, inliers = fit_plane(near, rng)
    if normal is None or inliers / len(near) < MIN_FLOOR_FRAC:
        return None
    up = normal if (normal @ np.array([0.0, -1.0, 0.0])) > 0 else -normal   # up = -y in optical
    view = np.array([0.0, 0.0, 1.0])               # optical +z (forward)
    fwd = view - (view @ up) * up
    nf = np.linalg.norm(fwd)
    if nf < 1e-6:
        return None
    fwd /= nf
    left = np.cross(up, fwd)
    R = np.vstack([fwd, left, up]).astype(np.float32)
    t = np.array([CAM_X, CAM_Y, abs(d)], dtype=np.float32)
    return R, t


class CameraScan(Node):
    def __init__(self):
        super().__init__("camera_scan")
        # Fallback pose from the saved file (used only until the first good live fit, or
        # when too little floor is visible to fit).
        try:
            with open(CALIB) as f:
                c = yaml.safe_load(f)
            self.R = np.array(c["R"], dtype=np.float32)
            self.t = np.array(c["t"], dtype=np.float32)
            self.get_logger().info(
                "self-calibrating; fallback %.1f deg, %.3f m" % (c.get("tilt_deg", 0), c.get("height", 0)))
        except Exception:
            # no file: start flat-ish; the first good frame will fix it
            self.R = np.eye(3, dtype=np.float32)
            self.t = np.array([CAM_X, CAM_Y, 0.35], dtype=np.float32)
            self.get_logger().warn("no camera_calib.yaml; self-calibrating from scratch")
        self.live = False
        self.n_bins = int(2 * ANGLE_LIM / ANGLE_INC) + 1
        self.pub = self.create_publisher(LaserScan, "/camera_scan", qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/depth_cam/depth/points",
                                 self.on_cloud, qos_profile_sensor_data)
        self.count = 0
        self._last_tick = 0.0
        self._last_fit = 0.0
        self._xyz_off = None      # byte offset of x within a point, once we've worked it out
        self._checks_left = 5     # prove the fast reader against the slow one, on real clouds
        self._prof = {"t0": time.monotonic(), "n": 0, "read": 0.0, "fit": 0.0, "rest": 0.0}

    def _xyz(self, msg):
        """The x/y/z of every STRIDE-th point, as a plain (N, 3) float32 array.

        The replacement for point_cloud2.read_points_numpy(), which was costing 0.35 core-seconds
        per cloud -- 175% CPU for five clouds a second -- because it assembles a structured array
        field by field. The message is already a flat buffer of fixed-size points, and x, y, z are
        three consecutive float32s inside each one. So: reshape to (points, point_step), take every
        STRIDE-th row, slice out the 12 bytes, and view them as floats. No Python per point.

        Returns None if the cloud is not laid out the way we expect, and the caller falls back."""
        if self._xyz_off is None:
            off = {f.name: (f.offset, f.datatype) for f in msg.fields}
            try:
                ox, tx = off["x"]
                oy, _ = off["y"]
                oz, _ = off["z"]
            except KeyError:
                return None
            # 7 = FLOAT32, and x/y/z must be consecutive for the 12-byte slice to work.
            if tx != 7 or oy != ox + 4 or oz != ox + 8:
                self.get_logger().warn(
                    "unexpected point layout -- falling back to the slow reader")
                self._xyz_off = -1
            else:
                self._xyz_off = ox
        if self._xyz_off == -1:                       # odd layout: the old, slow, safe path
            return point_cloud2.read_points_numpy(
                msg, field_names=("x", "y", "z"), skip_nans=True)[::STRIDE].astype(np.float32)

        o = self._xyz_off
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)[::STRIDE]
        pts = rows[:, o:o + 12].copy().view(np.float32).reshape(-1, 3)
        return pts[np.isfinite(pts).all(axis=1)]      # skip_nans, vectorised

    def _self_check(self, msg, fast):
        """Does the fast reader see the same world as the one it replaces? Real clouds, first few.

        Not "are the points identical" -- they are not, and cannot be (see on_cloud). What must
        agree is what this node exists to produce: where the floor is, and how many things are
        standing on it. So fit both and compare the two answers, and print it, so the next person
        does not have to take anybody's word for it.
        """
        self._checks_left -= 1
        try:
            slow = point_cloud2.read_points_numpy(
                msg, field_names=("x", "y", "z"), skip_nans=True)[::STRIDE].astype(np.float32)
            pf, ps = pose_from_floor(fast), pose_from_floor(slow)
            if pf is None or ps is None:
                self.get_logger().info(
                    f"[self-check] no floor fit ({'fast' if pf is None else 'slow'}) -- "
                    "point the camera at open floor to check this properly")
                return
            # The floor plane each reader sees: compare the direction it faces and how high it is.
            nf, nsl = pf[0][2], ps[0][2]              # the z row of R = the floor normal
            tilt = math.degrees(math.acos(min(1.0, abs(float(np.dot(nf, nsl))))))
            dh = abs(float(pf[1][2] - ps[1][2]))      # camera height above the floor
            hits_f = int(((fast @ pf[0].T + pf[1])[:, 2] > MIN_H).sum())
            hits_s = int(((slow @ ps[0].T + ps[1])[:, 2] > MIN_H).sum())
            ok = tilt < 1.0 and dh < 0.01
            self.get_logger().info(
                f"[self-check] fast reader vs read_points_numpy: {len(fast)} vs {len(slow)} pts | "
                f"floor tilt differs {tilt:.2f} deg, height {dh*100:.1f} cm | "
                f"above-floor points {hits_f} vs {hits_s} | {'AGREE' if ok else 'DISAGREE'}")
            if not ok:
                self.get_logger().error(
                    "[self-check] the fast reader DISAGREES about the floor. Falling back to the "
                    "slow one -- CPU will be high, but the robot will see straight.")
                self._xyz_off = -1                    # every later call takes the slow path
        except Exception as e:
            self.get_logger().warn(f"[self-check] could not run: {e}")

    def on_cloud(self, msg):
        # This node was eating 3.3 of the Jetson's 6 cores, and everything else on the robot was
        # starving behind it: Whisper took 8 s to transcribe 3 s of speech, Piper 5 s for one
        # sentence, the machine sat at load 15. Nothing was slow. Nothing could get a core.
        #
        # Two things were wrong, in this order:
        #
        #  1. The camera published depth at 30 fps, then 15. Throttling THIS CALLBACK barely helped
        #     (334% -> 293%): by the time our code runs, rclpy has already deserialised the cloud.
        #     Dropping the frame here is throwing the food out after cooking it. Fixed at the source
        #     -- depth_fps: 5 in dabai_dcw.launch.py, which is all the local costmap consumes.
        #
        #  2. read_points_numpy() itself. It builds a structured array, field by field, and cost
        #     0.35 core-seconds PER CLOUD -- 175% CPU for five clouds a second. The cloud is already
        #     a flat byte buffer with x, y, z as three consecutive float32s per point. Slice it and
        #     view it. No per-point Python, no structured dtype.
        now = time.monotonic()
        if now - self._last_tick < MIN_PERIOD:
            return
        self._last_tick = now

        # THE FAST READER IS ON. It was written, unit-checked, and then deliberately left switched
        # off two days before the demo, because it had never seen a real cloud. That was the right
        # call then. It is not now: with Nav2 up, this node sat at 230% of a core and the machine at
        # load 15 on six cores -- and the robot has since been given the ability to run Whisper,
        # Claude and Piper WHILE it drives. There is no CPU left to pay 0.35 core-seconds a cloud
        # for a structured array we immediately flatten.
        #
        # So it is on, and it PROVES ITSELF against the reader it replaces -- on real clouds, not
        # synthetic ones (see _self_check). Note the two are NOT expected to return identical
        # points, and demanding that they did would have been a test that only looked rigorous:
        #
        #   slow:  drop NaNs, THEN keep every 4th of the survivors
        #   fast:  keep every 4th pixel, THEN drop NaNs
        #
        # Different sets. Both are a quarter of the valid points, and the fast one is arguably the
        # better sample (every 4th pixel is spatially regular; every 4th survivor is not). What has
        # to agree is what the node actually produces: the same floor, and the same obstacles.
        # That is what _self_check measures, on the first clouds off the camera, and it says so in
        # the log. If the layout is odd, _xyz returns the slow reader's answer anyway.
        # WHERE DOES THE TIME ACTUALLY GO? Set CAMERA_SCAN_PROFILE=1 and it says, every 5 s:
        # clouds a second, and the milliseconds split between reading the buffer, fitting the floor
        # and building the scan. This node has eaten this Jetson twice now, and both times the
        # first guess about WHICH part was expensive was wrong. Measure. Do not guess.
        t_read0 = time.monotonic()
        pts = self._xyz(msg)
        if pts is None or not len(pts):
            return
        pts = pts.astype(np.float32)
        self._prof["read"] += time.monotonic() - t_read0
        self._prof["n"] += 1

        if self._checks_left > 0:
            self._self_check(msg, pts)

        # LIVE re-calibration, but only every FIT_PERIOD. This is the RANSAC, and it is the whole
        # cost of this node. The floor is not going anywhere between frames; the arm trembles, but
        # slowly. Fit it twice a second and reuse the pose in between. If the fit fails (an obstacle
        # fills the view), keep the last good one -- exactly as before.
        t_fit0 = time.monotonic()
        if now - self._last_fit >= FIT_PERIOD:
            self._last_fit = now
            pose = pose_from_floor(pts)
            if pose is not None:
                self.R, self.t = pose
                if not self.live:
                    self.live = True
                    self.get_logger().info("live floor fit active")
        self._prof["fit"] += time.monotonic() - t_fit0
        t_rest0 = time.monotonic()

        P = pts @ self.R.T + self.t                    # -> base_footprint

        # THE OBSTACLE LINE RISES WITH DISTANCE, and this is the fix for the cage.
        #
        # Measured on empty floor (floor_probe.py), the ghosts were not scattered -- they were ALL
        # far away, and the near field was spotless:
        #
        #     0.20 - 0.75 m      0 ghosts
        #     0.75 - 1.25 m      0
        #     1.25 - 1.75 m      4
        #     1.75 - 2.25 m     20
        #
        # That shape is not a broken camera. It is geometry. The floor plane is fitted on what the
        # camera sees CLOSE (NEAR_M = 1.2 m) and then used out to 2.5 m, so whatever angular error
        # the fit has gets multiplied by distance. Two degrees is nothing at half a metre and lifts
        # the floor five centimetres at two -- straight over MIN_H, the 4 cm line above which a
        # point is called an obstacle. The far floor became a wall.
        #
        # And a wall 1.4 m ahead is inside the costmap's obstacle_max_range (1.5 m), so it got
        # MARKED. The robot laid a trail of phantom mines in front of itself, drove onto them, and
        # once a mark is within 50 cm the camera layer can never clear it (raytrace_min_range: 0.5).
        # TEB then found no feasible trajectory -- 2809 times in one session -- and could not even
        # spin to recover, because it believed it was walled in. That is the "I couldn't reach X",
        # and it is why nudging the robot with the drive arrows freed it: moving scrolled the marks
        # out of the local costmap's rolling window.
        #
        # So allow the line to rise exactly as fast as the error does. Nothing changes inside 1 m --
        # the near field, where the robot actually has to see a chair leg in time to stop, keeps its
        # 4 cm. Beyond that it relaxes, and it relaxes where the robot has metres of warning anyway:
        # a leg missed at 2 m is seen again at 1 m, at full sensitivity, three seconds before it
        # matters.
        #
        #     0.5 m -> 4.0 cm      1.5 m -> 6.5 cm
        #     1.0 m -> 4.0 cm      2.0 m -> 9.0 cm
        rng_xy = np.hypot(P[:, 0], P[:, 1])
        h_min = MIN_H + LIFT_SLOPE * np.maximum(0.0, rng_xy - LIFT_FROM)

        z = P[:, 2]
        keep = (z > h_min) & (z < MAX_H)               # drop the floor and the tall stuff
        P, rng_xy = P[keep], rng_xy[keep]
        if len(P):
            rng = rng_xy
            ang = np.arctan2(P[:, 1], P[:, 0])
            ok = (rng > RANGE_MIN) & (rng < RANGE_MAX) & (np.abs(ang) < ANGLE_LIM)
            rng, ang = rng[ok], ang[ok]
        else:
            rng = ang = np.empty(0, dtype=np.float32)

        ranges = np.full(self.n_bins, np.inf, dtype=np.float32)
        if len(rng):
            idx = ((ang + ANGLE_LIM) / ANGLE_INC).astype(np.int32)
            np.minimum.at(ranges, idx, rng)            # nearest hit wins per bearing

        s = LaserScan()
        s.header.stamp = msg.header.stamp
        s.header.frame_id = "base_footprint"           # already in the robot's frame
        s.angle_min = -ANGLE_LIM
        s.angle_max = ANGLE_LIM
        s.angle_increment = ANGLE_INC
        s.time_increment = 0.0
        s.scan_time = 0.1
        s.range_min = RANGE_MIN
        s.range_max = RANGE_MAX
        s.ranges = ranges.tolist()
        self.pub.publish(s)
        self._prof["rest"] += time.monotonic() - t_rest0

        # The report. Once every 5 s, and only when asked (CAMERA_SCAN_PROFILE=1).
        #
        # It prints BOTH costs, and printing both is the whole point. The wall-clock cost (how long
        # our code takes) and the CPU cost (how much of the machine the process actually burns) are
        # different numbers, and the day they diverge is the day this node is spinning again. Three
        # times it has eaten this Jetson, and every time it was invisible: the work looked cheap,
        # because it WAS cheap, and the machine was on its knees anyway.
        if PROFILE and now - self._prof["t0"] >= 5.0:
            el = now - self._prof["t0"]
            n = max(1, self._prof["n"])
            work = 100 * (self._prof["read"] + self._prof["fit"] + self._prof["rest"]) / el
            cpu, threads = self._cpu_and_threads(el)
            warn = ""
            if cpu is not None and cpu > 2 * max(work, 10.0) + 50:
                warn = ("   <-- BURNING CPU IT IS NOT USING. Threads spinning? "
                        "OPENBLAS_NUM_THREADS should be 1.")
            self.get_logger().info(
                f"[profile] {self._prof['n']/el:.1f} clouds/s, {len(pts)} pts each | "
                f"per cloud: read {1000*self._prof['read']/n:.1f} ms, "
                f"floor-fit {1000*self._prof['fit']/n:.1f} ms, "
                f"scan {1000*self._prof['rest']/n:.1f} ms | "
                f"work {work:.0f}% of a core, CPU {cpu if cpu is None else f'{cpu:.0f}'}%, "
                f"{threads} threads{warn}")
            if warn:
                self.get_logger().error(
                    f"[profile] this process is burning {cpu:.0f}% of the CPU to do {work:.0f}% of "
                    "work. That is threads busy-waiting, and it has brought this robot to its knees "
                    "three times. Check OPENBLAS_NUM_THREADS at the top of camera_scan_node.py.")
            self._prof.update(t0=now, n=0, read=0.0, fit=0.0, rest=0.0)

    def _cpu_and_threads(self, elapsed):
        """This process's REAL CPU use over the last window, and its thread count.

        Not ps's %CPU, which is an average over the whole life of the process and hides a node that
        has only just started misbehaving. Read the counters, take the difference."""
        try:
            with open("/proc/self/stat") as f:
                parts = f.read().rsplit(")", 1)[1].split()
            ticks = int(parts[11]) + int(parts[12])              # utime + stime
            threads = int(parts[17])
            hz = os.sysconf("SC_CLK_TCK")
            last = self._prof.get("ticks")
            self._prof["ticks"] = ticks
            if last is None:
                return None, threads
            return 100.0 * (ticks - last) / hz / elapsed, threads
        except Exception:
            return None, -1


def main():
    rclpy.init()
    node = CameraScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
