#!/bin/bash
# camera_check.sh - one command, one verdict: GREEN or RED (with the reason).
#
# Checks the whole low-obstacle path, in order, and repairs what it can:
#
#   [1/3] depth eye    is /depth_cam/depth/points flowing?
#                      If not: the camera lost the USB race at boot and came up
#                      COLOR ONLY (see camera_start.sh). We do the human's move
#                      automatically: kill the half-dead camera, relaunch it
#                      alone, wait again.
#   [2/3] fake laser   is /camera_scan publishing?
#                      If not: start camera_scan_node via camera_scan.sh.
#   [3/3] truth        OPTIONAL (--floor): with the robot FACING EMPTY FLOOR,
#                      /camera_scan should report ~no obstacles. Many hits on
#                      empty floor = ghost walls = the floor fit is off.
#                      Needs a human to stage the robot, so it never runs by
#                      itself at startup.
#
#   bash ~/ros2_ws/camera_check.sh            # startup check + self-repair
#   bash ~/ros2_ws/camera_check.sh --floor    # + truth check (stage empty floor first)
#
# Exit 0 = GREEN, exit 1 = RED, exit 2 = BLOCKED. Meant to gate startup: don't
# announce "ready" (or start a trip) until this exits 0.
#
# BLOCKED is not a fault, and that distinction is the whole point of exit 2: the
# camera is healthy and something is simply in front of it. Nothing to restart --
# move the robot, or move the thing. Relaunching a driver at a wall is a loop it
# can never win, and we spent a day inside one.
#
# Remember: the fake laser only makes sense with the arm tilted down
# (python3 ~/ros2_ws/arm_gesture.py scan). This script does NOT move the arm.
# NOTE: `set -u` must come AFTER sourcing ROS: setup.bash dereferences unset vars,
# so with -u the script died right here, silently (stderr was going to /dev/null).
# ONE AT A TIME. Two things call this -- the "Robot Web QR" button and the web app's camera watchdog
# -- and they raced: both saw /camera_scan silent, both started a camera_scan_node, and two of them
# ended up publishing to the same topic.
#
# WAIT for the lock, do not fail on it. A bare `flock 9` waits FOREVER in silence -- a stuck run once
# left the lock held and the next person got a hung terminal with no output at all. But -n (give up
# at once) was no better in practice: the watchdog repairs on its own every 60 s and a repair can
# take ~60 s, so a human running this by hand kept bouncing off a lock that was already doing exactly
# what they wanted. Whoever gets here second usually finds the camera fixed and exits GREEN in a
# second -- the probes are the whole point, this script only touches what is actually broken.
#
# -w 45, and it MUST stay under the watchdog's own 90 s timeout in server.py. At -w 100 the watchdog
# killed its own repair, 90 s in, while that repair was politely waiting for the lock.
exec 9>/tmp/camera_check.lock
if ! flock -w 45 9; then
    echo "another camera_check.sh has held the lock for over 45 s -- it is stuck." >&2
    echo "  pkill -f camera_check.sh   then run this again." >&2
    exit 1
fi

source /opt/ros/humble/setup.bash 2>/dev/null
source /home/ubuntu/ros2_ws/install/setup.bash 2>/dev/null
set -u
export ROS_DOMAIN_ID=0

GREEN='\033[0;32m'; RED='\033[0;31m'; NC='\033[0m'
fail() { echo -e "${RED}RED${NC} - $1"; exit 1; }

# probe TOPIC KIND SECONDS -> exit 0 as soon as one message arrives, 1 on timeout.
# (Waits for actual DATA, not just the topic name -- a topic can exist and be silent.)
probe() {
python3 - "$1" "$2" "$3" <<'EOF'
import sys, time, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, LaserScan, Image
topic, kind, secs = sys.argv[1], sys.argv[2], float(sys.argv[3])
MSG = {"cloud": PointCloud2, "scan": LaserScan, "image": Image}[kind]
rclpy.init()
node = rclpy.create_node("camera_check_probe")
got = []
node.create_subscription(MSG, topic, lambda m: got.append(1), qos_profile_sensor_data)
t0 = time.time()
while not got and time.time() - t0 < secs:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if got else 1)
EOF
}

# ---- [1/3] depth eye ---------------------------------------------------------
BLOCKED=0        # set when the camera is healthy but something is inside its 30 cm blind spot
echo "[1/3] depth eye: waiting for /depth_cam/depth/points (up to 10 s)..."
if ! probe /depth_cam/depth/points cloud 10; then
    # IS ANYTHING IN FRONT OF THE LENS? Ask before killing anything.
    #
    # No depth has two causes that look identical from here and want OPPOSITE treatment:
    #
    #   half-dead driver   lost the USB race at boot, came up colour-only  -> relaunch it
    #   COVERED LENS       a wall a hand's width away, someone standing there
    #                      -> every point is inside the 30 cm blind spot, the driver publishes an
    #                         empty cloud (so: no cloud at all), and it is in PERFECT HEALTH
    #
    # We used to relaunch in both cases. Against a wall that is a fight the robot cannot win: kill
    # a healthy driver, start a healthy driver, still no depth, wait 60 s, do it again -- for ever.
    # We watched it do 25 rounds, and each repair killed half-way left orphan drivers behind, until
    # four of them were fighting over one USB device and two camera_scan_nodes were feeding the
    # costmap contradictory scans. The robot, in Rodrigo's words, went mad.
    #
    # Colour is the tell. A dead camera sends nothing. A blocked one still sends pictures.
    if probe /depth_cam/rgb/image_raw image 4; then
        BLOCKED=1
    else
        echo "      no depth AND no colour. Half-dead boot. Relaunching the camera alone..."
        pkill -INT -f 'component_container.*depth_cam'   # never -9 (camera_start.sh)
        sleep 3
        # setsid, not just nohup. nohup only blocks SIGHUP (closing the terminal); Ctrl-C sends
        # SIGINT to the whole FOREGROUND PROCESS GROUP, and a plain `cmd &` stays in it. So a Ctrl-C
        # anywhere in the terminal that ran this script killed the camera with it -- which is
        # exactly how the camera kept "dying on its own". setsid puts it in its own session.
        # 9>&- CLOSES THE LOCK FD IN THE CHILD, and this one line is worth the whole comment.
        # Without it, the camera driver we are about to start INHERITS fd 9 -- the flock above --
        # and holds it for as long as the camera runs. Which is forever. Every camera_check after
        # this one then waited 45 s for a lock held by the very camera it was checking on, gave up,
        # and reported failure. The watchdog saw a failed repair, waited 60 s, tried again, failed
        # again -- twenty-five times -- and the killed half-finished repairs left orphan drivers
        # behind. We ended up with FOUR camera drivers fighting over one USB device and TWO
        # camera_scan_nodes publishing contradictory scans into the costmap. The robot behaved,
        # in Rodrigo's words, like it had gone mad. One inherited file descriptor.
        setsid nohup bash /home/ubuntu/ros2_ws/camera_start.sh >/tmp/camera_start.log 2>&1 </dev/null 9>&- &
        if ! probe /depth_cam/depth/points cloud 25; then
            # Ask the same question again, now that a FRESH driver is up. If this one streams colour
            # and still has no depth, the driver was never the problem -- the view is. From a cold
            # start (no driver at all) we could not tell before; now we can, and we must, or a robot
            # parked in a corner reports a hardware fault and sends someone hunting a USB cable.
            if probe /depth_cam/rgb/image_raw image 4; then
                BLOCKED=1
            else
                fail "depth stream never opened, even relaunched alone. Check the USB cable and /tmp/camera_start.log"
            fi
        else
            echo "      relaunch fixed it."
        fi
    fi
fi
if [ "$BLOCKED" = "1" ]; then
    echo "      no depth, but COLOUR IS FLOWING -- the camera is healthy, the view is not."
else
    echo "      OK - depth flowing."
fi

# ---- [2/3] fake laser --------------------------------------------------------
# camera_scan_node goes up EVEN WHEN THE LENS IS BLOCKED, and that is deliberate. With no depth it
# publishes nothing, costs nothing, and sits there waiting. The moment the robot backs out of the
# corner, depth flows and /camera_scan is alive on the very next frame -- no repair, no restart,
# nobody pressing anything. Skipping it while blocked meant the robot could not heal on its own:
# the node was never there to come back to, so the web saw a camera that had "never come up",
# ran a repair every 60 s for ever, and reported a dead camera that was in perfect health.
#
# While blocked we check it by PROCESS, not by topic: a node with nothing to publish cannot prove
# it is alive by publishing.
echo "[2/3] fake laser: waiting for /camera_scan (up to 6 s)..."
# Anchored on ^python3: pgrep -f matches whole command lines, and camera_scan.sh and the web app
# both carry "camera_scan_node.py" inside their own pgrep command lines. Match the bare name and we
# can find someone else's SEARCH for the node and call the node alive. Only the node starts python3.
SCAN_NODE='^python3 .*camera_scan_node\.py'
if ! probe /camera_scan scan 6; then
    if [ "$BLOCKED" = "1" ]; then
        # It CANNOT publish -- there is no depth to convert. So judge it by the process table, and
        # only here. Trusting a pulse instead of a heartbeat is normally how you miss a wedged node,
        # which is why the healthy branch below still demands to see a real message.
        if pgrep -f "$SCAN_NODE" >/dev/null; then
            echo "      camera_scan_node is up, silent only because there is no depth to convert."
        else
            echo "      starting camera_scan_node, so it is ready the moment the view clears..."
            setsid nohup bash /home/ubuntu/ros2_ws/camera_scan.sh >/tmp/camera_scan.log 2>&1 </dev/null 9>&- &   # 9>&-: never hand the lock to a child
            sleep 5
            pgrep -f "$SCAN_NODE" >/dev/null || fail "camera_scan_node would not start. Check /tmp/camera_scan.log"
            echo "      started -- it will publish as soon as the camera can see again."
        fi
    else
        # Depth IS flowing and /camera_scan is STILL silent. The node is dead, or alive and wedged --
        # and a wedged one is the dangerous case: it has a pulse, so a process check would wave it
        # through, and the robot would drive at chair legs with a GREEN light on the dashboard.
        # So we restart it and demand an actual message. camera_scan.sh kills any previous instance
        # first, so a wedged node is REPLACED, never doubled.
        echo "      not publishing. Starting camera_scan_node..."
        setsid nohup bash /home/ubuntu/ros2_ws/camera_scan.sh >/tmp/camera_scan.log 2>&1 </dev/null 9>&- &   # 9>&-: never hand the lock to a child
        if ! probe /camera_scan scan 15; then
            fail "camera_scan_node would not come up. Check /tmp/camera_scan.log"
        fi
        echo "      started."
    fi
fi

if [ "$BLOCKED" = "1" ]; then
    echo -e "${RED}BLOCKED${NC} - the camera is FINE. Something is right in front of the lens: it"
    echo "          cannot measure anything closer than ~30 cm, so every depth point is invalid."
    echo "          A wall? A corner? Someone standing there? Move the robot clear -- it will come"
    echo "          back on its own. There is NOTHING to restart."
    exit 2
fi
echo "      OK - /camera_scan publishing."

# ---- [3/3] truth (only on request: needs a human to stage empty floor) -------
if [ "${1:-}" = "--floor" ]; then
    echo "[3/3] truth: robot must be FACING EMPTY FLOOR (nothing within 2.5 m)."
python3 - <<'EOF' || fail "ghost walls: obstacles reported on empty floor. The floor fit is off -- check the arm pose (arm_gesture.py scan) and rerun camera_calib.py"
import sys, time, numpy as np, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
MAX_MEDIAN_HITS = 20      # empty floor should be ~0; leave slack for noise specks
rclpy.init()
node = rclpy.create_node("camera_check_floor")
got = []
node.create_subscription(LaserScan, "/camera_scan", lambda m: got.append(m), qos_profile_sensor_data)
t0 = time.time()
while len(got) < 10 and time.time() - t0 < 15:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
if len(got) < 3:
    sys.exit(1)   # scan died mid-check
counts = [int(np.isfinite(np.array(m.ranges, dtype=np.float32)).sum()) for m in got]
med = int(np.median(counts))
print(f"      hits per scan (median of {len(counts)}): {med}")
sys.exit(0 if med <= MAX_MEDIAN_HITS else 1)
EOF
    echo "      OK - empty floor reads clean."
else
    echo "[3/3] truth check skipped (needs staged empty floor): bash camera_check.sh --floor"
fi

echo -e "${GREEN}GREEN${NC} - camera obstacle path is up."
