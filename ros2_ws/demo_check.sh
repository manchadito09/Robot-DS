#!/bin/bash
# demo_check.sh - one command, one verdict: is the robot ready to demo?
#
# Run it BEFORE the camera starts rolling, not during. Everything it checks has bitten us at least
# once: a flat battery makes the robot stagger and thrash, a dead depth camera makes it drive blind
# into chair legs, a lost AMCL sends it to the wrong room, and the Claude daemon being down turns
# the voice into a 10-second pause.
#
#   bash ~/ros2_ws/demo_check.sh
#
# Exit 0 = READY, 1 = NOT READY (with the reason). It asks the web app, which already holds a live
# ROS node, instead of shelling out to `ros2` -- faster, and it cannot hang on a stale ROS daemon.
API=http://127.0.0.1:8000
GREEN='\033[0;32m'; RED='\033[0;31m'; YEL='\033[0;33m'; NC='\033[0m'
bad=0
ok()   { echo -e "  ${GREEN}OK${NC}   $1"; }
warn() { echo -e "  ${YEL}WARN${NC} $1"; }
fail() { echo -e "  ${RED}FAIL${NC} $1"; bad=1; }

echo "=== ROBOT-DS: ready to demo? ==="

# ---- the web app: everything else is read through it -------------------------
S=$(curl -s -m 5 "$API/api/status" 2>/dev/null)
if [ -z "$S" ]; then
    fail "the web app is not answering on $API"
    echo -e "\n${RED}NOT READY${NC} - start it: sudo systemctl restart robot-web"
    exit 1
fi
ok "web app answering"

val() { echo "$S" | python3 -c "import sys,json; print(json.load(sys.stdin).get('$1'))" 2>/dev/null; }

# ---- the address the QR points at --------------------------------------------
#
# The QR is not a fixed picture: it encodes http://<the robot's IP>:8000, and that IP comes from the
# office WiFi by DHCP. A card printed last week can point nowhere today, and the failure happens
# with a visitor holding a phone up to it, wondering what they did wrong. So print the URL here,
# every time, and check it against whatever is printed and lying at Base.
LANIP=$(python3 -c "
import socket
s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try: s.connect(('8.8.8.8',80)); print(s.getsockname()[0])
except Exception: print('127.0.0.1')
finally: s.close()" 2>/dev/null)
if [ "${LANIP:-127.0.0.1}" = "127.0.0.1" ]; then
    fail "the robot has NO WiFi address -- no phone can reach it, and the QR is useless"
else
    ok "phones reach it at http://${LANIP}:8000"
fi

# ---- IS THE QR ACTUALLY ON THE SCREEN? ---------------------------------------
#
# The QR on the robot's screen is the ONLY way a phone gets in, and nothing checked it.
#
# It is only there while the image viewer is OPEN. Close that window -- or simply never press
# "Robot Web QR" -- and the screen is blank. A visitor stands in front of the robot with a phone
# out and no way to reach it. Everything else can be perfect and the demo has no door.
QRIMG=/tmp/robot_qr.png
if ! pgrep -f "(eog|feh|xdg-open|eom|ristretto|gpicview).*robot_qr" >/dev/null 2>&1; then
    fail "the QR is NOT on the robot's screen -- no phone can get in. Press 'Robot Web QR'."
elif [ ! -f "$QRIMG" ]; then
    warn "a viewer is open but $QRIMG is gone -- press 'Robot Web QR' again"
else
    # DECODE IT. A QR on the screen is not the same as a QR that WORKS: it was drawn when the robot
    # started, and if the WiFi has handed it a new address since (DHCP does that), the code up there
    # now points at somebody else's phone. It would scan perfectly and go nowhere. So read it back
    # and check it against the address the robot actually has.
    QRURL=$(python3 -c "
import cv2
img = cv2.imread('$QRIMG')
txt, _, _ = cv2.QRCodeDetector().detectAndDecode(img)
print(txt or '')" 2>/dev/null)
    if [ -z "$QRURL" ]; then
        warn "the QR is on screen but could not be read back -- check it with a phone"
    elif [ "$QRURL" = "http://${LANIP}:8000" ]; then
        ok "the QR on the robot's screen points at $QRURL"
    else
        fail "the QR on the screen is STALE: it points at $QRURL, but the robot is now at http://${LANIP}:8000"
        echo "       It scans perfectly and goes nowhere. The WiFi gave the robot a new address."
        echo "       Press 'Robot Web QR' to redraw it."
    fi
fi

# ---- CPU: the first thing to look at, always ---------------------------------
#
# This check did not exist, and it should have been the first one written. Three times a single node
# has quietly eaten this Jetson, and every time it presented as FOUR separate bugs: Whisper taking
# 8 s, Piper 5 s, Claude 3 s, Nav2 lurching down the corridor. Nothing was slow. Nothing could get a
# core. And demo_check happily said READY through all of it, because it never once asked whether the
# machine had any CPU left.
#
# Six cores. Past a load of 6 the robot is starving. And name the process, because a load number on
# its own sends you hunting: the answer last time was one Python process burning 374% of the CPU to
# do 29% of the work -- six OpenBLAS threads busy-waiting over 3x3 matrices.
LOAD=$(cut -d' ' -f1 /proc/loadavg)
HOG=$(ps -eo pcpu=,comm= --sort=-pcpu | awk '$1 > 150 {printf "%s at %s%%  ", $2, $1}')
# Claude Code (the interactive one) eats a core and 560 MB. The warm claude-daemon pool does NOT --
# it is three IDLE `claude -p` processes at 1-2% each, and it is what makes the robot answer in 2 s
# instead of 10. `ps -C claude` cannot tell them apart, and telling Rodrigo to close "Claude" when
# the number is really the daemon would have him kill the very thing that keeps the voice fast.
# The interactive one is the one WITHOUT -p.
CLAUDE=$(ps -eo pcpu=,args= -C claude 2>/dev/null | grep -v -- ' -p ' | awk '{s+=$1} END{printf "%.0f", s+0}')
if awk -v l="$LOAD" 'BEGIN{exit !(l > 7.0)}'; then
    fail "load $LOAD on 6 cores -- the robot is STARVING. Voice, planning and the camera are all fighting for a core."
    [ -n "$HOG" ] && echo "       eating it: $HOG"
    echo "       a Python process over 100% is usually a thread pool SPINNING, not working."
    echo "       Count its threads before you optimise anything:  top -H -p <pid>"
    [ "${CLAUDE:-0}" -gt 5 ] 2>/dev/null && echo "       (Claude Code is using ${CLAUDE}% of it. Close it: /exit)"
elif awk -v l="$LOAD" 'BEGIN{exit !(l > 6.0)}'; then
    warn "load $LOAD on 6 cores -- right at the edge. ${HOG:-}"
    [ "${CLAUDE:-0}" -gt 5 ] 2>/dev/null && echo "       Claude Code is using ${CLAUDE}% of it. Close it before the demo: /exit"
else
    ok "load $LOAD on 6 cores"
fi

# ---- ONE of each. Two is the robot going mad ---------------------------------
#
# Two camera drivers fighting over one USB device, or two camera_scan_nodes publishing
# contradictory scans into the same costmap, and the robot behaves -- in Rodrigo's words -- like it
# is possessed. We have had FOUR drivers and TWO nodes at once, left behind by a watchdog that spent
# an hour "repairing" a camera that was merely covered. The repair loop is fixed. This is here so
# that if anything else ever spawns a second one, it is caught at the door and not in front of a
# customer. Anchored patterns: pgrep -f finds other pgreps otherwise.
DRV=$(pgrep -cf '^/opt/ros/humble/lib/rclcpp_components/component_container --ros-args -r __node:=camera_container')
SCAN=$(pgrep -cf '^python3 .*camera_scan_node\.py')
if [ "$DRV" -gt 1 ] || [ "$SCAN" -gt 1 ]; then
    fail "TWO OF SOMETHING: $DRV camera drivers, $SCAN camera_scan_nodes (must be 1 and 1)"
    echo "       Two /camera_scan streams feed one costmap and contradict each other."
    echo "       Kill them and press 'Robot Web QR':  pkill -INT -f camera_scan_node.py"
else
    ok "one camera driver, one camera_scan_node"
fi

# ---- the fixed lines are BAKED ------------------------------------------------
#
# The robot's fixed lines are baked to WAV once, standing still, and then just played. When one is
# missing it does not fail -- it quietly synthesises it LIVE, with Piper, WHILE DRIVING AND TALKING,
# which is a core for a second and makes Nav2 lurch. It still speaks. It is just late, and nothing
# says why. 12 of the 13 had never been baked at all and nobody knew.
#
# The cache key is a hash of the exact text, so editing one word in office_knowledge.yaml orphans
# its WAV. This is the check that catches that -- and "Robot Web QR" bakes them, so the fix is a
# button.
UNBAKED=$(python3 - <<'PY' 2>/dev/null
import os, sys, types
for p in ("~/ros2_ws/install/robot_ds_behavior/lib/python3.10/site-packages",
          "~/ros2_ws/src/robot_ds_behavior"):
    d = os.path.expanduser(p)
    if os.path.isdir(os.path.join(d, "robot_ds_behavior")):
        sys.path.insert(0, d); break
for m in ("rclpy","rclpy.node","rclpy.action","nav2_msgs","nav2_msgs.action",
          "action_msgs","action_msgs.msg"):
    sys.modules[m] = types.ModuleType(m)
sys.modules["rclpy.node"].Node = object
sys.modules["rclpy.action"].ActionClient = object
sys.modules["nav2_msgs.action"].NavigateToPose = object
sys.modules["action_msgs.msg"].GoalStatus = object
from robot_ds_behavior import guide as g
print(sum(1 for t in g.NARRATE_LINES if not g._cached_wav(t)))
PY
)
if [ "${UNBAKED:-?}" = "0" ]; then
    ok "every fixed line is pre-baked (no Piper while driving)"
elif [ -n "$UNBAKED" ]; then
    fail "$UNBAKED spoken lines are NOT baked -- Piper will synthesise them WHILE DRIVING"
    echo "       It still speaks, just late, and nothing tells you why."
    echo "       Fix:  python3 ~/ros2_ws/prebake_lines.py    (or press 'Robot Web QR')"
else
    warn "could not check the baked lines"
fi

# ---- is Nav2 aborting? --------------------------------------------------------
#
# demo_check said READY through a session in which Nav2 gave up on 162 goals. It never asked. The
# robot was caging itself in phantom obstacles and saying "I couldn't reach Developers", and the
# pre-flight check was perfectly happy. "Controller patience exceeded" is Nav2 aborting BY ITSELF --
# pressing STOP does not produce it -- so it is the one number that says whether the cage is back.
NAVLOG_PID=$(pgrep -f 'component_container_isolated' | head -1)
if [ -n "$NAVLOG_PID" ]; then
    NAVLOG=$(ls -t "$HOME/.ros/log/component_container_isolated_${NAVLOG_PID}"_*.log 2>/dev/null | head -1)
fi
if [ -n "${NAVLOG:-}" ] && [ -f "$NAVLOG" ]; then
    AB=$(grep -c "Controller patience exceeded" "$NAVLOG")
    if [ "$AB" -eq 0 ]; then
        ok "Nav2 has not aborted a single goal this run"
    elif [ "$AB" -lt 5 ]; then
        warn "Nav2 aborted $AB goal(s) this run -- watch it (bash ~/ros2_ws/nav2_errors.sh)"
    else
        fail "Nav2 has ABORTED $AB goals this run -- it is caging itself again"
        echo "       bash ~/ros2_ws/nav2_errors.sh          # the full picture"
        echo "       bash ~/ros2_ws/camera_check.sh --floor # ghost obstacles? (empty floor)"
    fi
fi

# ---- battery ----------------------------------------------------------------
V=$(val battery_v)
if [ "$V" = "None" ]; then
    warn "battery: no reading"
elif python3 -c "import sys; sys.exit(0 if float('$V') > 11.0 else 1)"; then
    ok "battery $V V"
elif python3 -c "import sys; sys.exit(0 if float('$V') > 5.0 else 1)"; then
    fail "battery $V V -- under 11 V the motors sag: it stalls, jitters, and gets stuck. CHARGE IT."
else
    fail "battery $V V = NO BATTERY (running off the Jetson cable). No lidar, no servos, no driving."
fi

# ---- lidar ------------------------------------------------------------------
[ "$(val lidar)" = "True" ] && ok "lidar publishing" || fail "lidar silent -- no obstacle sensing, no localization"

# ---- depth camera: the low-obstacle path ------------------------------------
case "$(val camera)" in
    ok)      ok "depth camera: the low-obstacle path is live" ;;
    # BLOCKED is not a fault, and it must not read like one. The camera is healthy and streaming;
    # something is simply within its 30 cm blind spot, so every depth point is invalid. Parking the
    # robot nose-first in a corner does this, and it sent us hunting a USB cable for half a day.
    # The fix is to move the robot, not to restart anything.
    blocked) fail "depth camera BLOCKED -- it is FINE, but something is right in front of it (a wall? a corner? someone standing there?). Move the robot clear and re-check." ;;
    stale)   fail "depth camera DIED -- the robot cannot see chair legs. Press 'Robot Web QR'." ;;
    *)       fail "depth camera never came up -- the robot is blind to chair legs. Press 'Robot Web QR'." ;;
esac

# ---- localization -----------------------------------------------------------
# "localized" only ever meant "an /amcl_pose message arrived", and AMCL always sends one -- even a
# wrong one. So this said OK with the robot in the wrong room. It is a pulse, not an answer: keep it
# (no pose at all is still a failure) but never let it be the last word.
# A STANDING ROBOT STOPS PUBLISHING ITS POSE, and that is not a fault.
#
# AMCL updates the filter when the ODOMETRY says the robot has moved. Park it, and it has nothing
# new to say, so /amcl_pose goes quiet -- and the web's 30-second freshness window then reports
# "not localized". Which is exactly the state a robot is in when you run this check: standing at
# Base, about to be asked to go somewhere.
#
# So this is a WARN, never a fail. The real question -- is the robot where it thinks it is -- is
# answered below, against the LIDAR, by pose_check, and that works whether it is moving or not
# (it reads the map->lidar transform, which AMCL keeps publishing regardless).
if [ "$(val localized)" = "True" ]; then
    ok "AMCL is publishing a pose"
else
    warn "no fresh /amcl_pose -- normal for a robot standing still (AMCL only speaks when it moves)"
fi

# THE REAL QUESTION: is the robot WHERE IT THINKS IT IS?
#
# Navigation boots AMCL already parked at Base so no visitor ever has to localize a robot. But AMCL
# believes that pose whether or not it is true -- shove the robot in a corner and it is confidently,
# precisely wrong, and silent about it. pose_check.py asks the lidar: do these walls exist where the
# map says they do? Endpoints on walls = the belief is real. Endpoints in open space = it is not.
OUT=$(python3 "$HOME/ros2_ws/pose_check.py" 2>&1); RC=$?
case $RC in
    0) ok "the robot really IS where it thinks it is ($(echo "$OUT" | grep -oE '[0-9.]+ cm and [0-9.]+ deg away' | head -1 | sed 's/away/from Base/'))" ;;
    1) fail "MISLOCALIZED -- the robot is NOT where AMCL thinks. Park it at Base and relaunch Navigation." ;;
    *) warn "could not verify the pose (no lidar? no Nav2?) -- $(echo "$OUT" | head -1)" ;;
esac

# ---- navigation stack -------------------------------------------------------
pgrep -f 'nav2_container' >/dev/null && ok "Nav2 running" \
    || fail "Nav2 is not running -- open the NAVIGATION icon"

# ---- the voice brain --------------------------------------------------------
# "is the service active" is a pulse, not an answer -- the same mistake "localized" made. A daemon
# can be up and still be answering in ten seconds. So ASK it, and time the reply: voice_check.py
# drives the whole chain (Piper -> Whisper -> Claude) with a real sentence and a stopwatch.
#
# This is the failure that never announces itself. Nothing crashes and nothing logs; the robot just
# takes ten seconds to say hello, on camera. Warm on an idle Jetson: Whisper ~2 s, Claude ~2.8 s,
# Piper ~1.5 s -- about 6 s from the visitor going quiet to the robot speaking.
# CAN IT BE ASKED THINGS WHILE IT WALKS?
# On the way it says "ask me anything -- I know the company, not only the corridors". If the
# installed brain has no --no-drive, /api/talk refuses questions during a trip, and the robot
# invites the visitor to ask and then tells them it is busy. That is worse than never inviting
# them, and the whole gap is one forgotten `colcon build` wide.
[ "$(val chat_while_driving)" = "True" ] && ok "it can answer questions WHILE it walks" \
    || fail "it INVITES questions on the walk but cannot answer them -- run: colcon build --packages-select robot_ds_behavior, then restart robot-web"

VOICE=$(python3 "$HOME/ros2_ws/voice_check.py" 2>&1)
if [ $? -eq 0 ]; then
    ok "voice: $(echo "$VOICE" | grep '^SUMMARY:' | cut -d' ' -f2-)"
else
    fail "the VOICE path is broken or crawling:"
    echo "$VOICE" | grep -E "FAIL|SLOW|unreachable|NOTHING" | sed 's/^/       /'
fi

# ---- the speaker ------------------------------------------------------------
# We mute this while testing in a working office (amixer -c Device sset Speaker 0% mute), and the robot
# gives no sign of it: it still "speaks" -- text in the web chat, text in the logs, same timings --
# it just makes no sound. A guide robot that narrates a whole tour in silence, on camera, is a very
# quiet kind of disaster. So check the speaker itself, not whether the software thinks it spoke.
SPK=$(amixer -c Device sget Speaker 2>/dev/null | grep -o '\[on\]\|\[off\]' | head -1)
VOL=$(amixer -c Device sget Speaker 2>/dev/null | grep -o '\[[0-9]*%\]' | head -1 | tr -d '[]%')
# The FLAG is checked first, and it fails even if the volume happens to be up. It means somebody ran
# `speaker.sh mute` to test in a room full of people -- and while it is there, "Robot Web QR" will
# NOT restore the voice on the next press. A speaker that is loud right now but will go silent at
# the next startup is the worst of the three states, because it passes every check you would think
# to run. This is the check nobody would think to run.
if [ -f "$HOME/.cache/robot_ds/speaker_muted" ]; then
    fail "the speaker was MUTED ON PURPOSE and never given back ($(cat "$HOME/.cache/robot_ds/speaker_muted" 2>/dev/null))"
    echo "       The robot will narrate the whole demo in silence, and give no sign of it."
    echo "       Give it its voice back:  bash ~/ros2_ws/speaker.sh unmute"
elif [ "$SPK" = "[off]" ] || [ "${VOL:-0}" -lt 20 ]; then
    fail "the speaker is MUTED (${VOL:-0}%) -- the robot will narrate the whole demo in silence."
    echo "       give it its voice back:  bash ~/ros2_ws/speaker.sh unmute"
elif [ -z "$SPK" ]; then
    warn "could not read the speaker (is the USB audio device plugged in?)"
else
    ok "speaker on (${VOL}%)"
fi

# ---- destinations -----------------------------------------------------------
python3 - <<'EOF'
import sys, yaml, math
import numpy as np
MAPD = '/home/ubuntu/ros2_ws/src/slam/maps'
try:
    wps = yaml.safe_load(open(f'{MAPD}/map_01.waypoints.yaml'))['waypoints']
    y = yaml.safe_load(open(f'{MAPD}/map_01.yaml'))
    res, ox, oy = y['resolution'], y['origin'][0], y['origin'][1]
    f = open(f'{MAPD}/map_01.pgm', 'rb'); assert f.readline().strip() == b'P5'
    l = f.readline()
    while l.startswith(b'#'):
        l = f.readline()
    w, h = map(int, l.split()); int(f.readline())
    img = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
    occ = img < 100
    NEED = 0.36                      # robot_radius 0.16 + inflation 0.20
    bad = []
    for n, p in wps.items():
        c = int((p['x'] - ox) / res); r = h - 1 - int((p['y'] - oy) / res)
        win = occ[max(0, r-40):r+41, max(0, c-40):c+41]
        ys, xs = np.where(win)
        d = np.min(np.hypot(ys - (r - max(0, r-40)), xs - (c - max(0, c-40)))) * res if len(ys) else 9.9
        if img[r, c] < 100 or d < NEED:
            bad.append(f'{n} ({d:.2f} m)')
    if bad:
        print(f'  \033[0;31mFAIL\033[0m too close to a wall, Nav2 may refuse to reach them: {", ".join(bad)}')
        sys.exit(1)
    print(f'  \033[0;32mOK\033[0m   {len(wps)} destinations, all on free floor with room to stop')
except Exception as e:
    print(f'  \033[0;33mWARN\033[0m could not check the destinations: {e}')
EOF
[ $? -ne 0 ] && bad=1

echo
if [ $bad -eq 0 ]; then
    echo -e "${GREEN}READY${NC} - go."
else
    echo -e "${RED}NOT READY${NC} - fix the FAILs above first."
    exit 1
fi
