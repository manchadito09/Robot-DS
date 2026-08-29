#!/usr/bin/env bash
# Start-of-session button: restart the web app, start the camera, then show a QR
# "scan me" card so a phone on the same WiFi can open it. (The Jetson has no working
# browser.)
#
# WHY THE RESTART: the web keeps serving HTTP even after its ROS node dies (it then
# shows no camera, no robot dot, and a STALE battery reading that hides the problem).
# 'Show QR' is pressed at the start of a session -- after Navigation is up -- so it is
# the right moment to give it a fresh ROS node. It is SKIPPED while the robot is busy,
# so pressing this mid-demo can never cut a trip short.
#
# WHY THE CAMERA: the navigation no longer starts the camera driver (use_camera=false,
# so that stopping the camera cannot take the whole nav stack down with it). Nothing
# else starts it, so the web's video stayed black no matter how often this was pressed.
# We start it here, only if nobody is already publishing.
#
# Launched from the desktop icon (show_qr.desktop). A desktop launcher shows no output
# when it fails, so everything is logged to /tmp/show_qr.log.
LOG=/tmp/show_qr.log
API=http://127.0.0.1:8000
echo "--- $(date) show_qr ---" >>"$LOG"

# ---- Everything the robot needs, made true. Press this and nothing else. ---------------------
# One button at the start of a session. Everything below has failed silently at least once, and a
# silent failure in this lot is the kind you only discover with a customer watching.

# THE SPEAKER. We mute it while testing in a shared office, and a muted robot gives NO sign of it:
# it still "speaks" -- text in the web chat, text in the logs, same timings -- it just makes no
# sound. A guide robot narrating a whole demo in silence, on camera, is a very quiet disaster.
# By card NAME: plugging the 6-mic array in renumbers the cards and pushes the speaker from 1 to 0,
# and every hard-coded plughw:1,0 then points at a microphone.
#
# UNLESS IT WAS MUTED ON PURPOSE. Testing in a room full of people working, you mute the robot and
# read what it says in the phone's chat -- and then this button would hand it its voice straight
# back, mid-sentence, in an open-plan office. So a deliberate mute (speaker.sh mute) leaves a flag,
# and the flag is respected here. It is also the first thing demo_check.sh FAILS on, so a mute set
# on a Tuesday cannot survive quietly into Thursday.
if [ -f "$HOME/.cache/robot_ds/speaker_muted" ]; then
    echo "speaker: MUTED ON PURPOSE (speaker.sh mute) -- leaving it alone" >>"$LOG"
    notify-send -u normal "Robot speaker: MUTED" \
        "Muted on purpose. It will say everything -- read it in the phone chat. Give it its voice back with: bash ~/ros2_ws/speaker.sh unmute" 2>/dev/null
elif amixer -c Device -q sset Speaker 100% unmute 2>>"$LOG"; then
    echo "speaker: on, 100%" >>"$LOG"
else
    echo "speaker: COULD NOT SET IT -- is the USB audio device plugged in?" >>"$LOG"
fi

# THE WARM DAEMONS. Both are systemd services and both start at boot, so this is a belt-and-braces
# check, not a launcher. Without the Claude one, every answer takes ~10 s instead of ~2 (it cold-
# starts `claude -p` per sentence). Without the Whisper one, every message the visitor speaks pays
# 2.9 s to load the model again, for 0.25 s of actual transcription.
for svc in claude-daemon stt-daemon; do
    if systemctl is-active --quiet "$svc"; then
        echo "$svc: up" >>"$LOG"
    else
        echo "$svc: DOWN -> starting it" >>"$LOG"
        sudo -n systemctl start "$svc" >>"$LOG" 2>&1
    fi
done

busy=$(curl -s -m 3 "$API/api/status" 2>/dev/null | grep -o '"busy": *true')
if [ -n "$busy" ]; then
    echo "robot is busy -> not restarting the web" >>"$LOG"
    systemctl is-active --quiet robot-web || sudo -n systemctl start robot-web
else
    echo "restarting robot-web (fresh ROS node)" >>"$LOG"
    sudo -n systemctl restart robot-web
fi

# It takes a few seconds to import ROS; wait until it answers again.
for _ in $(seq 1 30); do
    curl -s -m 2 "$API/api/status" >/dev/null 2>&1 && break
    sleep 1
done
echo "web up: $(curl -s -m 3 "$API/api/status" 2>/dev/null | head -c 80)" >>"$LOG"

# THE CAMERA -- the whole chain, not just the driver.
#
# This used to start camera_start.sh only. That gives the phone its video, which looks like
# success, but it is HALF the chain:
#
#   driver -> /depth_cam/depth/points -> camera_scan_node -> /camera_scan -> Nav2 costmap
#   ^^^^^^                               ^^^^^^^^^^^^^^^^
#   video for the phone                  the part that stops the robot hitting chair legs
#
# Nobody started camera_scan_node, so the robot drove BLIND to low obstacles while the phone
# happily showed a picture. It cost us a morning: the robot kept clipping a chair and we blamed
# the costmap layer, when the truth was that nothing was feeding it.
#
# camera_check.sh does the lot, and repairs what it can: relaunches the camera if the depth
# stream never opened (it often boots colour-only), starts camera_scan_node if /camera_scan is
# silent, and exits 0 GREEN / 1 RED. It is safe to run when the camera is already up: it probes
# first and only touches what is broken.
# IN THE BACKGROUND. camera_check.sh probes and repairs, and its waits add up: up to 10 s for depth,
# 25 s more if the camera has to be relaunched, 15 s for camera_scan_node. Waiting for all that
# before drawing the QR meant a minute-plus staring at nothing, for a card that has nothing to do
# with the camera. Kick it off and move on -- and if it does fail, the web app's camera watchdog
# retries on its own anyway, so nothing is lost by not blocking here.
# setsid: this script ends by exec'ing the image viewer, and a Ctrl-C to close that viewer would
# otherwise take the camera down with it (SIGINT hits the whole process group). That is exactly how
# the camera kept "dying on its own".
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/ubuntu/ros2_ws/install/setup.bash 2>/dev/null

# A desktop launcher can start with no DISPLAY -> the image viewer dies silently
# with "cannot open display". Find the user's graphical session and use it.
# Set BEFORE the camera goes up: the camera's verdict is a desktop notification, and it needs this.
if [ -z "$DISPLAY" ]; then
    D=$(who 2>/dev/null | awk '$2 ~ /^:/ {print $2; exit}')
    export DISPLAY="${D:-:0}"
fi
echo "DISPLAY=$DISPLAY" >>"$LOG"

# THE ARM, BEFORE THE CAMERA. camera_scan_node does not trust a saved angle: it fits the REAL floor
# in front of the robot, live, every half second, and projects with that. Which only works if the
# camera is actually LOOKING at the near floor -- the 'scan' pose. With the arm anywhere else there
# is too little floor in view to fit a plane, it falls back to the saved camera_calib.yaml (measured
# in the scan pose, so wrong for any other), and the floor lands 10-35 cm up in the air: phantom
# obstacles, and a robot that paints a cage around itself and stops dead in an empty corridor.
# Nothing else put the arm here at startup. A trip did it -- so the check you ran BEFORE the first
# trip was measuring a pose the robot was not in.
# Needs the servos, which need the battery: on cable power this fails, and says so.
echo "arm -> scan pose (the camera must look at the near floor)" >>"$LOG"
if ! python3 /home/ubuntu/ros2_ws/arm_gesture.py scan >>"$LOG" 2>&1; then
    echo "arm: COULD NOT SET THE SCAN POSE -- is the battery in? (no battery = no servos)" >>"$LOG"
    notify-send -u critical "Robot: arm" "Could not set the scan pose. Is the battery in?" 2>/dev/null
fi

# THE VOICE, BAKED AHEAD OF TIME.
#
# The robot's fixed lines -- the 12 things it says mid-walk, and "almost there" -- are baked to WAV
# once and then just played. guide.py does bake them, in a daemon thread... which it starts when the
# Guide node is built, PER TRIP, and which dies with the trip. A short trip kills the bake half-way;
# the next trip starts over and dies again. They never finished: 12 of the 13 had never been baked
# at all, and every one of them was being synthesised LIVE, by Piper, on this Jetson, WHILE the
# robot was driving and talking -- the exact cost pre-baking exists to avoid.
#
# So bake them out here, standing still, before anyone asks for a trip. It skips whatever it already
# has, so this is instant on every start but the first -- and it re-bakes automatically after you
# edit office_knowledge.yaml, because the cache key is a hash of the text.
echo "baking the fixed spoken lines (instant if they are already baked)" >>"$LOG"
setsid nohup python3 /home/ubuntu/ros2_ws/prebake_lines.py >>"$LOG" 2>&1 </dev/null &

# THE CAMERA -- in the background, but its verdict is NOT silent any more.
# camera_check.sh probes and repairs and its waits add up (up to ~50 s), so we do not block the QR
# on it. But it used to disappear into /tmp/show_qr.log, and a RED camera then announced itself the
# way you least want: the robot not moving, with a customer watching. Now it pops up on screen.
echo "starting the camera in the background (camera_check.sh) -- see below for its verdict" >>"$LOG"
setsid nohup bash -c '
    bash /home/ubuntu/ros2_ws/camera_check.sh >>"'"$LOG"'" 2>&1
    case $? in
      0) notify-send -u normal   "Robot camera: GREEN" \
           "Depth + /camera_scan are up. It can see chair legs." ;;
      2) notify-send -u critical "Robot camera: BLOCKED" \
           "The camera is FINE -- something is right in front of it. Move the robot away from the wall/corner, then press this again." ;;
      *) notify-send -u critical "Robot camera: RED" \
           "It CANNOT see low obstacles. See /tmp/show_qr.log" ;;
    esac

    # IS THE ROBOT WHERE IT THINKS IT IS?
    #
    # Navigation boots AMCL already parked at Base, so a visitor never has to localize a robot --
    # no RViz, no tapping the map. That is the point. But AMCL believes that pose whether or not it
    # is TRUE: leave the robot in a corner, press the buttons, and it is confidently, precisely
    # wrong. And it says nothing -- the app happily reported "localized", because all that ever
    # meant was that AMCL had spoken.
    #
    # So we ask the lidar whether those walls are really there. It is the one failure that has to be
    # caught HERE, while you are setting up: by the time it shows itself, the robot is driving into
    # a wall with a customer watching.
    #
    # Navigation is up by now (it is pressed before this button), so the pose exists to check.
    pose=$(python3 /home/ubuntu/ros2_ws/pose_check.py 2>&1); rc=$?
    echo "$pose" >>"'"$LOG"'"
    case $rc in
      0) : ;;   # agrees -- say nothing. Nothing to do is not news.
      1) notify-send -u critical "Robot is NOT at Base" \
           "AMCL is confidently WRONG -- the lidar sees walls the map does not have. Park the robot at Base, relaunch Navigation, and press this again." ;;
      *) notify-send -u normal   "Pose not verified" \
           "Could not check where the robot is (no lidar? Navigation not up? battery out?). See /tmp/show_qr.log" ;;
    esac
' >>"$LOG" 2>&1 </dev/null &

IMG=$(python3 /home/ubuntu/Robot-DS/robot_web/show_qr.py 2>>"$LOG")
echo "IMG=$IMG" >>"$LOG"
if [ -z "$IMG" ] || [ ! -f "$IMG" ]; then
    echo "no QR image produced" >>"$LOG"
    exit 1
fi

# Try the image viewers in order; the first one present wins.
for viewer in eog xdg-open feh; do
    if command -v "$viewer" >/dev/null 2>&1; then
        echo "opening with $viewer" >>"$LOG"
        exec "$viewer" "$IMG"
    fi
done
echo "no image viewer found" >>"$LOG"
exit 1
