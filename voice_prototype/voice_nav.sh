#!/usr/bin/env bash
# voice_nav.sh - rosita-side helper for the laptop voice front-end (voice_talk.py).
# Deploy to rosita's home:  scp voice_nav.sh rosita:~/   (or it's already there)
#
# Takes the spoken text as $1, lets Claude pick a POI and drives there (stay),
# streaming the narration back so the laptop can speak it. On the real robot this
# isn't needed (voice + brain + Nav2 all run on the robot).
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash   # humble on the robot, jazzy on rosita
cd ~/ros2_ws/src/robot_ds_behavior/robot_ds_behavior
python3 -u brain.py --stay "$1"
