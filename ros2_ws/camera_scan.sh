#!/bin/bash
# camera_scan.sh - virtual 2D laser from the depth camera, for the LOW obstacles the
# lidar cannot see (it sweeps at 16.8 cm; chair feet and whiteboard legs live below).
#
# Uses camera_scan_node.py, which ignores the camera's TF on purpose: that TF comes from
# the arm (URDF + servo angles) and is wrong by ~20 deg of tilt, which throws floor points
# 10-35 cm into the air and floods the costmap with phantom obstacles. It uses instead the
# pose measured against the real floor, and publishes the scan already in base_footprint.
#
# Order matters:
#   python3 ~/ros2_ws/arm_gesture.py camera_forward   # always the SAME arm pose
#   python3 ~/ros2_ws/camera_calib.py                 # once, robot facing EMPTY FLOOR
#   bash    ~/ros2_ws/camera_scan.sh                  # -> /camera_scan
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/ubuntu/ros2_ws/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0
# ONE node, ever. Two camera_scan_nodes publish two different /camera_scan streams into the same
# costmap, out of step with each other, and the robot behaves like it is possessed. We had two of
# them, and four camera drivers, after the watchdog spent an hour relaunching a camera that kept
# flapping: each repair that got killed mid-way left its children behind.
if pgrep -f "python3 /home/ubuntu/ros2_ws/camera_scan_node.py" >/dev/null; then
    echo "camera_scan_node is already running -- killing it before starting a new one"
    pkill -INT -f "python3 /home/ubuntu/ros2_ws/camera_scan_node.py"
    sleep 2
    pkill -KILL -f "python3 /home/ubuntu/ros2_ws/camera_scan_node.py" 2>/dev/null   # it is pure python: safe
    sleep 1
fi

# ONE THREAD. This is the line that gave the robot three cores back.
#
# The node was at 356% -- 3.5 of the Jetson's 6 cores -- while its own stopwatch said the work took
# 29% of one. Both numbers were true. `top -H` showed why: SIX threads, each burning ~60%, on a
# six-core machine. That is not our code. That is OpenBLAS, which numpy hands its linear algebra to,
# and which by default starts one thread per core AND SPINS them while it waits.
#
# The matrices here are 3x3. There is nothing to parallelise. The threads did no work -- they
# busy-waited, and busy-waiting looks exactly like being busy: load average 15, Nav2 lurching, Whisper
# taking 8 s. Twice this node has been blamed for eating the machine, and twice the fix aimed at the
# wrong thing (the frame rate, then the point-cloud reader). It was never doing the work. It was
# spinning.
#
# MEASURE, DON'T GUESS -- and when a Python process shows more than 100% CPU, count its threads
# before you optimise a single line of it.
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1

# ON by default. This node has eaten this Jetson twice, and on both occasions the first guess about
# WHICH part of it was expensive turned out to be wrong. A line every five seconds saying how much
# of a core it is using, and where the milliseconds go, costs nothing and would have caught both.
# CAMERA_SCAN_PROFILE=0 to silence it.
export CAMERA_SCAN_PROFILE="${CAMERA_SCAN_PROFILE:-1}"

exec python3 /home/ubuntu/ros2_ws/camera_scan_node.py
