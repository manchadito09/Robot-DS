#!/bin/bash
# camera_start.sh - launch the Orbbec depth camera on its own.
#
# The camera the navigation launch starts often comes up with COLOR ONLY: it loses the
# USB race during the boot storm and the depth stream never opens (no /depth_cam/depth/*,
# no point cloud). Relaunching it by itself, once the system is calm, brings up depth too.
#
#   pkill -INT -f 'component_container.*depth_cam'   # stop the broken one (NEVER kill -9)
#   bash ~/ros2_ws/camera_start.sh                   # then start this one
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/ubuntu/ros2_ws/install/setup.bash 2>/dev/null
export need_compile=False
export DEPTH_CAMERA_TYPE=Dabai
export ROS_DOMAIN_ID=0
# ONE driver, ever. Two of them fight over the same USB device and BOTH end up half-dead -- and
# then the watchdog, seeing no depth, launches a third. We ended up with FOUR camera drivers and two
# camera_scan_nodes after an hour of that, the costmap fed contradictory scans, and the robot
# behaved like it was possessed. Kill whatever is there before opening the USB again.
# SIGINT, never -9: the Orbbec driver leaves the sensor wedged if it is killed hard.
if pgrep -f 'component_container.*depth_cam' >/dev/null; then
    echo "a camera driver is already running -- stopping it before starting another"
    pkill -INT -f 'component_container.*depth_cam'
    pkill -INT -f 'ros2 launch peripherals depth_camera'
    for _ in $(seq 1 10); do
        pgrep -f 'component_container.*depth_cam' >/dev/null || break
        sleep 1
    done
fi

exec ros2 launch peripherals depth_camera.launch.py
