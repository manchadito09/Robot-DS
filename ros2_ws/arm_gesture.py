#!/usr/bin/env python3
# arm_gesture.py - play a friendly arm GESTURE by name (guide-robot expressiveness).
#
# Gestures map to the JetRover's own pre-recorded, collision-safe action groups in
# ~/software/arm_pc/ActionGroups (the vendor uses these in its demos, so they're
# known good). This is expressive movement only -- NO manipulation/pick-up, which
# is Phase 2. To add gestures like a real wave/greet, record them with the arm_pc
# app and drop <name>.d6a in that folder; they become usable by name automatically.
#
# Needs the base/servo stack running (ros_robot_controller + servo_controller nodes,
# which the bringup starts). Usage:
#   python3 ~/ros2_ws/arm_gesture.py rest     # neutral / home pose
#   python3 ~/ros2_ws/arm_gesture.py point    # arm extended -> "it's this way"
#   python3 ~/ros2_ws/arm_gesture.py raise    # arm up
#   python3 ~/ros2_ws/arm_gesture.py list     # show gestures + available action groups
import os
import sys
import time
import rclpy
from servo_controller_msgs.msg import ServosPosition
from servo_controller.action_group_controller import ActionGroupController

ACTION_PATH = "/home/ubuntu/software/arm_pc/ActionGroups"

# Friendly gesture -> vendor action group (.d6a). Only safe, non-manipulation poses.
GESTURES = {
    "rest":  "init",        # neutral / home (kept for internal use; no UI button)
    "home":  "init",
    # SCAN pose: wrist tilted ~34 deg down so the depth camera sees the floor just
    # ahead and catches LOW obstacles the lidar misses (chair legs). This is the pose
    # the camera calibration (camera_calib.yaml) MUST be measured against -- change
    # this mapping and you must recalibrate. Guide/navigation sets this at trip start.
    "scan":  "self_drive",  # Servo4/Wrist=280 -> camera looks down at the near floor
    # HAND-BUILT EXPRESSIVE GESTURES (measured on hardware 16-jul, finger on STOP).
    # Real limits Rodrigo measured: BASE (S1) is free to rotate (no collision) -> wide swings
    # ok; ELBOW (S3) 15 -> 500 lifts the whole forearm (the big move); SHOULDER (S2) stays 750;
    # WRIST (S4) 220-425. The arm now visibly RISES via the elbow, not just the wrist.
    "wave":  "wave",         # elbow lifts arm + wrist waves full range 220<->425 -- a real hello
    "greet": "wave",
    "bigwave": "big_wave",   # arm up high (elbow) + base swings wide 250<->750 -- big wave
    "raise": "raise_arm",    # arm rises high and holds 2s, then lowers -- "presenting"
    "look":  "look",         # turns to one side, pauses, the other -- like looking for someone
    # FUN gestures (measured on hardware 16-jul). base is free, shoulder 600-765, elbow 15-500,
    # wrist-pitch 220-425, wrist-roll S5 centre 500, gripper S6 0=open..1000=closed. All eased
    # (2-step rise/return) so the arm never jerks the chassis.
    "nod":   "nod",          # wrist bobs -- "yes"
    "shake": "shake",        # base wiggles left-right -- "no"
    "dance": "dance",        # quick little tremor (base + wrist) -- a silly dance
    "twirl": "twirl",        # arm up, the hand spins (S5) -- a flourish
    "mouth": "mouth",        # arm up, gripper opens/closes like a talking mouth (S6)
}


def available():
    try:
        return sorted(f[:-4] for f in os.listdir(ACTION_PATH) if f.endswith(".d6a"))
    except Exception:
        return []


def main():
    arg = (sys.argv[1] if len(sys.argv) > 1 else "").strip().lower()
    if not arg or arg == "list":
        print("Gestures:", ", ".join(sorted(GESTURES)))
        print("Action groups present:", ", ".join(available()) or "(none)")
        return

    action = GESTURES.get(arg, arg)     # unknown name -> try it as a raw action group
    if not os.path.exists(os.path.join(ACTION_PATH, action + ".d6a")):
        hint = " (record it with the arm_pc app first)" if arg in ("wave", "greet") else ""
        sys.exit('No arm gesture "%s" (action group "%s.d6a" not found)%s.' % (arg, action, hint))

    rclpy.init()
    node = rclpy.create_node("arm_gesture")
    pub = node.create_publisher(ServosPosition, "servo_controller", 1)
    time.sleep(0.8)                     # let the publisher connect to servo_controller
    try:
        ActionGroupController(pub, ACTION_PATH).run_action(action)
        print('Arm gesture "%s" done.' % arg)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
