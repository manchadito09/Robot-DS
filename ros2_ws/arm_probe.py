#!/usr/bin/env python3
# arm_probe.py - SAFE, interactive limit finder for ONE arm servo.
#
# Purpose: measure the real mechanical range of a single servo (shoulder S2 or
# elbow S3) by hand. You move it ONE small step at a time and watch. Nothing is
# automatic. Finger on STOP. Standalone script (NO colcon build), like arm_gesture.py.
#
# Servo columns -> bus ids (from action_group_controller.py): S1..S5 = ids 1..5, S6 = id 10.
# Rest / init pose: S1=500 S2=765 S3=15 S4=220 S5=500 S6=500.
# Only the CHOSEN servo moves; every other servo is held at its rest value each step.
#
#   python3 ~/ros2_ws/arm_probe.py s1     # probe BASE rotation (the one to watch)
#   python3 ~/ros2_ws/arm_probe.py s2     # probe shoulder
#   python3 ~/ros2_ws/arm_probe.py s3     # probe elbow
#
# For the BASE (s1): rest is 500 (centre). Step DOWN finds the left limit, then use
# 'x' to flip and step UP to find the right limit. Note BOTH numbers.
#
# Commands (typed after each step):
#   ENTER  -> move ONE step in the current direction
#   x      -> flip direction (down <-> up)
#   <num>  -> change step size (e.g. 5 for fine, 20 for coarse)
#   h      -> go back to rest value (this servo)
#   q      -> quit and print the LAST value reached
import sys
import time
import rclpy
from servo_controller_msgs.msg import ServosPosition, ServoPosition

REST = {1: 500, 2: 765, 3: 15, 4: 220, 5: 500, 10: 500}  # id -> rest pulse
COL_TO_ID = {1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 10}        # table col -> bus id

HARD_MIN, HARD_MAX = 0, 1000   # servo electrical range; mechanical limit is what we FIND
STEP_DEFAULT = 10              # pulses per step (small on purpose)
MOVE_TIME = 0.6               # seconds per move (gentle)


def clamp(v):
    return max(HARD_MIN, min(HARD_MAX, v))


def publish(pub, target_id, value):
    msg = ServosPosition()
    msg.position_unit = 'pulse'
    msg.duration = MOVE_TIME
    data = []
    for sid, rest in REST.items():
        s = ServoPosition()
        s.id = sid
        s.position = float(value if sid == target_id else rest)
        data.append(s)
    msg.position = data
    pub.publish(msg)


def main():
    arg = (sys.argv[1] if len(sys.argv) > 1 else "").strip().lower().lstrip('s')
    if arg not in ("1", "2", "3"):
        print("Usage: python3 arm_probe.py s1 (base) | s2 (shoulder) | s3 (elbow)")
        return
    col = int(arg)
    target_id = COL_TO_ID[col]
    rest_val = REST[target_id]
    name = {1: "BASE (S1)", 2: "SHOULDER (S2)", 3: "ELBOW (S3)"}[col]

    rclpy.init()
    node = rclpy.create_node("arm_probe")
    pub = node.create_publisher(ServosPosition, "servo_controller", 1)
    time.sleep(0.8)

    print("=" * 56)
    print("  PROBE %s   rest=%d   step=%d" % (name, rest_val, STEP_DEFAULT))
    print("  Finger on STOP. Small steps. Watch it. Don't rush.")
    print("  ENTER=step   x=flip dir   <num>=step size   h=home   q=quit")
    print("=" * 56)

    # go to rest first, known start
    publish(pub, target_id, rest_val)
    time.sleep(1.2)

    cur = rest_val
    step = STEP_DEFAULT
    direction = -1  # start going DOWN (toward 0); flip with 'x'
    try:
        while True:
            arrow = "DOWN v" if direction < 0 else "UP ^"
            try:
                cmd = input("  %s = %d   [%s, step %d] > " % (name, cur, arrow, step)).strip().lower()
            except EOFError:
                cmd = "q"
            if cmd == "q":
                break
            elif cmd == "x":
                direction *= -1
            elif cmd == "h":
                cur = rest_val
                publish(pub, target_id, cur)
                time.sleep(MOVE_TIME)
            elif cmd.isdigit():
                step = max(1, int(cmd))
            elif cmd == "":
                nxt = clamp(cur + direction * step)
                if nxt == cur:
                    print("  -- at hard limit %d, cannot go further this way --" % cur)
                    continue
                cur = nxt
                publish(pub, target_id, cur)
                time.sleep(MOVE_TIME)
            else:
                print("  ? unknown. ENTER=step  x=flip  <num>=step size  h=home  q=quit")
    finally:
        print("")
        print(">>> LAST VALUE for %s = %d   (rest was %d)" % (name, cur, rest_val))
        print(">>> tell Rodrigo's Claude this number. NOT returning to rest (staying put).")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
