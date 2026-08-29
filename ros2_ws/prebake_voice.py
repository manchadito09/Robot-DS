#!/usr/bin/env python3
# prebake_voice.py - bake the robot's fixed spoken lines to WAV, ONCE, standing still.
#
#     python3 ~/ros2_ws/prebake_voice.py                    # bake guide.py's mid-trip lines
#     python3 ~/ros2_ws/prebake_voice.py "Excuse me"        # bake extra lines too
#     python3 ~/ros2_ws/prebake_voice.py --list             # just show what is baked
#
# WHY: Piper eats a core for ~a second to synthesise a line. Doing that WHILE Nav2 is driving is
# how the robot ends up stuttering (the 334%-CPU lesson). The mid-trip lines never change, so we
# bake them here, once, with the robot parked -- and driving then costs nothing but an aplay.
#
# WHY THE FAKE MODULES BELOW: the lines, the voice model and the cache-filename rule all live in
# guide.py, and guide.py imports rclpy/nav2_msgs at the top. This script must run with NO ROS (no
# colcon build, no sourcing, robot on the bench). So we put stand-ins for those modules in
# sys.modules FIRST, then import guide.py for real. Nothing ROS is ever called -- we only read its
# constants and call its _prebake(). Doing it this way (instead of copying the phrases in here)
# means editing a line in guide.py can never leave this script baking the wrong file.
import os
import sys
import types

SRC = os.path.expanduser("~/ros2_ws/src/robot_ds_behavior/robot_ds_behavior")


class _Stub:
    """Stands in for Node / ActionClient / message types. Never called, only inherited from."""

    def __init__(self, *a, **k):
        pass

    def __getattr__(self, _):
        return lambda *a, **k: None


def _fake(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m


_fake("rclpy", init=lambda *a, **k: None, shutdown=lambda *a, **k: None,
      spin_until_future_complete=lambda *a, **k: None)
_fake("rclpy.node", Node=_Stub)
_fake("rclpy.action", ActionClient=_Stub)
_fake("nav2_msgs")
_fake("nav2_msgs.action", NavigateToPose=_Stub)
_fake("action_msgs")
_fake("action_msgs.msg", GoalStatus=_Stub)

if not os.path.isdir(SRC):
    sys.exit(f"can't find the guide source at {SRC}")
sys.path.insert(0, SRC)          # so guide.py's `from pois import ...` fallback resolves

import guide                      # noqa: E402  (the fakes above must land first)


def main():
    args = [a for a in sys.argv[1:] if a != "--list"]
    lines = list(guide.NARRATE_LINES) + args

    print(f"voice model : {guide.PIPER_MODEL}")
    print(f"piper       : {guide.PIPER_BIN}")
    print(f"cache       : {guide.CACHE_DIR}")
    if not os.path.exists(guide.PIPER_BIN):
        sys.exit("piper binary not found -- nothing baked (the robot will still speak, "
                 "it will just synthesise on the fly)")
    if not os.path.exists(guide.PIPER_MODEL):
        sys.exit("voice model not found -- nothing baked")
    print()

    if "--list" not in sys.argv:
        guide._prebake(lines)     # skips whatever is already baked

    ok = True
    for text in lines:
        p = guide._wav_path(text)
        if os.path.exists(p):
            kb = os.path.getsize(p) / 1024.0
            secs = (os.path.getsize(p) - 44) / (22050.0 * 2)   # 16-bit mono at the model's rate
            print(f"  OK    {kb:6.1f} KB  {secs:4.1f}s  \"{text}\"")
        else:
            ok = False
            print(f"  MISS                  \"{text}\"")
    print()
    print("baked. driving now costs zero CPU for these lines." if ok else
          "some lines did NOT bake -- the robot will synthesise those on the fly (slower, "
          "but it still speaks).")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
