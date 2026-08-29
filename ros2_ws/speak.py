#!/usr/bin/env python3
# speak.py - say ONE line out loud, and nothing else.
#
#   python3 ~/ros2_ws/speak.py "I lost my depth camera"
#   python3 ~/ros2_ws/speak.py --optional "Hi Jonny!"     # drop it if the robot is already talking
#
# The web app already speaks through guide/brain, but those go through Claude to decide WHAT to
# say. For a fixed alert -- the camera watchdog -- we know the words already; we just need a mouth.
# Reuses guide.say() so it sounds like the robot (Piper, espeak as a fallback), and runs as its own
# process so a stuck TTS can never block the web server.
#
# --optional marks a line as NICE TO HAVE. guide.say() then drops it rather than queueing it when
# the robot is mid-sentence. That is the right shape for a greeting (face_greet.py): saying hello
# to someone who walks past must never interrupt an answer, or the narration of a trip a visitor
# is being led on. An alert is not optional; a hello is.
import sys

from robot_ds_behavior.guide import say

if __name__ == "__main__":
    args = sys.argv[1:]
    optional = "--optional" in args
    text = " ".join(a for a in args if not a.startswith("--")).strip()
    if not text:
        sys.exit("nothing to say")
    say(text, optional=optional)
