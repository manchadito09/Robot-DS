#!/usr/bin/env python3
# mic_vad_test.py - try the silence-stopping mic on its own, and print what it decided.
#
#   python3 ~/ros2_ws/mic_vad_test.py
#
# Speak a sentence when it says "listening", then stop. It should cut off ~0.8 s after you finish,
# not at a fixed time. Run it a few times, in the real room, to check the noise floor and threshold
# it picks are sane before we trust it in the greeter. NO ROS -- safe to run directly.
import sys
import time

sys.path.insert(0, "/home/ubuntu/ros2_ws")
import face_greet as fg

print("Say something after 'listening', then stop. Ctrl-C to quit.\n")
for i in range(3):
    print(f"--- try {i + 1}/3 ---")
    t0 = time.monotonic()
    text = fg.listen()
    print(f"    total {time.monotonic() - t0:.1f}s   heard: {text!r}\n")
    time.sleep(0.5)
print("Done. If it stopped ~0.8s after you finished, VAD works. If it cut you off, raise")
print("MIC_SILENCE_HANG_MS; if it never stopped, lower MIC_FLOOR_MULT or MIC_MIN_THRESH.")
