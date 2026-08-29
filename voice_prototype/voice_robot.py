#!/usr/bin/env python3
# voice_robot.py - TALK TO THE ROBOT BY VOICE, 100% ON THE ROBOT (no laptop/SSH).
#
#   mic (6-mic array) -> STT (faster-whisper, English) -> ~/voice_nav.sh
#   -> brain.py --stay (Claude picks a saved waypoint, Nav2 drives)
#   -> the robot narrates OUT LOUD through its speaker (guide.py -> espeak).
#
# Needs Nav2 launched and the robot localized (2D Pose Estimate in RViz).
# Run (from this folder):
#   .venv/bin/python voice_robot.py
import os
import subprocess

# Silence onnxruntime's one-off "GPU device discovery failed" line (prints at
# import time only; we never touch stderr during recording/transcription).
_saved = os.dup(2)
_null = os.open(os.devnull, os.O_WRONLY)
os.dup2(_null, 2)
try:
    import onnxruntime
    onnxruntime.set_default_logger_severity(3)
except Exception:
    pass
finally:
    os.dup2(_saved, 2)
    os.close(_saved)
    os.close(_null)

import socket
import struct

from record import record_until_enter

VOICE_NAV = os.path.expanduser("~/voice_nav.sh")
STT_LANG = "en"          # understand English only
STT_SOCK = "/tmp/robot_ds_stt.sock"


def find_mic():
    """The 6-mic far-field array, BY NAME. It used to be MIC_DEVICE = 0, hard-coded, and 0 was the
    array -- until the array was unplugged and plugged back in, at which point 0 became the cheap USB
    dongle and the robot was quietly listening through the wrong microphone. Indices depend on what
    is plugged in and in what order; names do not. Falls back to whatever can record, so a missing
    array costs us the good mic, not the demo."""
    import sounddevice as sd
    devs = sd.query_devices()
    for i, d in enumerate(devs):
        if d["max_input_channels"] > 0 and "XFM" in d["name"]:
            return i, d["name"]
    for i, d in enumerate(devs):
        if d["max_input_channels"] > 0:
            print(f"  (no 6-mic array found -- falling back to '{d['name']}')")
            return i, d["name"]
    return None, "?"


def stt(path):
    """Transcribe through the warm Whisper daemon (0.15 s), or load the model here if it is down
    (4 s). Keeping the model out of this process also keeps ~400 MB out of the Jetson's RAM."""
    try:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(30)
        s.connect(STT_SOCK)
        d = os.path.abspath(path).encode("utf-8")
        s.sendall(struct.pack(">I", len(d)) + d)
        n = struct.unpack(">I", s.recv(4))[0]
        buf = b""
        while len(buf) < n:
            chunk = s.recv(n - len(buf))
            if not chunk:
                break
            buf += chunk
        s.close()
        return buf.decode("utf-8").strip()
    except Exception as e:
        print(f"  (stt_daemon unreachable: {e} -- using the in-process model)")
        from stt import transcribe
        return (transcribe(path, language=STT_LANG)[0] or "").strip()


def drive(text):
    """Send the text to the brain; the robot picks a goal, drives and speaks."""
    proc = subprocess.Popen(["bash", VOICE_NAV, text],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    for line in proc.stdout:
        print("  " + line.rstrip())
    proc.wait()


def main():
    mic, mic_name = find_mic()
    if mic is None:
        print("No microphone at all. Is the mic array plugged in?")
        return
    print(f"Mic: [{mic}] {mic_name}")
    if not os.path.exists(STT_SOCK):
        print("  (stt_daemon is not running -- transcription will be ~4 s instead of ~0.2 s:")
        print("   sudo systemctl start stt-daemon)")
    print("Ready. Press ENTER to talk (Ctrl+C to quit)\n")
    while True:
        try:
            record_until_enter("audio.wav", device=mic)
        except (EOFError, KeyboardInterrupt):
            print("\nBye!")
            break
        text = stt("audio.wav")
        print("You said:", repr(text))
        if not text:
            print("  (heard nothing -- is the mic muted, or too far away?)")
            continue
        drive(text)


if __name__ == "__main__":
    main()
