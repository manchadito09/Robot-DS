#!/usr/bin/env python3
# voice_check.py - is the VOICE path alive, and is it fast? Exit 0 = yes.
#
# demo_check.sh checked that the speaker was not muted -- and stopped there. But a muted speaker is
# not the way the voice fails. It fails like this:
#
#   claude-daemon down  -> brain.py falls back to `claude -p`, which cold-starts per sentence:
#                          every answer goes from ~2 s to ~10 s. The robot still works. It is just
#                          unbearable to stand next to.
#   stt-daemon down     -> the web spawns a fresh Whisper per turn and pays 2.9 s to load the model
#                          again, for 0.25 s of actual transcription.
#
# Both of those are silent. Nothing crashes, nothing logs, demo_check said READY -- and the robot
# takes ten seconds to answer "hello" with a customer watching. So measure the thing that matters:
# ask each daemon a real question and time the reply.
#
#   python3 ~/ros2_ws/voice_check.py
#
#   exit 0  the voice path is up and quick
#   exit 1  something is down or crawling
import os
import socket
import struct
import subprocess
import sys
import time
import wave

CLAUDE_SOCK = "/tmp/robot_ds_claude.sock"
STT_SOCK = "/tmp/robot_ds_stt.sock"
PIPER = os.path.expanduser("~/Robot-DS/voice_prototype/piper/piper/piper")
MODEL = os.path.expanduser("~/Robot-DS/voice_prototype/piper/models/en_US-ryan-medium.onnx")

# Warm, on an idle Jetson: Whisper ~2.0 s, Claude ~2.8 s, Piper ~1.5 s. These ceilings are generous
# -- they are here to catch a daemon that is DOWN (and so paying a cold start every single turn),
# not to police a slow second. Note they are measured with the robot standing still: during a demo
# Nav2, the costmap and the camera are all competing for the same six cores, and everything here
# gets slower. That is the point of `uptime` being the first thing you look at.
LIMIT = {"whisper": 6.0, "claude": 8.0, "piper": 4.0}

GREEN, RED, YEL, NC = '\033[0;32m', '\033[0;31m', '\033[0;33m', '\033[0m'
bad = 0
took = {}          # stage -> seconds, for the one-line summary demo_check.sh prints


def say(kind, name, secs, detail="", stage=None):
    global bad
    if kind == "ok":
        if stage:
            took[stage] = secs
        print(f"  {GREEN}OK{NC}   {name}: {secs:.2f}s  {detail}")
    else:
        print(f"  {RED}FAIL{NC} {name}: {detail}")
        bad = 1


def _round_trip(sock, payload, timeout):
    """[4-byte len][utf8] out, [4-byte len][utf8] back -- the protocol both daemons speak."""
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(timeout)
    s.connect(sock)
    s.sendall(struct.pack(">I", len(payload)) + payload)
    raw = b""
    while len(raw) < 4:
        c = s.recv(4 - len(raw))
        if not c:
            raise IOError("daemon closed the connection")
        raw += c
    n = struct.unpack(">I", raw)[0]
    buf = b""
    while len(buf) < n:
        c = s.recv(n - len(buf))
        if not c:
            break
        buf += c
    s.close()
    return buf.decode("utf-8")


print("=== the voice path: mic -> Whisper -> Claude -> Piper ===")

# ---- Piper first: we need a WAV of speech to hand to Whisper, and making one tests Piper ------
wav = "/tmp/voice_check.wav"
env = dict(os.environ, LD_LIBRARY_PATH=os.path.dirname(PIPER))
PHRASE = "Take me to the kitchen, please."
try:
    t0 = time.time()
    subprocess.run([PIPER, "--model", MODEL, "--output_file", wav], input=PHRASE.encode(),
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env, timeout=30)
    dt = time.time() - t0
    secs = wave.open(wav).getnframes() / wave.open(wav).getframerate()
    if dt > LIMIT["piper"]:
        say("fail", "Piper (the robot's voice)", dt, f"SLOW: {dt:.1f}s to speak one line")
    else:
        say("ok", "Piper (the robot's voice)", dt, f"-> {secs:.1f}s of audio", stage="piper")
except Exception as e:
    say("fail", "Piper (the robot's voice)", 0, f"cannot synthesise: {e}")
    wav = None

# ---- Whisper: the warm STT daemon --------------------------------------------------------------
if wav and os.path.exists(wav):
    try:
        t0 = time.time()
        heard = _round_trip(STT_SOCK, wav.encode(), 60)
        dt = time.time() - t0
        if not heard.strip():
            say("fail", "Whisper (stt-daemon)", dt, "heard NOTHING from a clean recording")
        elif dt > LIMIT["whisper"]:
            say("fail", "Whisper (stt-daemon)", dt,
                f"SLOW ({dt:.1f}s) -- is stt-daemon up? A cold Whisper reloads per turn")
        else:
            say("ok", "Whisper (stt-daemon)", dt, f'heard "{heard.strip()}"', stage="whisper")
    except Exception as e:
        say("fail", "Whisper (stt-daemon)", 0,
            f"unreachable ({e}) -- sudo systemctl start stt-daemon")

# ---- Claude: the warm brain --------------------------------------------------------------------
try:
    t0 = time.time()
    reply = _round_trip(CLAUDE_SOCK, b"Reply with exactly one word: OK", 60)
    dt = time.time() - t0
    if not reply.strip():
        say("fail", "Claude (claude-daemon)", dt,
            "empty reply -- brain.py falls back to `claude -p`, ~10s per answer")
    elif dt > LIMIT["claude"]:
        say("fail", "Claude (claude-daemon)", dt, f"SLOW ({dt:.1f}s) -- is the daemon warm?")
    else:
        say("ok", "Claude (claude-daemon)", dt, f'said "{reply.strip()[:20]}"', stage="claude")
except Exception as e:
    say("fail", "Claude (claude-daemon)", 0,
        f"unreachable ({e}) -- sudo systemctl start claude-daemon")

print()
if bad:
    print(f"{RED}VOICE NOT READY{NC}")
else:
    # One line demo_check.sh can print verbatim. The total is what the visitor actually waits
    # through: they stop talking, and this long later the robot starts.
    total = sum(took.values())
    print(f"SUMMARY: hear {took.get('whisper', 0):.1f}s + think {took.get('claude', 0):.1f}s "
          f"+ speak {took.get('piper', 0):.1f}s = {total:.1f}s before the robot answers")
    print(f"{GREEN}VOICE READY{NC} - it hears, it thinks, it speaks.")
sys.exit(bad)
