#!/usr/bin/env python3
# listen_toggle.py - record from the robot's mic until told to stop, then transcribe.
#
# Driven by the phone web app (server.py):
#   - 1st mic tap -> server starts this process, recording begins at once.
#   - 2nd mic tap -> server writes a newline to our stdin; that's our "stop".
# We then run STT (English) and print ONLY the recognized text on stdout.
#
# Recording uses `arecord` (direct ALSA), NOT sounddevice/PortAudio: PortAudio hangs
# when PULSE_SERVER is set (which it is under the systemd service), so the mic never
# recorded. arecord opens the hardware directly and is fast + reliable. The mic array
# is addressed by CARD NAME so it survives USB re-enumeration / card-number changes.
#
# Runs in the voice_prototype venv (faster-whisper lives there):
#   ~/Robot-DS/voice_prototype/.venv/bin/python listen_toggle.py <out.wav>
import os
import sys
import time
import signal
import threading
import subprocess
import socket
import struct

# Keep stdout clean: only the final transcript goes to the real stdout (the pipe the
# server reads); all library chatter is redirected to stderr.
_real_stdout = sys.stdout
sys.stdout = sys.stderr

# Imported lazily: with stt_daemon.py running we never touch Whisper in this process,
# and importing faster_whisper alone costs 0.9 s of the visitor's time.
def transcribe(*a, **k):
    from stt import transcribe as _t
    return _t(*a, **k)


def get_model():
    from stt import get_model as _g
    return _g()


STT_SOCK = "/tmp/robot_ds_stt.sock"

# The 6-mic far-field array (iflytek XFM-DP). By card NAME, never a number: unplug it and the USB
# audio dongle shifts from card 1 to card 0 and back again, and every hard-coded plughw:N,0 in the
# project starts pointing at the wrong device.
#
# When it was unplugged, arecord had nothing to open and failed EVERY time -- silently, because its
# stderr went to /dev/null. The previous turn's WAV was still on disk, so Whisper transcribed that
# again. And again. That is why the robot kept answering the first thing you ever said to it, no
# matter how many times you spoke to it afterwards. The fallbacks below (the dongle's own near-field
# mic, then ALSA's default) mean a missing array degrades the microphone instead of the robot.
MIC_DEV = os.environ.get("ROBOT_MIC_DEV", "plughw:CARD=XFMDPV0018,DEV=0")
MIC_FALLBACKS = ("plughw:CARD=Device,DEV=0", "default")
WAV = sys.argv[1] if len(sys.argv) > 1 else "audio.wav"


def _arecord(dev):
    # stderr to OUR stderr, not /dev/null. It used to be swallowed, so when arecord could not open
    # the mic it failed in complete silence -- and that silence is what caused the weirdest bug in
    # this project: the robot kept answering the FIRST thing you ever said to it. arecord died
    # without writing anything, the PREVIOUS recording was still sitting at WAV, and Whisper happily
    # transcribed it again. And again. The user hears their own first sentence come back forever.
    return subprocess.Popen(
        ["arecord", "-q", "-D", dev, "-f", "S16_LE", "-r", "16000", "-c", "1", WAV])


# Delete last turn's recording BEFORE we start. If this recording fails, there must be no audio left
# to transcribe -- an empty transcript is a problem you can see; a stale one pretends to work.
try:
    os.remove(WAV)
except OSError:
    pass

# No need to warm Whisper here any more: stt_daemon.py holds it loaded. If the daemon is
# down we warm it in the background anyway, so the fallback is not slower than before.
if not os.path.exists(STT_SOCK):
    threading.Thread(target=get_model, daemon=True).start()

# The array first, twice, then the fallbacks. The retry on the same device is not paranoia: the
# previous turn's arecord can still be letting go of it when the next tap arrives, and a "device
# busy" arecord dies instantly.
rec = None
for dev, wait in [(MIC_DEV, 0.3), (MIC_DEV, 0.6)] + [(d, 0.3) for d in MIC_FALLBACKS]:
    rec = _arecord(dev)
    time.sleep(wait)
    if rec.poll() is None:
        break                          # still alive = it got the mic
    print(f"[listen] arecord could not open {dev} -- trying the next device", file=sys.stderr)

# Stop when the server sends a line on stdin (2nd tap) or closes it (EOF).
try:
    sys.stdin.readline()
except Exception:
    pass

# Timed, because "speech-to-text took 5 s" turned out to be almost none of it Whisper.
_t = time.time()
rec.send_signal(signal.SIGINT)          # arecord writes a valid WAV header on SIGINT
try:
    rec.wait(timeout=5)
except Exception:
    rec.kill()
print(f"[listen] arecord stopped in {time.time()-_t:.2f}s", file=sys.stderr)
_t = time.time()

text = ""
# Only transcribe audio we actually recorded THIS turn. A missing or tiny file means arecord never
# got the mic -- say so, and return nothing, rather than handing Whisper whatever happens to be on
# disk. 8 KB is a quarter of a second at 16 kHz mono: below that there is no sentence in there.
size = os.path.getsize(WAV) if os.path.exists(WAV) else 0
if size < 8000:
    print(f"[listen] no audio recorded ({size} bytes) -- did arecord get the mic?", file=sys.stderr)
else:
    # The warm daemon first. Loading Whisper costs 2.9 s and transcribing costs 0.25 s, and this
    # process is spawned fresh for every message the visitor speaks -- so we were paying the 2.9 s
    # again, every single time, for a quarter of a second of work. stt_daemon.py keeps the model
    # loaded. If it is not running we fall back to loading it here: slow, but it always answers.
    try:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(30)
        s.connect(STT_SOCK)
        d = os.path.abspath(WAV).encode("utf-8")
        s.sendall(struct.pack(">I", len(d)) + d)
        n = struct.unpack(">I", s.recv(4))[0]
        buf = b""
        while len(buf) < n:
            chunk = s.recv(n - len(buf))
            if not chunk:
                break
            buf += chunk
        s.close()
        text = buf.decode("utf-8").strip()
        print(f"[listen] stt_daemon answered in {time.time()-_t:.2f}s", file=sys.stderr)
    except Exception as e:
        print(f"[listen] stt_daemon unreachable ({e}) -- loading Whisper in-process", file=sys.stderr)
        try:
            text = (transcribe(WAV, language="en")[0] or "").strip()
        except Exception as e2:
            print(f"[listen] transcription failed: {e2}", file=sys.stderr)
_real_stdout.write(text + "\n")
_real_stdout.flush()
