#!/usr/bin/env python3
# stt_daemon.py - keep Whisper loaded, so speech-to-text costs 0.25 s instead of 4.
#
# Measured on the robot, per spoken message:
#
#   import faster_whisper ...  0.90 s
#   LOAD THE MODEL .........   2.91 s   <- paid again for EVERY message
#   transcribe the audio ...   0.25 s   <- the actual work
#                              -------
#                              4.06 s
#
# Transcribing is fast. Loading the model is not, and every message paid for it again, because the
# web app spawns a fresh listen_toggle.py per turn. So: load it ONCE, here, and hand it a WAV over a
# unix socket. Same trick as claude_daemon.py, same reason.
#
# Protocol (deliberately dumb):
#   client -> [4-byte big-endian length][utf-8 path to a .wav]
#   server -> [4-byte big-endian length][utf-8 transcript]   (empty = it failed; fall back)
#
# Run under the voice venv (faster-whisper lives there), as a systemd service:
#   ~/Robot-DS/voice_prototype/.venv/bin/python stt_daemon.py
import os
import socket
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stt import get_model, transcribe        # noqa: E402

SOCK = "/tmp/robot_ds_stt.sock"
MAXLEN = 4096


def _recv_exactly(conn, n):
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def handle(conn):
    try:
        conn.settimeout(30)
        raw = _recv_exactly(conn, 4)
        if not raw:
            return
        n = struct.unpack(">I", raw)[0]
        if n > MAXLEN:
            return
        path = (_recv_exactly(conn, n) or b"").decode("utf-8").strip()
        text = ""
        if path and os.path.exists(path) and os.path.getsize(path) >= 8000:
            t0 = time.time()
            try:
                text = (transcribe(path, language="en")[0] or "").strip()
                print(f"[stt_daemon] {time.time()-t0:.2f}s -> {text!r}", flush=True)
            except Exception as e:
                print(f"[stt_daemon] transcription failed: {e}", flush=True)
        else:
            # No audio is not a transcription failure -- say so, and return nothing. An empty
            # transcript is a problem you can see; the alternative (whatever was on disk last time)
            # is what made the robot keep answering the first thing it ever heard.
            print(f"[stt_daemon] no usable audio at {path!r}", flush=True)
        data = text.encode("utf-8")
        conn.sendall(struct.pack(">I", len(data)) + data)
    except Exception as e:
        print(f"[stt_daemon] client error: {e}", flush=True)
    finally:
        try:
            conn.close()
        except Exception:
            pass


def main():
    print("[stt_daemon] loading Whisper...", flush=True)
    t0 = time.time()
    get_model()                       # the whole point: pay this once, at boot, not per message
    print(f"[stt_daemon] model ready in {time.time()-t0:.1f}s", flush=True)

    if os.path.exists(SOCK):
        os.remove(SOCK)
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(SOCK)
    os.chmod(SOCK, 0o666)             # the web app runs as a different user under systemd
    srv.listen(4)
    print(f"[stt_daemon] listening on {SOCK}", flush=True)
    while True:
        conn, _ = srv.accept()
        threading.Thread(target=handle, args=(conn,), daemon=True).start()


if __name__ == "__main__":
    main()
