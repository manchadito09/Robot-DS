#!/usr/bin/env python3
# claude_daemon.py - keep a POOL of warm `claude` processes so the robot's Claude calls
# take ~2s instead of the ~10s cold-start of a fresh `claude -p` every time -- WITHOUT
# changing the answers.
#
# WHY A POOL (and not one reused process): `claude -p --input-format stream-json` stays
# alive between messages, but the only way to wipe its memory between prompts is `/clear`
# -- and measured, `/clear` DEGRADES the model's reasoning (it starts answering "none" to
# indirect requests like "i'm hungry" that a fresh process answers correctly). So instead:
#   * we PRE-WARM each process with one throwaway prompt ("say ready") -- that pays the ~10s
#     Node/harness boot but does NOT degrade anything,
#   * the robot's real prompt is that process's SECOND turn (no /clear) -> full-quality,
#     fast answer, identical to a cold `claude -p`,
#   * then we THROW THE PROCESS AWAY (one prompt per process) so nothing leaks between the
#     robot's sentences -> each answer stays INDEPENDENT, exactly like before.
# A background refiller keeps POOL_SIZE warm processes ready, so the ~10s boot happens off
# the critical path. Same model (Opus), same Max subscription, same behaviour, ~6x faster.
#
# brain.py connects to SOCK, sends a prompt, gets the reply. If this daemon is down or the
# pool is momentarily empty, brain.py falls back to plain `claude -p` -- the robot never breaks.
#
#   run by systemd (claude-daemon.service); manual: python3 ~/ros2_ws/claude_daemon.py
import os
import json
import queue
import socket
import struct
import subprocess
import threading
import time
import shutil

SOCK = "/tmp/robot_ds_claude.sock"
# 3, not 2. Measured over three back-to-back questions: 1.85 s, 1.31 s, then 6.22 s -- the third
# one drained the pool and had to wait for a cold `claude` to boot. A demo conversation is exactly
# that: several questions in a row, and one 6-second silence in front of a customer is the one
# everybody remembers. Costs ~260 MB of RAM, which the Jetson has as long as Claude Code is not also
# running on it (it takes ~520 MB on its own -- close it before the demo).
POOL_SIZE = 3               # warm processes kept ready
WARMUP = "Reply with the single word: ready"
QUERY_TIMEOUT = 45.0        # a single real answer should never take longer than this
GET_TIMEOUT = 8.0           # how long a request waits for a warm process before falling back
CLAUDE = shutil.which("claude") or "/home/ubuntu/.local/bin/claude"


def _log(msg):
    print("[claude_daemon] %s" % msg, flush=True)


def _spawn_warm():
    """Start a claude stream-json process and pay its boot with a throwaway prompt.
    Returns the live process (ready for ONE real prompt), or None on failure."""
    try:
        p = subprocess.Popen(
            [CLAUDE, "-p", "--input-format", "stream-json",
             "--output-format", "stream-json", "--verbose"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1)
        _send(p, WARMUP)
        _wait_result(p, 40)          # blocks ~10s (the boot) — done off the request path
        return p
    except Exception as e:
        _log("warmup failed: %s" % e)
        return None


def _send(p, text):
    p.stdin.write(json.dumps({"type": "user", "message": {"role": "user", "content": text}}) + "\n")
    p.stdin.flush()


def _wait_result(p, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        line = p.stdout.readline()
        if not line:
            raise IOError("claude stdout closed")
        line = line.strip()
        if not line:
            continue
        try:
            ev = json.loads(line)
        except ValueError:
            continue
        if ev.get("type") == "result":
            return str(ev.get("result", "")).strip()
    raise TimeoutError("no result within %.0fs" % timeout)


class Pool:
    def __init__(self):
        self.ready = queue.Queue()
        threading.Thread(target=self._refill_loop, daemon=True).start()

    def _refill_loop(self):
        while True:
            if self.ready.qsize() < POOL_SIZE:
                p = _spawn_warm()
                if p is not None:
                    self.ready.put(p)
                    _log("warm process ready (pool=%d)" % self.ready.qsize())
                else:
                    time.sleep(2)
            else:
                time.sleep(0.2)

    def ask(self, prompt):
        """Take a warm process, run ONE real prompt on it, then discard it. Raises if no
        warm process is available in time (caller falls back to `claude -p`)."""
        p = self.ready.get(timeout=GET_TIMEOUT)   # raises queue.Empty -> caller falls back
        try:
            _send(p, prompt)                       # real query = 2nd turn, no /clear
            return _wait_result(p, QUERY_TIMEOUT)
        finally:
            try:
                p.kill()                           # one prompt per process -> no leakage
            except Exception:
                pass


def handle(conn, pool):
    """Protocol: client sends [4-byte len][utf8 prompt]; we reply [4-byte len][utf8]."""
    try:
        conn.settimeout(QUERY_TIMEOUT + 10)
        raw = _recvn(conn, 4)
        if not raw:
            return
        n = struct.unpack(">I", raw)[0]
        prompt = _recvn(conn, n).decode("utf-8")
        try:
            reply = pool.ask(prompt)
        except Exception as e:
            _log("ask failed (%s) -> client falls back" % e)
            reply = ""                              # empty => brain.py uses plain claude -p
        data = reply.encode("utf-8")
        conn.sendall(struct.pack(">I", len(data)) + data)
    except Exception as e:
        _log("conn error: %s" % e)
    finally:
        try:
            conn.close()
        except Exception:
            pass


def _recvn(conn, n):
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return buf
        buf += chunk
    return buf


def main():
    if os.path.exists(SOCK):
        os.remove(SOCK)
    pool = Pool()
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(SOCK)
    os.chmod(SOCK, 0o666)
    srv.listen(8)
    _log("listening on %s (warming a pool of %d)" % (SOCK, POOL_SIZE))
    while True:
        conn, _ = srv.accept()
        threading.Thread(target=handle, args=(conn, pool), daemon=True).start()


if __name__ == "__main__":
    main()
