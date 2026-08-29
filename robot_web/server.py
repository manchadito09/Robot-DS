#!/usr/bin/env python3
# server.py - Robot-DS web app (control Wall-E from a phone over office WiFi).
#
# Home menu -> pick a mode:
#   Take me somewhere : tap a saved point -> Nav2 leads you there (+ add/remove points)
#   Talk to me        : say (or type) what you need -> Claude picks the place, robot goes
#   Drive me          : manual teleop buttons (hold to move) -> /cmd_vel
#   Messenger         : pick a point + a phrase -> robot goes there and says it
#   Tour              : pick several points -> robot visits each, narrating
#   Come here         : pick a point -> robot drives to it
#
# The page itself is index.html (next to this file); the robot serves it and you
# can also open index.html by double-click for a design preview.
#
# Runs ON the robot in the ROS 2 environment (source it first). It reuses the
# existing package instead of reinventing anything:
#   - navigation/talk/messenger/tour/come : `ros2 run robot_ds_behavior guide ...`
#   - add/remove points                   : `python3 ~/ros2_ws/waypoint_tool.py save|del`
#   - manual drive                        : publishes geometry_msgs/Twist to /cmd_vel
#
# Run (on the robot, ROS sourced):   python3 ~/Robot-DS/robot_web/server.py
import os
import sys
import json
import math
import time
import shutil
import socket
import threading
import subprocess
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import rclpy
    from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
    HAS_ROS = True
except Exception:
    HAS_ROS = False   # laptop preview: no ROS installed -> just show the UI

try:  # separate import so a missing servo pkg can't disable the rest of the app
    from servo_controller_msgs.msg import ServosPosition, ServoPosition
    HAS_ARM = True
except Exception:
    HAS_ARM = False   # servo msgs not built -> arm free-move disabled

try:  # "Add me": the forward camera + the two face models. Optional, like everything else here --
      # no C270 plugged in, or no models downloaded, and the app simply does not offer the section.
      # face_lib lives in ~/ros2_ws (standalone, no colcon build) and imports NO ROS, so a wedged
      # ROS cannot take the web app down with it.
    sys.path.insert(0, os.path.expanduser("~/ros2_ws"))
    import face_lib
    import cv2                    # face_lib caps its thread count BEFORE this import -- see there
    import base64
    import numpy as np
    HAS_FACES = os.path.exists(face_lib.YUNET) and os.path.exists(face_lib.SFACE)
    if not HAS_FACES:
        print("[faces] models missing from ~/ros2_ws/models -> 'Add me' is off", flush=True)
except Exception as e:
    # SAY WHY. A bare 'except -> feature off' is how this arrived broken the first time: the real
    # error was a missing `import sys`, and the only symptom was a button that never appeared.
    # An optional feature may switch itself off; it may not do it silently.
    HAS_FACES = False
    print(f"[faces] disabled: {e.__class__.__name__}: {e}", flush=True)

try:  # health-panel sensors (lidar + battery) -- optional
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import UInt16
    HAS_HEALTH = True
except Exception:
    HAS_HEALTH = False

try:
    import cv2
    HAS_CV2 = True
except Exception:
    HAS_CV2 = False   # no OpenCV -> Camera tab shows "not available"

# ---- Config (override with env vars) ----------------------------------------
PORT = int(os.environ.get("ROBOT_WEB_PORT", "8000"))
CMD_VEL_TOPIC = os.environ.get("ROBOT_CMD_VEL", "/cmd_vel")
LIN_SPEED = float(os.environ.get("ROBOT_LIN", "0.30"))   # m/s   forward/back
ANG_SPEED = float(os.environ.get("ROBOT_ANG", "1.0"))    # rad/s turning
DRIVE_TTL = 0.6   # s: keep moving this long after the last button ping (deadman)
# The Orbbec DaBai DCW has no /dev/video* node -- it's driven by the Orbbec ROS
# driver, which republishes color as /depth_cam/rgb/image_raw. We subscribe there.
CAM_TOPIC = os.environ.get("ROBOT_CAM_TOPIC", "/depth_cam/rgb/image_raw")

# ---- Camera watchdog --------------------------------------------------------
# The lidar sweeps at 16.8 cm and passes clean OVER a chair leg. The depth camera is what sees
# those, via camera_scan_node -> /camera_scan -> the costmap. When that chain dies the robot does
# not slow down or complain: it drives on, blind, and clips the chair. It has happened twice, and
# the second time we spent a morning blaming the costmap.
#
# So: watch /camera_scan, the END of the chain. It only publishes when the driver AND
# camera_scan_node are both alive, which makes one topic proof of the whole path. If it goes quiet
# mid-trip, stop the robot and say so out loud. Better parked and honest than blind in front of a
# customer.
CAM_SCAN_TOPIC = os.environ.get("ROBOT_CAM_SCAN_TOPIC", "/camera_scan")
CAM_WATCHDOG = os.environ.get("ROBOT_CAM_WATCHDOG", "1") != "0"   # set 0 to drive without a camera
CAM_STALE = 3.0      # s of silence on /camera_scan = the camera path is down
CAM_GRACE = 8.0      # s at the start of a trip before we judge it (the driver takes ~5 s to open)
CAM_COOLDOWN = 60.0  # s between repair attempts, so a truly dead camera can't spawn a storm
# s the lens may stay covered mid-trip before we give up and ask them to move. Short enough that we
# never plough on blind, long enough that someone crossing in front of the robot is not an incident.
CAM_BLOCKED_GRACE = 15.0
SPEAK_TOOL = os.path.expanduser("~/ros2_ws/speak.py")

# THE SPEAKER SWITCH, and there is exactly one of it.
#
# The phone can now silence the robot -- a visitor may not want to be talked at, and the office it is
# tested in is full of people working. But a muted robot gives NO sign of being muted: it still
# "speaks", with text in this chat, text in the logs, the same pauses for lines nobody hears. It just
# makes no sound. Muted on a Tuesday and forgotten, it narrates an entire demo in silence, on camera,
# and the only warning is an investor looking politely puzzled.
#
# So the button does NOT reach for amixer itself. It runs speaker.sh, which is also what the
# terminal uses, and which leaves a flag file behind. That flag is what makes a mute impossible to
# forget: "Robot Web QR" refuses to undo it, and demo_check.sh FAILS on it, in red, even when the
# volume happens to be up. One switch, one flag, one way out.
SPEAKER_TOOL = os.path.expanduser("~/ros2_ws/speaker.sh")
SPEAKER_MUTED_FLAG = os.path.expanduser("~/.cache/robot_ds/speaker_muted")

# When this flag exists, the robot announces arrivals but skips the description -- for visitors who
# already know the floor (Direct Supply's own people). guide.py reads it on arrival; this just
# creates or removes the file. Same cache dir as everything else the behaviour side leaves notes in.
EXPLAIN_OFF_FLAG = os.path.expanduser("~/.cache/robot_ds/explanations_off")

# SAVED TOURS. A named tour is just an ordered list of stop names -- {"Default": ["Developers",
# "Kitchen", ...]}. Kept as one small JSON file so they survive restarts, next to everything else the
# robot remembers. The web loads one to pre-tick the stops (it does not auto-start -- the human hits
# Start), and saves the currently-ticked stops under a name.
TOURS_FILE = os.path.expanduser("~/.cache/robot_ds/tours.json")


def load_tours():
    """All saved tours as {name: [stops]}. Seeds a 'Default' from the office's demo_places the very
    first time, so the section is never empty and the obvious tour is one tap away."""
    import json
    try:
        with open(TOURS_FILE) as f:
            tours = json.load(f)
        if isinstance(tours, dict):
            return tours
    except (FileNotFoundError, ValueError, OSError):
        pass
    # first run: build a Default from demo_places, filtered to stops that really exist on the map
    seed = {}
    try:
        import yaml
        kb = yaml.safe_load(open(os.path.expanduser("~/ros2_ws/office_knowledge.yaml"))) or {}
        demo = (kb.get("office") or {}).get("demo_places") or []
        places = set(_place_names())
        stops = [n for n in demo if n in places]
        if stops:
            seed = {"Default": stops}
            save_tours(seed)
    except Exception:
        pass
    return seed


def save_tours(tours):
    import json
    os.makedirs(os.path.dirname(TOURS_FILE), exist_ok=True)
    tmp = TOURS_FILE + ".part"
    with open(tmp, "w") as f:
        json.dump(tours, f, indent=2)
    os.replace(tmp, TOURS_FILE)          # atomic: never leave a half-written file


def _place_names():
    """The waypoint names the robot can actually drive to. Used to keep saved tours honest."""
    try:
        import yaml
        wp = yaml.safe_load(open(os.path.expanduser(
            "~/ros2_ws/src/slam/maps/map_01.waypoints.yaml"))) or {}
        return list((wp.get("waypoints") or {}).keys())
    except Exception:
        return []


def explanations_on():
    return not os.path.exists(EXPLAIN_OFF_FLAG)


def set_explanations(on):
    try:
        if on:
            if os.path.exists(EXPLAIN_OFF_FLAG):
                os.remove(EXPLAIN_OFF_FLAG)
        else:
            os.makedirs(os.path.dirname(EXPLAIN_OFF_FLAG), exist_ok=True)
            open(EXPLAIN_OFF_FLAG, "w").close()
        return True, ("I'll tell you about each place we reach."
                      if on else "I'll just announce arrivals, no descriptions.")
    except Exception as e:
        return False, f"Could not change that ({e})."


def speaker_on():
    """Is the robot allowed to make a sound? (The flag is the truth -- see SPEAKER_TOOL.)"""
    return not os.path.exists(SPEAKER_MUTED_FLAG)


def speaker_volume():
    """The current speaker level as 0-100, or None if it can't be read. The mixer's own range is
    0-30 (Limits: Playback 0 - 30), and amixer reports the percentage, so this is already 0-100."""
    try:
        r = subprocess.run(["amixer", "-c", "Device", "sget", "Speaker"],
                           stdout=subprocess.PIPE, text=True, timeout=5)
        import re
        m = re.search(r"\[(\d+)%\]", r.stdout)
        if not m:
            return None
        # muted reads as some %, so fold the flag in: silenced is 0 to the UI.
        return 0 if not speaker_on() else int(m.group(1))
    except Exception:
        return None


def set_speaker(on):
    try:
        r = subprocess.run(["bash", SPEAKER_TOOL, "unmute" if on else "mute"],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=10)
        if r.returncode != 0:
            return False, "Could not reach the speaker. Is the USB audio device plugged in?"
    except Exception as e:
        return False, f"Could not set the speaker ({e})."
    return True, ("Speaker on. I'll talk you through it."
                  if on else "Speaker off. I'll still say everything -- read it here.")


def set_volume(pct):
    """Set the speaker to an exact level, 0-100. 0 mutes (and leaves the flag, so the QR button
    respects it). Anything above 0 unmutes. One tool does both -- see speaker.sh."""
    try:
        pct = max(0, min(100, int(pct)))
    except (TypeError, ValueError):
        return False, "Volume must be a number 0-100."
    try:
        r = subprocess.run(["bash", SPEAKER_TOOL, "vol", str(pct)],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=10)
        if r.returncode != 0:
            return False, "Could not reach the speaker. Is the USB audio device plugged in?"
    except Exception as e:
        return False, f"Could not set the volume ({e})."
    return True, (f"Volume {pct}%." if pct else "Muted -- I'll still say everything, read it here.")
# camera_check.sh IS the repair: it relaunches the driver if the depth stream never opened, and
# starts camera_scan_node if /camera_scan is silent. The watchdog just calls it, so there is one
# recovery path and not two that can disagree.
CAM_CHECK = os.path.expanduser("~/ros2_ws/camera_check.sh")

HERE = os.path.dirname(os.path.abspath(__file__))
INDEX_HTML = os.path.join(HERE, "index.html")
WAYPOINTS_YAML = os.path.expanduser("~/ros2_ws/src/slam/maps/map_01.waypoints.yaml")
# Knowledge base the voice brain reads (what each place is). Adding a point from the
# web writes a stub here too, so the robot instantly KNOWS the new place, not just
# how to drive to it. brain.py reads this file live (no rebuild needed).
KNOWLEDGE_YAML = os.path.expanduser("~/ros2_ws/office_knowledge.yaml")
WAYPOINT_TOOL = os.path.expanduser("~/ros2_ws/waypoint_tool.py")
MAP_DIR = os.path.dirname(WAYPOINTS_YAML)
MAP_YAML = os.path.join(MAP_DIR, "map_01.yaml")
MAP_PGM = os.path.join(MAP_DIR, "map_01.pgm")
GUIDE = ["ros2", "run", "robot_ds_behavior", "guide"]
BRAIN = ["ros2", "run", "robot_ds_behavior", "brain"]
# Arm GESTURES only (rest / point / raise) -- expressive movement for the guide demo,
# NOT manipulation/pick-up (that stays Phase 2 per CLAUDE.md). arm_gesture.py plays
# the JetRover's own collision-safe action groups.
ARM_TOOL = os.path.expanduser("~/ros2_ws/arm_gesture.py")

# Voice: a phone browser can't use its mic over plain HTTP, so "Talk to me" records
# on the robot's OWN mic array instead. listen_toggle.py (in the voice venv) records
# until we tell it to stop, then prints the transcript. See /api/listen/{start,stop}.
VOICE_DIR = os.path.expanduser("~/Robot-DS/voice_prototype")
VOICE_PY = os.path.join(VOICE_DIR, ".venv/bin/python")
LISTEN_SCRIPT = os.path.join(VOICE_DIR, "listen_toggle.py")
LISTEN_WAV = os.path.join(VOICE_DIR, "web_listen.wav")

# ---- Shared state -----------------------------------------------------------
_lock = threading.Lock()
_drive = {"lin": 0.0, "ang": 0.0, "until": 0.0}     # current teleop command
_task = {"proc": None, "what": "", "t0": 0.0}       # running nav/talk subprocess (t0 = started at)
_listen = {"proc": None}                            # running mic-record subprocess
_ros = {"arm_pub": None, "goal_pub": None, "initialpose_pub": None, "node": None}   # live pubs for arm + nav goals + localize
_pose = {"x": None, "y": None, "yaw": None}         # latest robot pose (from /amcl_pose)
_health = {"scan_t": 0.0, "batt": None, "batt_t": 0.0, "pose_t": 0.0,
           "cam_t": 0.0,          # last /camera_scan message: the whole depth path in one number
           "cam_fail": ""}        # why the watchdog stopped the last trip ("" = it didn't)
_repair = {"running": False, "last": 0.0, "tries": 0}   # camera self-repair (see _repair_camera)
_status = {"line": ""}                              # latest robot narration (live "doing")
# Recent spoken lines, so the phone's "Talk to me" chat can show what the robot
# said (its answers + narration). Each entry {"id": n, "text": ...}; id grows.
_say = {"seq": 0, "log": []}
_control = {"holder": None, "expires": 0.0}         # single-controller lock (one pilot at a time)
CONTROL_TTL = 20.0                                  # seconds of inactivity before control frees
# POST paths that actually command the robot -> gated by the controller lock.
# NOT gated: /api/stop (anyone can STOP), /api/control, and all read-only GETs.
CONTROL_PATHS = {
    "/api/drive", "/api/go", "/api/messenger", "/api/tour",
    "/api/talk", "/api/arm", "/api/arm/joint", "/api/arm/center",
    "/api/goto", "/api/set_pose", "/api/add_point", "/api/remove_point", "/api/add_point_at",
    "/api/save_knowledge",
    "/api/listen/start", "/api/listen/stop",
    # Enrolling a face does not move the robot, but it does TAKE THE FORWARD CAMERA for ~5 s and
    # write to the shared face store. Two phones enrolling at once would interleave shots into
    # each other's fingerprints. The controller lock is already the answer to "one person drives".
    "/api/face/enroll", "/api/face/enroll_photos", "/api/face/remove", "/api/face/greeting",
    "/api/face/forget_learned", "/api/face/cooldown",
}


def claim_control(cid):
    """Grant control to cid if it's free, expired, or already theirs. Refreshes TTL."""
    cid = cid or "?"
    now = time.monotonic()
    with _lock:
        held = _control["holder"] is not None and now <= _control["expires"]
        if (not held) or _control["holder"] == cid:
            _control["holder"] = cid
            _control["expires"] = now + CONTROL_TTL
            return True
        return False


def control_status(cid, action="status"):
    """status = report (and heartbeat if you hold it); take = grab if free; release = drop."""
    cid = cid or "?"
    now = time.monotonic()
    with _lock:
        held = _control["holder"] is not None and now <= _control["expires"]
        you = held and _control["holder"] == cid
        if action == "release" and you:
            _control["holder"] = None
            held = you = False
        elif action == "take":                 # explicit takeover always wins (no lockout)
            _control["holder"] = cid
            _control["expires"] = now + CONTROL_TTL
            held = you = True
        # NOTE: 'status' does NOT refresh the hold. Control only stays while you're
        # actively commanding (claim_control refreshes on each real command), so an
        # idle open tab can't keep the lock hostage and block another device.
        return {"ok": True, "you": you, "held": held, "free": not held}
_preview_places = ["reception", "kitchen", "meeting", "desks", "exit"]


def _set_drive(lin, ang):
    with _lock:
        _drive.update(lin=lin, ang=ang, until=time.monotonic() + DRIVE_TTL)


def _stop_drive():
    with _lock:
        _drive.update(lin=0.0, ang=0.0, until=0.0)


def _busy():
    p = _task["proc"]
    return p is not None and p.poll() is None


# THE ROBOT CAN BE ASKED THINGS WHILE IT WALKS.
#
# It could not, and it was inviting people to try: mid-trip it says "ask me anything, I know the
# company, not only the corridors" -- and /api/talk went through _run_task, which refuses while the
# robot is busy. The visitor asks, and the robot says "Wall-E is busy right now. One thing at a
# time!" That is worse than never inviting them.
#
# So an ANSWER gets its own slot, and runs alongside the trip. What makes that safe is that the two
# things a brain can do are not equally dangerous:
#
#   talking  two voices at once is impossible anyway -- guide and brain share one flock on the audio
#            device, and the trip's chatter is droppable while a real answer waits its turn. The
#            visitor's question beats the robot's small talk, which is the right way round.
#   driving  two Nav2 goals at once means the robot ABANDONS the person it is leading, mid-corridor,
#            to go somewhere else.
#
# So the second brain runs with --no-drive: it may answer, it may not move. And only one at a time,
# or an impatient visitor asking three questions spawns three Claudes on a Jetson with six cores.
_chat = {"proc": None}                    # a brain answering a question DURING a trip

# The tasks that MOVE the robot. One definition, because two places now need it -- the camera
# watchdog (which stops a trip if the camera dies) and /api/talk (which will answer during one).
DRIVING_TASKS = {"go", "tour", "messenger"}


def _chatting():
    p = _chat["proc"]
    return p is not None and p.poll() is None


def _brain_can_hold_still():
    """Does the INSTALLED brain.py understand --no-drive? Checked once, at import.

    This guard is the whole safety of answering-while-driving, and it exists because of how
    brain.py parses its arguments:

        words = [a for a in sys.argv[1:] if not a.startswith("--")]

    It silently DISCARDS any flag it does not know. So handing --no-drive to a brain built before
    the flag existed does not fail, and does not warn: it drives. A second Nav2 goal, mid-trip, and
    the robot walks away from the visitor it was leading.

    brain.py runs from the colcon BUILD. One forgotten `colcon build` is all it takes -- and that
    is not a hypothetical, it is the single most-repeated mistake on this robot. So we do not
    assume the flag is there. We look. If it is missing we simply do not offer the feature, and say
    so in the journal, loudly.
    """
    try:
        r = subprocess.run(["ros2", "pkg", "prefix", "robot_ds_behavior"],
                           stdout=subprocess.PIPE, text=True, timeout=10)
        prefix = r.stdout.strip()
        if not prefix:
            return False
        for root, _dirs, files in os.walk(os.path.join(prefix, "lib")):
            if "brain.py" in files:
                with open(os.path.join(root, "brain.py")) as f:
                    return "--no-drive" in f.read()
        # ros2 run resolves an entry-point script; the module lives under site-packages.
        for root, _dirs, files in os.walk(prefix):
            if "brain.py" in files and "site-packages" in root:
                with open(os.path.join(root, "brain.py")) as f:
                    return "--no-drive" in f.read()
    except Exception:
        pass
    return False


CAN_CHAT_WHILE_DRIVING = _brain_can_hold_still() if HAS_ROS else False
if HAS_ROS and not CAN_CHAT_WHILE_DRIVING:
    print("[talk] the installed brain has no --no-drive: questions during a trip will be refused, "
          "not answered. Fix with: colcon build --packages-select robot_ds_behavior", flush=True)


def _run_task(what, argv):
    """Start a driving subprocess if none is running. Returns (ok, message)."""
    if not HAS_ROS:
        return True, "Preview: Wall-E would do that for real on the robot."
    if _busy():
        return False, "Wall-E is busy right now. One thing at a time!"
    _stop_drive()                         # manual driving must not fight Nav2
    # Capture the task's stdout so the web can show live what the robot is doing
    # (guide/brain print "[robot] <line>" for every thing they say).
    # ROBOT_TIMING makes brain.py stamp each stage -- imports, the Claude call and which path served
    # it, the moment it starts speaking -- and it all lands in journalctl -u robot-web. Cheap, and it
    # means the next "why is it so slow" is answered by reading, not guessing.
    env = dict(os.environ, ROBOT_TIMING="1")
    proc = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1, env=env)
    _task["proc"] = proc
    _task["what"] = what
    _task["t0"] = time.monotonic()        # the camera watchdog waits CAM_GRACE from here
    _health["cam_fail"] = ""              # a new trip clears the last watchdog verdict
    _status["line"] = ""
    threading.Thread(target=_read_task, args=(proc,), daemon=True).start()
    return True, "On it!"


def _answer_while_driving(text):
    """Answer a question WITHOUT interrupting the trip. See _chat above."""
    if _chatting():
        return False, "One question at a time -- I'm still thinking about the last one."
    env = dict(os.environ, ROBOT_TIMING="1")
    proc = subprocess.Popen(BRAIN + ["--stay", "--no-drive", text],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1, env=env)
    _chat["proc"] = proc
    # Same reader as a task: what it says still reaches the phone's chat and the journal.
    threading.Thread(target=_read_task, args=(proc,), daemon=True).start()
    return True, "On it!"


def _read_task(proc):
    """Feed the live status from the task's narration, and keep it in the journal."""
    try:
        for line in proc.stdout:
            line = line.strip()
            if not line:
                continue
            print(line, flush=True)              # still visible in journalctl -u robot-web
            if line.startswith("[robot]"):
                txt = line[7:].strip()
                _status["line"] = txt
                with _lock:                      # log it for the phone chat view
                    _say["seq"] += 1
                    _say["log"].append({"id": _say["seq"], "text": txt})
                    if len(_say["log"]) > 50:
                        del _say["log"][:-50]
    except Exception:
        pass


def stop_all():
    """Emergency stop: kill any running nav/talk task, cancel the Nav2 goal and
    zero teleop. Safe to spam and safe to press when nothing is moving.

    BOTH processes, and that is not a detail. The robot can now be answering a question while it
    drives, in a second process -- and STOP is the one control anyone in the room will reach for.
    A stop that halts the wheels while the robot carries on cheerfully talking about employee
    ownership is not a stop; it is a robot that has stopped listening.
    """
    _stop_drive()
    for slot in (_task, _chat):
        p = slot["proc"]
        if p is not None and p.poll() is None:
            try:
                p.terminate()             # ask guide/brain to quit...
                try:
                    p.wait(timeout=2)
                except Exception:
                    p.kill()              # ...and make sure it's gone
            except Exception:
                pass
        slot["proc"] = None
    _task["what"] = ""
    if not HAS_ROS:
        return True, "Stopped (preview)."
    # Killing the task doesn't stop Nav2 — cancel the active goal + zero cmd_vel.
    run_waypoint(["stop"])
    return True, "Stopped. Wall-E is holding position."


def listen_start():
    """Begin recording on the robot's mic array. Returns (ok, message)."""
    if not HAS_ROS:
        return True, "Listening… (preview)"
    if _listen["proc"] is not None and _listen["proc"].poll() is None:
        return False, "Already listening."
    if not os.path.exists(VOICE_PY):
        return False, "Voice environment not found on the robot."
    # stderr is NOT thrown away: listen_toggle reports which mic it got, how long arecord took to
    # stop and how long the transcription took. Swallowing it is what let the mic fail silently for
    # weeks. It goes to the journal, where the timings live.
    _listen["proc"] = subprocess.Popen(
        [VOICE_PY, LISTEN_SCRIPT, LISTEN_WAV], cwd=VOICE_DIR,
        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=None, text=True)
    return True, "Listening… tap again to stop."


def listen_stop():
    """Stop recording, transcribe, and return (ok, recognized_text).

    Timed, and printed to the journal (journalctl -u robot-web). Every "why is the robot so slow"
    answer on this project has turned out to be somewhere nobody was looking -- a Whisper model
    reloaded per message, a Claude reply forty words long -- so measure the visitor-facing path
    instead of guessing at it.
    """
    t0 = time.monotonic()
    p = _listen["proc"]
    _listen["proc"] = None
    if not HAS_ROS or p is None:
        return (HAS_ROS is False), ""
    try:
        # The newline is the "second ENTER" that stops listen_toggle.py; it then
        # transcribes and prints just the text, which comes back on stdout.
        out, _ = p.communicate(input="\n", timeout=45)
    except Exception:
        try:
            p.kill()
        except Exception:
            pass
        return False, ""
    lines = [ln.strip() for ln in (out or "").splitlines() if ln.strip()]
    text = lines[-1] if lines else ""
    print(f"[timing] speech-to-text: {time.monotonic()-t0:.2f}s -> {text!r}", flush=True)
    if not text:
        print("[timing]   (nothing heard -- mic muted, too far away, or arecord could not open it)",
              flush=True)
    return True, text


def arm_joint(servo_id, pos, duration=0.35):
    """Move ONE arm servo to an absolute pulse (0-1000). Used by the free-move
    sliders; publishes straight to the servo bus so dragging feels responsive."""
    pub = _ros.get("arm_pub")
    if not (HAS_ROS and HAS_ARM) or pub is None:
        return False, "Arm control not available."
    if _busy():                       # arm is locked in the scan pose while navigating
        return False, "Arm is locked while Wall-E is navigating."
    try:
        pos = max(0, min(1000, int(pos)))
        msg = ServosPosition()
        msg.position_unit = "pulse"
        msg.duration = float(duration)
        sp = ServoPosition()
        sp.id = int(servo_id)
        sp.position = float(pos)
        msg.position = [sp]
        pub.publish(msg)
        return True, "ok"
    except Exception as e:
        return False, str(e)


def arm_center():
    """Move the whole arm back to its start/home pose, eased slowly. This is the rest
    pose the arm boots in (NOT the mechanical all-500 center) and it mirrors the free-move
    sliders' defaults in index.html (JOINTS) — keep the two in sync if you retune it."""
    pub = _ros.get("arm_pub")
    if not (HAS_ROS and HAS_ARM) or pub is None:
        return False, "Arm control not available."
    if _busy():                       # arm is locked in the scan pose while navigating
        return False, "Arm is locked while Wall-E is navigating."
    try:
        msg = ServosPosition()
        msg.position_unit = "pulse"
        msg.duration = 1.5              # slow so it doesn't snap across
        home = {1: 500, 2: 765, 3: 15, 4: 220, 5: 500, 10: 500}   # start/rest pose
        data = []
        for sid, pos in home.items():
            sp = ServoPosition()
            sp.id = sid
            sp.position = float(pos)
            data.append(sp)
        msg.position = data
        pub.publish(msg)
        return True, "Arm reset to start."
    except Exception as e:
        return False, str(e)


def send_goto(x, y):
    """Tap-to-go: send Nav2 to an arbitrary map point via /goal_pose (Nav2 must be up)."""
    pub = _ros.get("goal_pub")
    node = _ros.get("node")
    if not HAS_ROS or pub is None or node is None:
        return False, "Navigation not available."
    if _busy():
        return False, "Wall-E is busy right now."
    if x is None or y is None:
        return False, "No location."
    try:
        _stop_drive()                  # don't fight teleop
        # Arm -> scan pose so the camera catches low obstacles on this trip too.
        # Non-blocking (Popen) so the tap doesn't stall the web; the arm tilts while
        # Nav2 plans. Same 'scan' gesture guide.py uses. Best-effort.
        try:
            subprocess.Popen(["python3", ARM_TOOL, "scan"],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = 1.0
        for _ in range(5):             # publish a few times so Nav2 reliably receives it
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            time.sleep(0.15)
        return True, "On my way there."
    except Exception as e:
        return False, str(e)


def set_initial_pose(x, y, yaw):
    """Tap-to-localize: tell AMCL where the robot is (and which way it faces) by
    publishing /initialpose, so nobody needs RViz's '2D Pose Estimate'. yaw in rad."""
    pub = _ros.get("initialpose_pub")
    node = _ros.get("node")
    if not HAS_ROS or pub is None or node is None:
        return False, "Navigation not available."
    if x is None or y is None:
        return False, "No location."
    try:
        yaw = float(yaw or 0.0)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36                      # AMCL's initial uncertainty (6x6, row-major)
        cov[0] = 0.25                         # x: ~0.5 m std
        cov[7] = 0.25                         # y: ~0.5 m std
        cov[35] = 0.0685                      # yaw: ~15 deg std
        msg.pose.covariance = cov
        for _ in range(3):                    # publish a few times so AMCL reliably gets it
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            time.sleep(0.15)
        return True, "Got it — I know where I am now."
    except Exception as e:
        return False, str(e)


def _on_amcl(msg):
    """Cache the latest robot pose (map dot) and mark localization alive."""
    p = msg.pose.pose
    _pose["x"] = p.position.x
    _pose["y"] = p.position.y
    _pose["yaw"] = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
    _health["pose_t"] = time.monotonic()


def _on_scan(msg):
    _health["scan_t"] = time.monotonic()          # lidar is alive


def _on_camera_scan(msg):
    # camera_scan_node only publishes when the depth driver is feeding it, so one message here
    # means the WHOLE low-obstacle path is alive: driver -> points -> node -> costmap.
    _health["cam_t"] = time.monotonic()


_scan_node = {"alive": False, "t": 0.0}      # cached pgrep: see _scan_node_alive()


def _scan_node_alive():
    """Is camera_scan_node still running? Cached -- the watchdog asks twice a second.

    We ask the PROCESS, not a topic. The far end of the depth chain going quiet has two very
    different causes and they need opposite treatment: a covered lens (leave it alone) and a dead
    camera_scan_node (restart it). Both look identical from ROS. Only the process table tells them
    apart. Not subscribing to /depth_cam/depth/points to find out is deliberate: that cloud is
    230k points at 15 Hz, and deserialising it is exactly the CPU bill that starved this Jetson.
    """
    now = time.monotonic()
    if now - _scan_node["t"] < 2.0:
        return _scan_node["alive"]
    try:
        # ANCHORED ON ^python3, and not out of tidiness. pgrep -f matches whole command lines, and
        # several things on this robot carry "camera_scan_node.py" in theirs -- camera_check.sh's
        # own pgrep, camera_scan.sh's. Match the bare name and, in the wrong millisecond, we find
        # someone else's *search* for the node and report the node itself alive. It would be a
        # once-a-week lie, which is the worst kind. Only the real node's command line begins with
        # python3.
        r = subprocess.run(["pgrep", "-f", r"^python3 .*camera_scan_node\.py"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2)
        _scan_node["alive"] = (r.returncode == 0)
    except Exception:
        _scan_node["alive"] = False       # can't tell -> assume the worst, so we repair
    _scan_node["t"] = now
    return _scan_node["alive"]


def camera_state():
    """'ok' | 'blocked' | 'stale' (it died) | 'off' (never came up).

    BLOCKED is the state that cost us a day, and it is not a fault at all.

    Put anything across the lens -- a wall a hand's width away, a visitor standing right in front
    of the robot -- and every depth point falls inside the sensor's 30 cm blind spot. The driver is
    in perfect health: it keeps streaming colour, it simply has no valid point to put in the cloud,
    so it publishes none. /camera_scan goes quiet. The old code read that silence as "the camera
    died", and then:

      - it KILLED a healthy driver and relaunched it, every 60 s, for ever. Against a wall, that is
        a fight it cannot win: 25 repairs, 24 failures, and each one killed half-way left orphans.
      - mid-trip it stopped the robot and announced a hardware failure that did not exist.

    Colour is the tell. A DEAD camera stops publishing everything; a BLOCKED one still sends
    pictures. So: depth quiet + colour flowing = nothing is broken, something is in the way.

    Colour alone is not enough, though. /camera_scan can also go quiet with a perfectly clear view,
    because camera_scan_node -- the far end of the chain -- died. Colour would still be flowing, we
    would call that "blocked", refuse to repair it, and the robot would drive blind to chair legs
    with the watchdog insisting nothing was wrong. So we ask whether the node is even alive. Only
    a live driver AND a live node AND a silent scan is a blocked lens; anything else is a fault, and
    faults get repaired.
    """
    now = time.monotonic()
    if _health["cam_t"] and (now - _health["cam_t"]) < CAM_STALE:
        return "ok"
    colour_flowing = CAMERA.last_rx and (now - CAMERA.last_rx) < CAM_STALE
    if colour_flowing and _scan_node_alive():
        return "blocked"
    return "off" if _health["cam_t"] == 0.0 else "stale"


def _speak(text):
    """Say one line out loud, in its own process, never blocking the web."""
    try:
        subprocess.Popen(["python3", SPEAK_TOOL, text],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass


def _speak_wait(text, timeout=8.0):
    """Say one line and WAIT for the mouth to close, so the next thing (a capture) happens after.
    Best-effort: returns quietly if the speaker is muted or speak.py wedges."""
    if not speaker_on():
        return
    try:
        subprocess.run(["python3", SPEAK_TOOL, text],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=timeout)
    except Exception:
        pass


def _repair_camera():
    """Bring the depth camera back, without anyone needing to know what a camera_scan_node is.

    Runs camera_check.sh in its own session (setsid): a Ctrl-C in whatever terminal launched the
    web must not kill the camera we just revived -- that is exactly how it kept dying. One repair
    at a time, and at most one per CAM_COOLDOWN, so an unplugged camera cannot spawn a storm.
    """
    with _lock:
        if _repair["running"] or (time.monotonic() - _repair["last"]) < CAM_COOLDOWN:
            return
        _repair["running"] = True
        _repair["last"] = time.monotonic()
        _repair["tries"] += 1

    def work():
        try:
            print("[camera-watchdog] camera is down -> running camera_check.sh", flush=True)
            r = subprocess.run(["setsid", "bash", CAM_CHECK], stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, timeout=90)
            # 2 = BLOCKED: the camera is healthy, the view is not. Nothing was restarted and
            # nothing should be. Saying "FAILED" here sent us hunting a hardware fault that did
            # not exist while the robot sat with its nose in a corner.
            verdict = {0: "OK", 2: "BLOCKED (camera is fine -- move it away from what it faces)"}
            print(f"[camera-watchdog] repair {verdict.get(r.returncode, 'FAILED')}:\n{r.stdout}",
                  flush=True)
        except Exception as e:
            print(f"[camera-watchdog] repair blew up: {e}", flush=True)
        finally:
            _repair["running"] = False

    threading.Thread(target=work, daemon=True).start()


def _camera_watchdog():
    """Keep the depth camera alive, and stop the robot if it dies mid-trip.

    Two jobs, because the camera failing is both a safety problem and a chore:

      ALWAYS  -- if the camera path is down, run camera_check.sh to bring it back. Nobody using
                 the robot should have to know it exists, let alone how to restart it.
      IN A TRIP -- also stop, and say so. The lidar sweeps over chair legs; without the camera the
                 robot is blind to them and would drive on and clip one. Parked and honest beats
                 blind in front of a customer. We do NOT auto-resume the trip: a robot that starts
                 moving again on its own, next to a person, is a nasty surprise. Ask it again.

    Both of those are for a camera that has actually FAILED. A camera that is merely BLOCKED --
    healthy driver, covered lens -- gets neither: see camera_state(). Repairing it means killing a
    working driver in a loop it cannot win, and stopping the trip to announce a hardware fault that
    does not exist is the worst thing this robot could do with a customer watching.
    """
    was_ok = True
    blocked_since = 0.0                      # when the lens was first covered (0 = it isn't)

    # SUBSCRIBE TO COLOUR NOW, not when the first phone opens the app.
    # camera_state() tells a BLOCKED camera from a DEAD one by asking whether colour frames are
    # still arriving -- and Camera.start() only subscribes on first use. Left lazy, a robot nobody
    # had opened the phone app on had no colour to check, every blocked camera looked dead, and the
    # kill-and-relaunch loop was back. The watchdog is the one thing that always runs, so it opens
    # the subscription itself.
    #
    # IN ITS OWN THREAD, because Camera.start() blocks for up to 3 s waiting for a first frame, and
    # it has to be retried until the driver is up. Waiting for it here cost the watchdog FOUR
    # MINUTES of blindness at boot on a robot whose camera was down -- which is precisely the robot
    # that needs it awake. It watches from the first second now, and the subscription lands when it
    # lands.
    def _open_colour():
        while not CAMERA.start():
            time.sleep(2.0)                  # ROS, or the driver, may still be coming up
    threading.Thread(target=_open_colour, daemon=True).start()

    while True:
        time.sleep(0.5)
        try:
            state = camera_state()
            on_trip = _busy() and _task["what"] in DRIVING_TASKS
            fresh_trip = on_trip and (time.monotonic() - _task["t0"]) < CAM_GRACE

            if state == "ok":
                if not was_ok:
                    print("[camera-watchdog] camera is back", flush=True)
                    if _health["cam_fail"]:
                        _speak("My depth camera is back. You can ask me to go again.")
                was_ok = True
                blocked_since = 0.0
                continue

            # BLOCKED: the camera is healthy, the view is not. Never "repair" this -- the driver is
            # fine, and killing it changes nothing about the wall in front of the lens. Whatever is
            # that close is big enough for the LIDAR to see anyway, so Nav2 is not driving blind.
            if state == "blocked":
                now = time.monotonic()
                if not blocked_since:
                    blocked_since = now
                    print("[camera-watchdog] depth is blocked -- colour still flowing, so the "
                          "camera is fine; something is right in front of the lens", flush=True)
                # Someone crossing in front of the robot blocks it for a second or two. Only a view
                # that STAYS covered, on a trip, is worth stopping for -- and then we say what is
                # actually true, and what would fix it.
                if on_trip and not fresh_trip and (now - blocked_since) > CAM_BLOCKED_GRACE:
                    if not _health["cam_fail"]:
                        _health["cam_fail"] = "something is blocking the depth camera"
                        print("[camera-watchdog] blocked for "
                              f"{CAM_BLOCKED_GRACE:.0f}s on a trip -> stopping", flush=True)
                        stop_all()
                        _speak("Something is right in front of me, so I cannot see the floor. "
                               "Please step aside, then ask me to go again.")
                was_ok = False
                continue

            blocked_since = 0.0

            if state == "off" and _repair["tries"] == 0 and not on_trip:
                _repair_camera()             # first boot: bring it up before anyone asks for a trip
                continue

            if on_trip and not fresh_trip:
                if was_ok or not _health["cam_fail"]:
                    why = ("the depth camera never started" if state == "off"
                           else "the depth camera stopped")
                    print(f"[camera-watchdog] {why} -> stopping the trip", flush=True)
                    _health["cam_fail"] = why
                    stop_all()               # cancels the Nav2 goal and zeroes cmd_vel
                    _speak("I lost my depth camera, so I have stopped. I cannot see low "
                           "obstacles like chair legs. I am bringing it back now.")
                was_ok = False
                _repair_camera()
                continue

            was_ok = False
            if not fresh_trip:
                _repair_camera()             # idle (or manual driving): just quietly fix it
        except Exception:
            pass                             # a watchdog that crashes is worse than none


def _on_batt(msg):
    _health["batt"] = int(msg.data)               # raw battery value (mV)
    _health["batt_t"] = time.monotonic()


def run_waypoint(args):
    """Run waypoint_tool.py save/del. Returns (ok, message)."""
    if not HAS_ROS:
        return True, "(preview)"
    try:
        r = subprocess.run(["python3", WAYPOINT_TOOL] + args,
                           capture_output=True, text=True, timeout=30)
        lines = ((r.stdout or "") + (r.stderr or "")).strip().splitlines()
        msg = lines[-1] if lines else ("done" if r.returncode == 0 else "failed")
        return r.returncode == 0, msg
    except Exception as e:
        return False, str(e)


def list_places():
    """Names of the saved destinations."""
    if not HAS_ROS:
        return list(_preview_places)
    try:
        import yaml
        with open(WAYPOINTS_YAML) as f:
            data = yaml.safe_load(f) or {}
        return sorted((data.get("waypoints") or {}).keys())
    except Exception:
        return []


def map_meta():
    """Map geometry + saved waypoints, so the phone can convert taps <-> world."""
    try:
        import yaml
        with open(MAP_YAML) as f:
            m = yaml.safe_load(f) or {}
        res = float(m.get("resolution", 0.05))
        ox, oy = (m.get("origin") or [0, 0, 0])[:2]
        wps = {}
        if os.path.exists(WAYPOINTS_YAML):
            with open(WAYPOINTS_YAML) as f:
                wdata = yaml.safe_load(f) or {}
            wps = {n: {"x": v.get("x"), "y": v.get("y")}
                   for n, v in (wdata.get("waypoints") or {}).items()}
        w = h = None
        if HAS_CV2:
            img = cv2.imread(MAP_PGM, cv2.IMREAD_GRAYSCALE)
            if img is not None:
                h, w = img.shape[:2]
        return {"ok": w is not None, "width": w, "height": h, "resolution": res,
                "origin_x": float(ox), "origin_y": float(oy), "waypoints": wps}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def map_png():
    """The map as PNG bytes (browsers can't render .pgm), or None."""
    if not HAS_CV2:
        return None
    try:
        img = cv2.imread(MAP_PGM, cv2.IMREAD_GRAYSCALE)
        if img is None:
            return None
        enc, buf = cv2.imencode(".png", img)
        return buf.tobytes() if enc else None
    except Exception:
        return None


def add_point_at(name, x, y):
    """Save a waypoint at an arbitrary map point (from tapping the map)."""
    name = (name or "").strip()
    if not name:
        return False, "Type a name."
    if x is None or y is None:
        return False, "No location."
    try:
        import yaml
        data = {}
        if os.path.exists(WAYPOINTS_YAML):
            with open(WAYPOINTS_YAML) as f:
                data = yaml.safe_load(f) or {}
        wps = data.get("waypoints") or {}
        wps[name] = {"x": round(float(x), 3), "y": round(float(y), 3), "yaw": 0.0}
        with open(WAYPOINTS_YAML, "w") as f:
            yaml.safe_dump({"map": "map_01", "waypoints": wps}, f,
                           allow_unicode=True, sort_keys=True)
        return True, 'Saved "%s".' % name
    except Exception as e:
        return False, str(e)


def suggest_knowledge(name):
    """Ask Claude what a newly-named point most likely is, so the web can PREFILL
    the 'what is here?' form (the human confirms/edits before it's saved). Returns
    {"ok":True,"what":..,"aka":[..],"category":..}. Blank (manual) if Claude isn't
    available or fails -- never blocks saving the point."""
    name = (name or "").strip()
    blank = {"ok": True, "what": "", "aka": [], "category": "landmark"}
    claude = shutil.which("claude")
    if not name or not claude:
        return blank
    prompt = (
        "A visitor-guide robot in an office just saved a new location named '"
        + name + "'. Guess briefly what this place most likely is. Reply with ONLY "
        "a JSON object (no prose) with keys: "
        '"what" (one short spoken sentence describing the place, in English), '
        '"aka" (list of 2-4 short alternative names people might say, English, lowercase), '
        '"category" (one of: team, room, amenity, landmark). '
        "If the name is too vague to guess, use empty what and aka."
    )
    try:
        r = subprocess.run([claude, "-p", prompt], capture_output=True,
                           text=True, timeout=45)
        out = (r.stdout or "").strip()
        i, j = out.find("{"), out.rfind("}")
        data = json.loads(out[i:j + 1]) if 0 <= i < j else {}
        return {"ok": True,
                "what": str(data.get("what", "")).strip(),
                "aka": [str(a).strip() for a in (data.get("aka") or []) if str(a).strip()],
                "category": (str(data.get("category", "")).strip() or "landmark")}
    except Exception:
        return blank


def save_knowledge(name, what, aka, category="landmark", who="", fun_fact=""):
    """Append a knowledge stub for `name` to office_knowledge.yaml so the voice
    brain knows what's there. APPENDS a text block (the file ends inside the
    `places:` map) instead of re-dumping the YAML, to preserve the file's helpful
    comments. Skips if the name already has an entry (no duplicate keys)."""
    name = (name or "").strip()
    if not name:
        return False, "No name."
    try:
        import yaml
        existing = {}
        if os.path.exists(KNOWLEDGE_YAML):
            with open(KNOWLEDGE_YAML) as f:
                existing = yaml.safe_load(f) or {}
        if name in (existing.get("places") or {}):
            return True, 'Already knew "%s".' % name

        def q(s):                       # quote a scalar, neutralising any quotes
            return '"%s"' % str(s or "").replace('"', "'")
        aka_items = [a for a in (aka or []) if str(a).strip()]
        aka_str = "[" + ", ".join(q(a) for a in aka_items) + "]"
        block = ("\n  %s:\n"
                 "    category: %s\n"
                 "    aka: %s\n"
                 "    what: %s\n"
                 "    who: %s\n"
                 "    fun_fact: %s\n") % (
            name, (category or "landmark"), aka_str, q(what), q(who), q(fun_fact))
        with open(KNOWLEDGE_YAML) as f:
            content = f.read()
        # make sure there's a places: section to append under (there always is)
        prefix = "" if ("\nplaces:" in content or content.startswith("places:")) else "\nplaces:\n"
        with open(KNOWLEDGE_YAML, "a") as f:
            f.write(prefix + block)
        return True, 'Got it — I\'ll remember what\'s at "%s".' % name
    except Exception as e:
        return False, str(e)


def knowledge_desc():
    """{name: what} from office_knowledge.yaml, so the phone can show a short
    description under each saved place. Empty {} if the file's missing."""
    try:
        import yaml
        with open(KNOWLEDGE_YAML) as f:
            kb = yaml.safe_load(f) or {}
        out = {}
        for n, p in (kb.get("places") or {}).items():
            w = (p or {}).get("what")
            if w:
                out[n] = str(w).strip()
        return out
    except Exception:
        return {}


def read_index():
    try:
        with open(INDEX_HTML, "rb") as f:
            return f.read()
    except Exception:
        return b"<h1>index.html not found next to server.py</h1>"


def get_lan_ip():
    """This machine's address on the office WiFi (no packet is actually sent)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip


def qr_url():
    return f"http://{get_lan_ip()}:{PORT}"


def make_qr(url):
    """Return a PNG data-URI of the QR, or None if no QR library is available."""
    try:                                   # segno: pure-python, no dependencies
        import segno
        return segno.make(url, error="m").png_data_uri(
            scale=5, border=2, dark="#0b1f12", light="#eafff1")
    except Exception:
        pass
    try:                                   # qrcode: needs Pillow
        import io
        import base64
        import qrcode
        buf = io.BytesIO()
        qrcode.make(url).save(buf, format="PNG")
        return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode()
    except Exception:
        return None


# ---- Camera (Orbbec RGB over ROS) -------------------------------------------
class Camera:
    """Subscribes to the Orbbec color topic and keeps the latest frame as JPEG.

    The DaBai DCW is not a UVC device, so cv2.VideoCapture can't reach it -- the
    frames arrive as sensor_msgs/Image on /depth_cam/rgb/image_raw (same topic the
    collect_picture app uses). We convert each to JPEG once and serve the latest."""

    def __init__(self, topic):
        self.topic = topic
        self.frame = None          # latest JPEG bytes
        self.lock = threading.Lock()
        self.started = False
        self.ok = False
        self.node = None
        self.bridge = None
        self.last_rx = 0.0

    def start(self):
        """Subscribe on first use. Returns True if frames are flowing."""
        if not (HAS_ROS and HAS_CV2):
            return False
        if not self.started:
            if not rclpy.ok():         # web server up but ROS not initialised yet
                return False
            try:
                from sensor_msgs.msg import Image
                from cv_bridge import CvBridge
                from rclpy.qos import qos_profile_sensor_data
            except Exception:
                return False
            self.bridge = CvBridge()
            self.node = rclpy.create_node("robot_web_cam")
            # sensor_data QoS (best-effort) is compatible with a reliable OR a
            # best-effort image publisher, so it won't silently receive nothing.
            self.node.create_subscription(
                Image, self.topic, self._on_image, qos_profile_sensor_data)
            self.started = True
            threading.Thread(target=self._spin, daemon=True).start()
        for _ in range(60):        # wait up to ~3 s for the first frame
            if self.frame is not None:
                break
            time.sleep(0.05)
        return self.ok

    def _spin(self):
        # Use a DEDICATED executor for the camera node. rclpy.spin()/spin_once()
        # default to the shared global executor; the main loop already drives that
        # with spin_once(), so spinning here too would have two threads mutating the
        # same wait set -> "IndexError: wait set index too big" and a crash.
        try:
            from rclpy.executors import SingleThreadedExecutor
            ex = SingleThreadedExecutor()
            ex.add_node(self.node)
            ex.spin()
        except Exception:
            pass

    def _on_image(self, msg):
        try:
            # bgr8 -> cv2.imencode writes correct colors straight to JPEG
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            enc, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if enc:
                with self.lock:
                    self.frame = buf.tobytes()
                self.ok = True
                self.last_rx = time.monotonic()
        except Exception:
            pass

    def jpeg(self):
        # if the camera stopped publishing, don't keep claiming we're "live"
        if self.last_rx and time.monotonic() - self.last_rx > 3.0:
            self.ok = False
        with self.lock:
            return self.frame


CAMERA = Camera(CAM_TOPIC)


# ---- "Add me": teaching the robot a face, from the robot's OWN camera ---------
#
# The obvious design was a form where people upload selfies. It is worse in every way that
# matters: a phone photo is taken level, in good light, at high resolution -- and this robot
# looks UP at you from 45 cm through a 640x480 webcam. You would be enrolling a face it never
# actually sees. Enrol through the same lens that has to do the recognising and the problem
# disappears. It also means no photo of an employee is ever stored, anywhere: we keep the
# 128-number fingerprint and throw the picture away.
#
# The camera is opened ON DEMAND and released once nobody is looking -- face_greet.py wants the
# same device, and two processes cannot stream one webcam. Whoever needs it, takes it, briefly.
FACE_IDLE_S = 8.0          # release the camera this long after the last preview frame
FACE_SHOTS = 6             # fingerprints per person
FACE_SHOT_GAP = 0.7        # s between shots -- time to turn your head a little
FACE_MIN_PX = 60           # a face smaller than this is too far to enrol well


FACE_GREET = os.path.expanduser("~/ros2_ws/face_greet.py")
PREBAKE = os.path.expanduser("~/ros2_ws/prebake_lines.py")


def enrol_notes(rec, db, name, vectors):
    """A short, honest heads-up appended to an enrolment result. Two things a tester needs to hear
    AT THE MOMENT they enrol, not after the robot fails to know them in front of people:

      - the shots are near-duplicates, so only one angle was learned (scan instead), and
      - this face is confusable with someone already enrolled.

    Returns '' when the enrolment is strong and unambiguous."""
    notes = []
    quality, _ = face_lib.enrolment_quality(rec, vectors)
    if quality in ("weak", "single"):
        notes.append("Heads up — those look almost identical, so I really only learned ONE angle. "
                     "For reliable recognition, use the live scan above and turn your head a little.")
    elif quality == "thin":
        notes.append("Tip: a live scan with your head turned a little would help me know you from "
                     "any angle.")
    other, sc = face_lib.closest_other(rec, db, vectors, exclude=name)
    if other and sc > face_lib.MATCH_THRESHOLD - 0.05:
        notes.append(f"Also, you look quite like {other} to me — I could mix you two up. "
                     f"A clear scan of you both is the fix.")
    return " ".join(notes)


def rebake_greetings():
    """Bake the per-name greeting lines after someone is enrolled, in the background. Their first
    real greeting is then an instant aplay instead of a ~3 s Piper synthesis mid-corridor -- the
    same pre-baking guide.py already does for the walk lines. Skips whatever is baked, so cheap."""
    try:
        subprocess.Popen(["python3", PREBAKE], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL, start_new_session=True)
    except Exception as e:
        print(f"[faces] could not kick off a re-bake: {e}", flush=True)

# The switch REMEMBERS. Same trick as the speaker mute and the explanations toggle: a file in the
# cache dir, so a reboot -- or a robot-web restart nobody noticed -- does not quietly turn the
# robot back into something that walks past people it knows without a word. Turn it on once.
GREETING_ON_FLAG = os.path.expanduser("~/.cache/robot_ds/greeting_on")

# How long the greeter waits before saying hello to the SAME person again. The web writes the
# seconds here; face_greet.py re-reads the file when it changes. Missing -> face_greet's default (300).
COOLDOWN_FILE = os.path.expanduser("~/.cache/robot_ds/greet_cooldown")
COOLDOWN_DEFAULT_S = 300
COOLDOWN_CHOICES = (30, 60, 300, 600)     # the buttons the phone offers: 30s, 1m, 5m, 10m


def greeting_cooldown():
    try:
        return int(float(open(COOLDOWN_FILE).read().strip()))
    except (OSError, ValueError):
        return COOLDOWN_DEFAULT_S


def set_greeting_cooldown(seconds):
    try:
        os.makedirs(os.path.dirname(COOLDOWN_FILE), exist_ok=True)
        with open(COOLDOWN_FILE, "w") as f:
            f.write(str(int(seconds)))
    except Exception as e:
        print(f"[faces] could not set cooldown: {e}", flush=True)


def greeting_wanted():
    return os.path.exists(GREETING_ON_FLAG)


def set_greeting_flag(on):
    try:
        if on:
            os.makedirs(os.path.dirname(GREETING_ON_FLAG), exist_ok=True)
            open(GREETING_ON_FLAG, "w").close()
        elif os.path.exists(GREETING_ON_FLAG):
            os.remove(GREETING_ON_FLAG)
    except Exception as e:
        print(f"[faces] could not remember the greeting switch: {e}", flush=True)

# ONE WEBCAM, ONE OWNER -- and the web is the one who decides who that is.
#
# The greeter holds the camera to watch for faces; "Add me" needs it to show you a mirror. They
# cannot both stream one device, and asking a visitor to remember that is not a design, it is an
# excuse. So: the web PAUSES the greeter the moment it needs the camera, and starts it again
# once nobody is looking. `want` is the switch a human set; `proc` is what is running right now.
# The two are allowed to disagree for a few seconds, and the idle watcher closes the gap.
_greeter = {"proc": None, "want": False}


def greeter_running():
    p = _greeter["proc"]
    return p is not None and p.poll() is None


def greeter_start():
    if greeter_running():
        return True
    try:
        # setsid: a Ctrl-C in whatever terminal launched the web must not take the greeter with
        # it. That exact mistake is why the depth camera kept "dying on its own" for a week.
        #
        # And its output goes to OUR stdout -- i.e. journalctl -u robot-web -- rather than to
        # /dev/null. It was DEVNULL for exactly one afternoon, during which "it doesn't greet me"
        # was undebuggable: the process was up, it had the camera, and it told nobody what it saw.
        # A background worker that cannot say what it is doing is a worker you cannot fix.
        _greeter["proc"] = subprocess.Popen(
            ["python3", "-u", FACE_GREET], stdout=None, stderr=subprocess.STDOUT,
            start_new_session=True)
        return True
    except Exception as e:
        print(f"[faces] could not start the greeter: {e}", flush=True)
        return False


def greeter_stop():
    p = _greeter["proc"]
    if p is not None and p.poll() is None:
        try:
            p.terminate()
            p.wait(timeout=3)
        except Exception:
            try:
                p.kill()
            except Exception:
                pass
    _greeter["proc"] = None


class FaceCam:
    """The C270, opened only while someone is actually using it."""

    def __init__(self):
        self.lock = threading.Lock()
        self.cap = None
        self.det = None
        self.rec = None
        self.last_use = 0.0
        self.busy = False          # an enrolment is running: previews must not steal frames

    def _ensure(self):
        if self.det is None:
            self.det, self.rec = face_lib.load_models()
        if self.cap is None:
            greeter_stop()                    # our turn with the camera; it gets it back later
            time.sleep(0.4)                   # let the kernel actually free the device
            cap, err = face_lib.open_camera()
            if cap is None:
                return err
            self.cap = cap
        self.last_use = time.monotonic()
        return None

    def release_if_idle(self):
        with self.lock:
            if (self.cap is not None and not self.busy
                    and time.monotonic() - self.last_use > FACE_IDLE_S):
                try:
                    self.cap.release()
                except Exception:
                    pass
                self.cap = None
            # Nobody is holding the camera and a human asked for the greeter: give it back.
            if self.cap is None and _greeter["want"] and not greeter_running():
                greeter_start()

    def preview_jpeg(self):
        """One frame with the face boxed and a plain-English hint drawn on it. The hint is on the
        IMAGE rather than in a second request: the person is looking at the picture, not at JSON."""
        with self.lock:
            err = self._ensure()
            if err:
                return None
            ok, frame = self.cap.read()
            if not ok:
                return None
            face = face_lib.biggest_face(self.det, frame)

        if face is None:
            msg, colour = "Step in front of the robot", (60, 170, 250)
        else:
            x, y, w, h = face[:4].astype(int)
            if h < FACE_MIN_PX:
                msg, colour = "Come a bit closer", (60, 170, 250)
            else:
                msg, colour = "Looking good", (80, 220, 120)
            cv2.rectangle(frame, (x, y), (x + w, y + h), colour, 2)
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 34), (20, 28, 24), -1)
        cv2.putText(frame, msg, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.62, colour, 2)
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        return buf.tobytes() if ok else None

    def enroll_photos(self, name, images):
        """Enrol from uploaded pictures instead of the robot's own camera.

        Worse than the camera, and worth saying why: this robot looks UP at you from 45 cm through
        a 640x480 webcam, and a phone photo is level, bright and sharp. You are teaching it a face
        it never quite sees. It still works -- SFace survives a bigger pose change than that (a
        standing shot and a sitting one measured 0.562, against a 0.363 threshold) -- but the
        camera route has more margin. It is here because uploading is easier for a room full of
        people, and easier wins the ones who would otherwise never enrol.

        The pictures are decoded, measured and dropped. They never touch the disk. What we keep is
        the same 128 numbers as always.

        `images` is a list of data: URLs. Returns (ok, message).
        """
        if self.det is None:
            self.det, self.rec = face_lib.load_models()
        vectors, skipped = [], 0
        for data_url in images:
            try:
                b64 = data_url.split(",", 1)[-1]
                buf = np.frombuffer(base64.b64decode(b64), dtype=np.uint8)
                img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                if img is None:
                    skipped += 1
                    continue
                face = face_lib.biggest_face(self.det, img)
                if face is None:
                    skipped += 1          # a photo with no face in it: say so, do not guess
                    continue
                vectors.append(face_lib.face_vector(self.rec, img, face))
            except Exception:
                skipped += 1
        if not vectors:
            return False, "I couldn't find a face in any of those. Try clearer, closer photos."
        db = face_lib.load_db()
        face_lib.set_enrolled(db, name, vectors)   # replaces, and drops what it had learned
        face_lib.save_db(db)
        rebake_greetings()                         # bake "Hi <name>!" so the first hello is instant
        msg = f"Got it — {len(vectors)} photo(s) of {name}."
        if skipped:
            msg += f" ({skipped} had no face I could see.)"
        note = enrol_notes(self.rec, face_lib.load_db(), name, vectors)
        if note:
            msg += "  " + note
        return True, msg

    def _grab_shots(self, want, budget_s):
        """Grab up to `want` face fingerprints, giving up after budget_s. Returns a list."""
        got = []
        tries = 0
        t0 = time.monotonic()
        while len(got) < want and time.monotonic() - t0 < budget_s and tries < want * 8:
            tries += 1
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            face = face_lib.biggest_face(self.det, frame)
            if face is None or int(face[3]) < FACE_MIN_PX:
                time.sleep(0.15)
                continue
            got.append(face_lib.face_vector(self.rec, frame, face))
            time.sleep(FACE_SHOT_GAP)
        return got

    def enroll(self, name):
        """Scan a face while the robot talks the person through turning their head, then store
        the fingerprints under `name`. Blocks ~8-10 s. Spoken guidance is best-effort: if the
        speaker is muted the scan still runs, just silently."""
        with self.lock:
            err = self._ensure()
            if err:
                return False, err
            self.busy = True
        try:
            for _ in range(6):                      # let auto-exposure settle
                self.cap.read()
            # Speak a line (and wait for the mouth to close), then capture a couple of shots at
            # that head angle. Each step catches a different view, which is what makes the robot
            # recognise them from across the room later, not only head-on.
            steps = [
                ("Look at me.",                       2),
                ("Now turn your head slowly to the left.", 2),
                ("And to the right.",                 2),
                ("Almost done.",                      1),
            ]
            vectors = []
            for line, n in steps:
                _speak_wait(line)                     # says it out loud (or no-ops if muted)
                vectors += self._grab_shots(n, budget_s=3.0)
            if len(vectors) < 2:
                _speak_wait("Sorry, I couldn't see your face.")
                return False, "I couldn't see your face. Stand in front of the robot and try again."
            _speak_wait("Got it!")
            db = face_lib.load_db()
            face_lib.set_enrolled(db, name, vectors)   # replaces, and drops what it had learned
            face_lib.save_db(db)
            rebake_greetings()                         # bake "Hi <name>!" so the first hello is instant
            msg = f"Got it — I'll say hello next time, {name}!"
            note = enrol_notes(self.rec, face_lib.load_db(), name, vectors)
            if note:                                   # a scan is usually 'good', but flag a clash
                msg += "  " + note
            return True, msg
        except Exception as e:
            return False, f"Enrolment failed ({e})."
        finally:
            self.busy = False
            self.last_use = time.monotonic()


FACECAM = FaceCam() if HAS_FACES else None


def _face_idle_watch():
    # Two jobs, every couple of seconds. Releasing the idle camera, and -- the part that matters for
    # a day of testing -- keeping the greeter ALIVE. It is a subprocess; subprocesses die (an
    # unhandled frame, an OOM, a wedged mic). Before, its only restart path was tangled inside the
    # camera-release logic, so a crash at the wrong moment left the robot standing there, switched
    # "on", greeting nobody, with no hint why. Now: if a human asked for greeting and it is not
    # running and nothing else holds the camera, bring it back -- and say so, once, not every loop.
    warned = False
    while True:
        time.sleep(2.0)
        try:
            FACECAM.release_if_idle()
            if _greeter["want"] and not greeter_running() and FACECAM.cap is None:
                if not warned:
                    print("[faces] greeter is down but switched on -> restarting it", flush=True)
                    warned = True
                greeter_start()
            elif greeter_running():
                warned = False               # healthy again; a future death is worth reporting
        except Exception:
            pass


if HAS_FACES:
    threading.Thread(target=_face_idle_watch, daemon=True).start()
    # Left switched on last time? Then switch on now, without anyone being asked to remember.
    # The idle watcher is already running, so it starts the greeter within a couple of seconds
    # -- and keeps starting it again after every pause for the mirror, forever.
    if greeting_wanted():
        if face_lib.load_faces():
            _greeter["want"] = True
            print("[faces] greeting was left ON -> starting the greeter", flush=True)
        else:
            set_greeting_flag(False)          # everyone was removed: do not promise what we cannot do
            print("[faces] greeting was ON but nobody is enrolled -> off", flush=True)


# ---- HTTP handler -----------------------------------------------------------
class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self):
        n = int(self.headers.get("Content-Length", 0) or 0)
        if not n:
            return {}
        try:
            return json.loads(self.rfile.read(n) or b"{}")
        except Exception:
            return {}

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            body = read_index()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path.split("?")[0] == "/api/face/preview.jpg":
            self._face_preview()
        elif self.path == "/api/face/list":
            if not HAS_FACES:
                self._json({"available": False, "names": []})
            else:
                # 'greeting' is what the human ASKED for, not whether the process happens to be
                # up this second -- it is paused on purpose whenever the mirror needs the camera,
                # and a switch that flickered off every time you opened Add me would be a liar.
                db = face_lib.load_db()
                # Report the two counts separately. "Rodrigo (6+4)" tells you at a glance that it
                # has been learning, and is the only way anyone would ever notice drift.
                people = [{"name": n, "enrolled": len(e["enrolled"]),
                           "learned": len(e["learned"])} for n in sorted(db) for e in [db[n]]]
                self._json({"available": True, "names": sorted(db), "people": people,
                            "greeting": _greeter["want"], "running": greeter_running(),
                            "cooldown": greeting_cooldown()})
        elif self.path == "/api/places":
            self._json({"places": list_places()})
        elif self.path == "/api/tours":
            self._json({"tours": load_tours()})
        elif self.path == "/api/qr":
            url = qr_url()
            self._json({"url": url, "img": make_qr(url) or ""})
        elif self.path.split("?")[0] == "/api/snapshot":
            self._snapshot()
        elif self.path.split("?")[0] == "/api/camera/stream":
            self._camera_stream()
        elif self.path == "/api/map/meta":
            self._json(map_meta())
        elif self.path.split("?")[0] == "/api/map/img":
            self._map_img()
        elif self.path == "/api/pose":
            self._json({"ok": _pose["x"] is not None, "x": _pose["x"],
                        "y": _pose["y"], "yaw": _pose["yaw"]})
        elif self.path == "/api/knowledge":
            self._json({"ok": True, "desc": knowledge_desc()})

        elif self.path.startswith("/api/say_log"):
            # Robot's spoken lines with id > since -> the phone chat appends them.
            since = 0
            if "since=" in self.path:
                try:
                    since = int(self.path.split("since=", 1)[1].split("&")[0])
                except ValueError:
                    since = 0
            with _lock:
                lines = [e for e in _say["log"] if e["id"] > since]
                seq = _say["seq"]
            self._json({"ok": True, "seq": seq, "lines": lines})

        elif self.path == "/api/status":
            now = time.monotonic()
            busy = _busy()
            batt = _health["batt"]
            self._json({
                "ok": True,
                "busy": busy,
                "what": _task["what"] if busy else "",
                "doing": _status["line"],                       # latest robot narration
                "battery_v": round(batt / 1000.0, 2) if batt else None,
                "battery_raw": batt,
                "lidar": (now - _health["scan_t"]) < 3.0,        # scan seen recently
                "localized": _health["pose_t"] > 0 and (now - _health["pose_t"]) < 30.0,
                # The low-obstacle path (depth camera -> costmap). 'off'/'stale' means the robot
                # cannot see chair legs -- press "Robot Web QR", which brings the whole chain up.
                "camera": camera_state(),
                "camera_fail": _health["cam_fail"],   # set when the watchdog stopped a trip
                # Can it be asked things WHILE it walks? False means the installed brain has no
                # --no-drive, so questions during a trip get refused -- while the robot is out loud
                # inviting people to ask them. That gap is one forgotten `colcon build` wide, so it
                # is reported, not assumed. demo_check.sh fails on it.
                "chat_while_driving": CAN_CHAT_WHILE_DRIVING,
                # Can it be HEARD? The phone shows this on the speaker button, because a robot that
                # has been silenced looks exactly like a robot that is working -- it still fills the
                # chat. The only honest place to show it is on the switch itself.
                "speaker": speaker_on(),
                "volume": speaker_volume(),
                "explanations": explanations_on(),
            })
        else:
            self._json({"error": "not found"}, 404)

    def _map_img(self):
        png = map_png()
        if not png:
            self._json({"error": "map not available"}, 503)
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/png")
        self.send_header("Content-Length", str(len(png)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(png)

    def _face_preview(self):
        """The mirror for 'Add me'. Without it people cannot tell whether the robot can see
        them, and enrol the top of their own head."""
        if not HAS_FACES:
            self._json({"error": "face recognition not available"}, 503)
            return
        frame = FACECAM.preview_jpeg()
        if not frame:
            self._json({"error": "forward camera not available"}, 503)
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(frame)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(frame)

    def _snapshot(self):
        CAMERA.start()
        frame = CAMERA.jpeg()
        if not frame:
            self._json({"error": "camera not available"}, 503)
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(frame)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(frame)

    def _camera_stream(self):
        if not CAMERA.start():
            self._json({"error": "camera not available"}, 503)
            return
        self.send_response(200)
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                frame = CAMERA.jpeg()
                if frame:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(("Content-Length: %d\r\n\r\n" % len(frame)).encode())
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                time.sleep(0.07)   # ~14 fps to the browser
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass   # phone closed the view

    def do_POST(self):
        d = self._read_json()
        p = self.path

        # Single-controller lock: only the pilot may command the robot. STOP and
        # control/status are exempt (anyone can stop; anyone can check/take control).
        if p in CONTROL_PATHS and not claim_control(d.get("cid")):
            self._json({"ok": False, "locked": True,
                        "message": "Another phone is controlling Wall-E right now."})
            return

        if p == "/api/control":
            self._json(control_status(d.get("cid"), (d.get("action") or "status")))

        elif p == "/api/drive":
            dr = (d.get("dir") or "stop").lower()
            if dr == "fwd":
                _set_drive(LIN_SPEED, 0.0)
            elif dr == "back":
                _set_drive(-LIN_SPEED, 0.0)
            elif dr == "left":
                _set_drive(0.0, ANG_SPEED)
            elif dr == "right":
                _set_drive(0.0, -ANG_SPEED)
            else:
                _stop_drive()
            self._json({"ok": True})

        elif p == "/api/stop":
            ok, msg = stop_all()
            self._json({"ok": ok, "message": msg})

        elif p == "/api/listen/start":
            ok, msg = listen_start()
            self._json({"ok": ok, "message": msg})

        elif p == "/api/listen/stop":
            ok, text = listen_stop()
            self._json({"ok": ok, "text": text})

        elif p == "/api/arm":
            pose = (d.get("pose") or "").strip()
            ok, msg = _run_task("arm", ["python3", ARM_TOOL, pose]) if pose else (False, "No gesture given.")
            self._json({"ok": ok, "message": msg})

        elif p == "/api/arm/joint":
            ok, msg = arm_joint(d.get("id"), d.get("pos"))
            self._json({"ok": ok, "message": msg})

        elif p == "/api/arm/center":
            ok, msg = arm_center()
            self._json({"ok": ok, "message": msg})

        elif p == "/api/goto":
            ok, msg = send_goto(d.get("x"), d.get("y"))
            self._json({"ok": ok, "message": msg})

        elif p == "/api/set_pose":
            ok, msg = set_initial_pose(d.get("x"), d.get("y"), d.get("yaw"))
            self._json({"ok": ok, "message": msg})


        elif p == "/api/add_point_at":
            if not HAS_ROS:
                nm = (d.get("name") or "").strip()
                if nm and nm not in _preview_places:
                    _preview_places.append(nm)
                self._json({"ok": True, "message": "(preview) saved " + nm})
                return
            ok, msg = add_point_at(d.get("name"), d.get("x"), d.get("y"))
            self._json({"ok": ok, "message": msg})

        elif p == "/api/suggest_knowledge":
            # Claude proposes "what is this place?" so the web can prefill the form.
            self._json(suggest_knowledge(d.get("name")))

        elif p == "/api/save_knowledge":
            if not HAS_ROS:
                self._json({"ok": True, "message": "(preview) knowledge saved"})
                return
            ok, msg = save_knowledge(d.get("name"), d.get("what"), d.get("aka"),
                                     d.get("category") or "landmark")
            self._json({"ok": ok, "message": msg})

        elif p == "/api/go":
            name = (d.get("name") or "").strip()
            ok, msg = _run_task("go", GUIDE + [name]) if name else (False, "No place given.")
            self._json({"ok": ok, "message": msg})


        elif p == "/api/messenger":
            name = (d.get("name") or "").strip()
            phrase = (d.get("phrase") or "").strip()
            if not name or not phrase:
                self._json({"ok": False, "message": "Pick a place and a message."})
                return
            ok, msg = _run_task("messenger", GUIDE + [name, "--say", phrase])
            self._json({"ok": ok, "message": msg})

        elif p == "/api/tour":
            names = [str(n).strip() for n in (d.get("names") or []) if str(n).strip()]
            if not names:
                self._json({"ok": False, "message": "Pick at least one stop."})
                return
            ok, msg = _run_task("tour", GUIDE + ["--tour"] + names)
            self._json({"ok": ok, "message": msg})

        elif p == "/api/tours/save":
            # Save the current set of stops under a name. Not gated -- it writes a file, it does not
            # move the robot. Overwrites a tour of the same name (that is how you edit one).
            name = (d.get("name") or "").strip()
            stops = [str(n).strip() for n in (d.get("names") or []) if str(n).strip()]
            if not name:
                self._json({"ok": False, "message": "Name the tour first."})
            elif not stops:
                self._json({"ok": False, "message": "Pick at least one stop to save."})
            else:
                tours = load_tours()
                tours[name] = stops
                save_tours(tours)
                self._json({"ok": True, "message": f"Saved '{name}'.", "tours": tours})

        elif p == "/api/tours/delete":
            name = (d.get("name") or "").strip()
            tours = load_tours()
            if name in tours:
                del tours[name]
                save_tours(tours)
                self._json({"ok": True, "message": f"Deleted '{name}'.", "tours": tours})
            else:
                self._json({"ok": False, "message": "No such tour.", "tours": tours})

        elif p == "/api/tours/rename":
            # Rename a saved tour, keeping its stops. Preserves order by rebuilding the dict, so the
            # renamed tour stays where it was in the row instead of jumping to the end.
            old = (d.get("old") or "").strip()
            new = (d.get("new") or "").strip()
            tours = load_tours()
            if old not in tours:
                self._json({"ok": False, "message": "No such tour.", "tours": tours})
            elif not new:
                self._json({"ok": False, "message": "Give it a name.", "tours": tours})
            elif new != old and new in tours:
                self._json({"ok": False, "message": f"'{new}' already exists.", "tours": tours})
            else:
                tours = {(new if k == old else k): v for k, v in tours.items()}
                save_tours(tours)
                self._json({"ok": True, "message": f"Renamed to '{new}'.", "tours": tours})

        elif p == "/api/speaker":
            # NOT gated by the controller lock, and that is deliberate. It is in the same class as
            # STOP: silencing the robot does not move it, and anybody standing next to a machine that
            # is talking at them should be able to turn it off without asking who is holding the
            # controls. Everything that DRIVES needs the lock. This does not.
            #
            # Two ways to call it: {"vol": 0-100} sets an exact level (the slider), {"on": true/false}
            # toggles (the button). The slider is why the robot is no longer all-or-nothing -- quiet
            # but audible, for testing next to people.
            if "vol" in d:
                ok, msg = set_volume(d.get("vol"))
            else:
                ok, msg = set_speaker(bool(d.get("on")))
            self._json({"ok": ok, "message": msg, "speaker": speaker_on(),
                        "volume": speaker_volume()})

        elif p == "/api/explanations":
            # Describe each place on arrival, or just announce it. Not gated -- it changes what the
            # robot says, not what it does, and anyone should be able to quiet it.
            ok, msg = set_explanations(bool(d.get("on")))
            self._json({"ok": ok, "message": msg, "explanations": explanations_on()})

        elif p == "/api/talk":
            text = (d.get("text") or "").strip()
            if not text:
                ok, msg = False, "Say something first."
            elif _busy() and _task["what"] in DRIVING_TASKS and CAN_CHAT_WHILE_DRIVING:
                # Walking someone somewhere, and they ask a question. Answer it -- without
                # stopping, and without being able to drive off (see _answer_while_driving).
                ok, msg = _answer_while_driving(text)
            else:
                ok, msg = _run_task("talk", BRAIN + ["--stay", text])
            self._json({"ok": ok, "message": msg})

        elif p == "/api/face/enroll":
            name = (d.get("name") or "").strip()
            if not HAS_FACES:
                self._json({"ok": False, "message": "Face recognition isn't set up on this robot."})
            elif not name:
                self._json({"ok": False, "message": "Type your name first."})
            else:
                # Blocks ~5 s (six shots, spaced). The phone shows a progress dot per shot; the
                # server just does the work and answers once. ThreadingHTTPServer means this
                # request holds up nobody else -- including STOP.
                ok, msg = FACECAM.enroll(name)
                self._json({"ok": ok, "message": msg,
                            "names": sorted(face_lib.load_faces()) if ok else None})

        elif p == "/api/face/enroll_photos":
            name = (d.get("name") or "").strip()
            images = d.get("images") or []
            if not HAS_FACES:
                self._json({"ok": False, "message": "Face recognition isn't set up on this robot."})
            elif not name:
                self._json({"ok": False, "message": "Type the name first."})
            elif not images:
                self._json({"ok": False, "message": "Pick some photos first."})
            else:
                ok, msg = FACECAM.enroll_photos(name, images)
                self._json({"ok": ok, "message": msg,
                            "names": sorted(face_lib.load_faces()) if ok else None})

        elif p == "/api/face/cooldown":
            # Set how long before the robot re-greets the same person. The phone sends seconds.
            try:
                secs = int(float(d.get("seconds")))
            except (TypeError, ValueError):
                self._json({"ok": False, "message": "Bad cooldown value."})
                return
            if secs not in COOLDOWN_CHOICES:
                self._json({"ok": False, "message": "Pick one of the offered times."})
                return
            set_greeting_cooldown(secs)
            self._json({"ok": True, "seconds": secs})

        elif p == "/api/face/greeting":
            if not HAS_FACES:
                self._json({"ok": False, "message": "Face recognition isn't set up on this robot."})
                return
            on = bool(d.get("on"))
            _greeter["want"] = on
            set_greeting_flag(on)              # survive a reboot: see GREETING_ON_FLAG
            if on:
                if not face_lib.load_faces():
                    _greeter["want"] = False
                    set_greeting_flag(False)
                    self._json({"ok": False, "greeting": False,
                                "message": "Nobody is enrolled yet — add someone first."})
                    return
                # If the mirror has the camera right now, do not fight it: the idle watcher will
                # start the greeter the moment it is free. Saying "on" is enough.
                started = greeter_start() if FACECAM.cap is None else True
                self._json({"ok": started, "greeting": started,
                            "message": "I'll say hello to people I know." if started
                                       else "Could not start the greeter."})
            else:
                greeter_stop()
                self._json({"ok": True, "greeting": False, "message": "I'll keep quiet."})

        elif p == "/api/face/forget_learned":
            # The undo for learning-as-it-goes. If a profile ever feels off, this drops everything
            # the robot worked out for itself and leaves exactly what a human taught it. The
            # enrolled set is untouched by design -- that is what makes this a safe button.
            name = (d.get("name") or "").strip()
            if not HAS_FACES:
                self._json({"ok": False, "message": "Face recognition isn't set up on this robot."})
                return
            db = face_lib.load_db()
            if name not in db:
                self._json({"ok": False, "message": f"I don't know anyone called {name}."})
                return
            n = len(db[name]["learned"])
            db[name]["learned"] = []
            face_lib.save_db(db)
            self._json({"ok": True,
                        "message": f"Dropped {n} thing(s) I'd worked out about {name}. "
                                   f"What you taught me is untouched."})

        elif p == "/api/face/remove":
            name = (d.get("name") or "").strip()
            if not HAS_FACES:
                self._json({"ok": False, "message": "Face recognition isn't set up on this robot."})
                return
            db = face_lib.load_db()
            if name not in db:
                self._json({"ok": False, "message": f"I don't know anyone called {name}."})
                return
            del db[name]
            face_lib.save_db(db)
            self._json({"ok": True, "message": f"Forgot {name}.", "names": sorted(db)})

        elif p == "/api/add_point":
            name = (d.get("name") or "").strip()
            if not name:
                self._json({"ok": False, "message": "Type a name."})
                return
            if not HAS_ROS:
                if name not in _preview_places:
                    _preview_places.append(name)
                self._json({"ok": True, "message": f"(preview) saved {name}"})
                return
            ok, msg = run_waypoint(["save", name])
            self._json({"ok": ok, "message": msg})

        elif p == "/api/remove_point":
            name = (d.get("name") or "").strip()
            if not HAS_ROS:
                if name in _preview_places:
                    _preview_places.remove(name)
                self._json({"ok": True, "message": f"(preview) removed {name}"})
                return
            ok, msg = run_waypoint(["del", name]) if name else (False, "No name.")
            self._json({"ok": ok, "message": msg})

        else:
            self._json({"error": "not found"}, 404)


def main():
    httpd = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()

    if not HAS_ROS:
        print(f"PREVIEW (no ROS) at  http://localhost:{PORT}   -- open it in a browser")
        print("Buttons answer in demo mode (move nothing real). Ctrl+C to quit.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        return

    rclpy.init()
    node = rclpy.create_node("robot_web")
    pub = node.create_publisher(Twist, CMD_VEL_TOPIC, 10)
    if HAS_ARM:                        # free arm moves publish straight to the servo bus
        _ros["arm_pub"] = node.create_publisher(ServosPosition, "servo_controller", 1)
    _ros["node"] = node
    _ros["goal_pub"] = node.create_publisher(PoseStamped, "/goal_pose", 1)   # tap-to-go
    _ros["initialpose_pub"] = node.create_publisher(                          # tap-to-localize
        PoseWithCovarianceStamped, "/initialpose", 1)
    node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", _on_amcl, 10)
    if HAS_HEALTH:                     # live health: lidar + battery + the depth-camera path
        from rclpy.qos import qos_profile_sensor_data
        node.create_subscription(LaserScan, "/scan", _on_scan, qos_profile_sensor_data)
        node.create_subscription(UInt16, "/ros_robot_controller/battery", _on_batt, 10)
        node.create_subscription(LaserScan, CAM_SCAN_TOPIC, _on_camera_scan,
                                 qos_profile_sensor_data)
        if CAM_WATCHDOG:
            threading.Thread(target=_camera_watchdog, daemon=True).start()
            print("[camera-watchdog] on: a trip stops if the depth camera goes quiet", flush=True)

    # Drain subscriptions (/scan ~26 Hz, amcl, battery) in a DEDICATED executor thread
    # so callbacks stay real-time. The old spin_once() in the loop below ran at ~10 Hz
    # and fell behind, making the lidar/localization dots look permanently stale.
    # cmd_vel is still published from the main loop (publishing needs no executor).
    from rclpy.executors import SingleThreadedExecutor
    _executor = SingleThreadedExecutor()
    _executor.add_node(node)

    def _spin_exec():
        try:
            _executor.spin()
        except Exception:
            pass          # ExternalShutdownException on Ctrl+C -> clean exit
    threading.Thread(target=_spin_exec, daemon=True).start()

    print(f"Wall-E web app on  http://0.0.0.0:{PORT}   (open from a phone on the same WiFi)")
    print(f"Manual drive -> '{CMD_VEL_TOPIC}'.  Ctrl+C to stop.")

    twist = Twist()
    stop_burst = 0
    was_active = False
    try:
        while rclpy.ok():
            with _lock:
                active = time.monotonic() < _drive["until"]
                lin = _drive["lin"] if active else 0.0
                ang = _drive["ang"] if active else 0.0
            if active:
                twist.linear.x = lin
                twist.angular.z = ang
                pub.publish(twist)
                was_active = True
            else:
                if was_active:
                    stop_burst = 3
                    was_active = False
                if stop_burst > 0:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    pub.publish(twist)
                    stop_burst -= 1
            time.sleep(0.1)     # subscriptions run in the executor thread, not here
    except KeyboardInterrupt:
        pass
    except Exception:
        # On Ctrl+C rclpy tears down the context asynchronously, so an in-flight
        # publish can raise RCLError ("context is invalid"). That's a clean exit,
        # not a crash -- swallow it instead of dumping a scary traceback.
        pass
    finally:
        try:
            if rclpy.ok():
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                pub.publish(twist)     # best-effort final stop
        except Exception:
            pass
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
