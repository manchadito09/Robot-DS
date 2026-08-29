#!/usr/bin/env python3
# guide.py - the guide node: LEADS the visitor to a destination and RETURNS to base.
#
# This is the REAL version of go_to(x,y): instead of moving a fake base, it asks
# Nav2 to navigate (Nav2 plans the route and avoids obstacles). On top of that it:
#   - NARRATES the trip using Nav2's live distance_remaining feedback,
#   - ANNOUNCES arrival,
#   - RETURNS to base on its own when done.
# That's the guide loop of the demo (confirm -> lead narrating -> announce ->
# return to base), all chained together.
#
# Tested on rosita against the Nav2 mini-sim: our node sends the goal and Nav2
# drives the robot to it. It runs THE SAME on the real robot; only the
# coordinates change (real floor-4 map) and Nav2 talks to the JetRover's motors
# instead of a simulated robot.
#
# Usage (with Nav2 running):
#     python3 guide.py kitchen        # lead to the kitchen and return to base
#     python3 guide.py kitchen --solo # only go, no return (for debugging)
import os
import sys
import math
import random
import wave
import yaml
import fcntl
import shutil
import hashlib
import contextlib
import subprocess
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
try:  # installed package: ros2 run (the real robot, Humble)
    from robot_ds_behavior.pois import POIS, MAP_OFFSET, MAP_YAW
except ImportError:  # loose scripts: python3 guide.py (the sim on rosita)
    from pois import POIS, MAP_OFFSET, MAP_YAW  # places + world->map calibration (config, not code)

# ---- Destinations: the named waypoints you save while mapping ---------------
# waypoint_tool.py saves points to this file, ALREADY in the Nav2 "map" frame
# (from TF map->base_footprint) with their own yaw. So this file is the single
# source of truth for where the robot can go: every point you save while mapping
# instantly becomes a voice/talk destination -- no code change. If it's missing
# (e.g. the sim on rosita) we fall back to POIS + the world->map calibration.
WAYPOINTS_YAML = os.path.expanduser("~/ros2_ws/src/slam/maps/map_01.waypoints.yaml")

# Preferred "home" names; the first one that exists is where lead() returns to.
BASE_CANDIDATES = ("inicio", "reception", "base", "home", "start", "center")


def _yaw_to_quat(yaw):
    """Planar yaw (rad) -> (z, w) quaternion."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def load_waypoints(path=WAYPOINTS_YAML):
    """Named waypoints (map frame) from waypoint_tool.py -> {name: (x, y, yaw)}.
    Empty dict if the file doesn't exist yet (nothing mapped/saved)."""
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
    except (FileNotFoundError, OSError):
        return {}
    out = {}
    for name, p in (data.get("waypoints") or {}).items():
        try:
            out[name] = (float(p["x"]), float(p["y"]), float(p.get("yaw", 0.0)))
        except (TypeError, KeyError, ValueError):
            continue
    return out


def _sim_places():
    """Fallback (sim on rosita): POIS in world frame -> map frame via calibration."""
    c, s = math.cos(MAP_YAW), math.sin(MAP_YAW)
    return {n: (c * wx - s * wy + MAP_OFFSET[0],
                s * wx + c * wy + MAP_OFFSET[1], 0.0)
            for n, (wx, wy) in POIS.items()}


def load_places():
    """All destinations {name: (x, y, yaw)} in the Nav2 map frame.
    Real saved waypoints if present (the robot), else the sim POIS."""
    return load_waypoints() or _sim_places()


def base_name(places=None):
    """Return-to-base is DISABLED: the robot stays wherever it's sent (no home base).
    Every mode (take-me / messenger / tour) now ends at the destination instead of
    driving back. Return a base name here again to re-enable the auto-return."""
    return None


# The arm carries the depth camera. Before any trip we tilt it to the SCAN pose so the
# camera sees the near floor and Nav2 can avoid low obstacles (chair legs) the lidar
# misses. It stays there after arrival (we never move it back). FIRE-AND-FORGET (Popen,
# not run): moving the arm takes several seconds and we must NOT block the trip from
# starting -- the arm tilts DOWN in parallel while Nav2 plans and the robot sets off.
# Best-effort: a failure here must never stop a trip.
ARM_TOOL = os.path.expanduser("~/ros2_ws/arm_gesture.py")


def set_scan_pose():
    try:
        subprocess.Popen(["python3", ARM_TOOL, "scan"],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass


# ---- Voice out (TTS): Piper (neural, natural) with espeak as fallback --------
# The speaker is the USB audio dongle, addressed BY CARD NAME, never by number. It used to say
# plughw:1,0, and card 1 WAS the speaker -- until the 6-mic array was plugged back in, took card 1
# for itself, and pushed the speaker down to card 0. The robot then "spoke" into a microphone and
# went silent, with nothing in any log to say why. Card numbers depend on what is plugged in and in
# what order; card names do not. Override with ROBOT_SPEAKER.
SPEAKER_DEV = os.environ.get("ROBOT_SPEAKER", "plughw:CARD=Device,DEV=0")
_ESPEAK = shutil.which("espeak") or shutil.which("espeak-ng")
# Piper: natural neural voice. Binary + model live next to the voice code.
# Swap the model (or ROBOT_PIPER_MODEL) to change the voice.
PIPER_BIN = os.path.expanduser("~/Robot-DS/voice_prototype/piper/piper/piper")
PIPER_MODEL = os.path.expanduser(os.environ.get(
    "ROBOT_PIPER_MODEL",
    "~/Robot-DS/voice_prototype/piper/models/en_US-ryan-medium.onnx"))
PIPER_RATE = "22050"   # medium Piper models are 22050 Hz

# ---- The robot has ONE mouth: everyone who speaks takes this lock ------------
# guide, brain and speak.py (the web app's camera watchdog) are SEPARATE PROCESSES that all end up
# in aplay on the same exclusive ALSA device. Two at once do not mix: the second one is refused and
# the line is lost, with nothing in any log. A threading lock cannot see across processes, so this
# is a FILE lock. Held for generate+play, so a line is never cut in half.
AUDIO_LOCK = "/tmp/robot_ds_audio.lock"
AUDIO_LOCK_WAIT = 15.0   # then speak anyway: a wedged aplay must not mute the robot for good


class _Mouth:
    """The audio lock. `with _Mouth(optional) as mine:` -- ALWAYS check `mine`.

    optional=False (departure, arrival, alerts): wait your turn, then speak regardless.
    optional=True  (mid-trip chatter): if the robot is already talking, DROP the line. Queueing it
                   would play "we're about halfway there" after we had already arrived.
    """

    def __init__(self, optional=False):
        self.optional = optional
        self.f = None

    def __enter__(self):
        try:
            self.f = open(AUDIO_LOCK, "w")
        except Exception:
            return True          # no lock file? speaking matters more than serialising
        deadline = time.time() + (0.0 if self.optional else AUDIO_LOCK_WAIT)
        while True:
            try:
                fcntl.flock(self.f, fcntl.LOCK_EX | fcntl.LOCK_NB)
                return True
            except OSError:
                if time.time() >= deadline:
                    if self.optional:
                        self.f.close()
                        self.f = None
                        return False
                    return True   # waited long enough -- an important line always gets said
                time.sleep(0.05)

    def __exit__(self, *exc):
        if self.f is not None:
            try:
                fcntl.flock(self.f, fcntl.LOCK_UN)
            except Exception:
                pass
            self.f.close()
            self.f = None
        return False


# ---- Pre-baked lines: Piper must not run while Nav2 is driving ---------------
# Synthesising eats a core for ~a second. Doing that mid-trip is exactly how the robot ends up
# stuttering (the 334%-CPU lesson: everything went slow because one node starved the Jetson).
# The mid-trip lines are FIXED, so they are baked to WAV once, standing still, and driving costs
# nothing but an aplay.
CACHE_DIR = os.path.expanduser("~/.cache/robot_ds/narration")

# WHAT THE ROBOT SAYS ON THE WAY.
#
# It used to fill the walk with "We're about halfway there." That is not conversation, it is a
# progress bar read aloud -- the visitor already knows how far they have walked. The middle of the
# trip is the one moment you have someone's attention with nothing else going on. Use it: tell them
# something about the company, or make them smile.
#
# These lines are FIXED strings on purpose. They get baked to WAV once, standing still, and cost
# nothing but an aplay while driving -- synthesising mid-trip eats a core for a second and is exactly
# how the robot ends up stuttering. So: a fixed pool, picked from at random, not generated per trip.
#
# The pool is the company facts from office_knowledge.yaml (the same ones the voice brain answers
# questions with -- edit them there, once) plus a few lines of the robot's own. Add your own; keep
# them SHORT: every word is another second of a visitor standing in a corridor.
# There is no "almost there" line any more, and no "we are about halfway".
#
# Both were a progress bar read aloud. The visitor can SEE how far they have walked; being told is
# not conversation, it is filler, and it spends the one moment you have their attention on nothing.
# The walk is now filled with things worth hearing instead: what the company does, a joke at the
# robot's own expense, or an invitation to ask it something.
# No glass joke. "I cannot see glass, so I trust the map" is true, and it is the single worst thing
# this robot could say in THIS office: there is a glass-walled room on the route, the lidar really
# does shoot straight through it, and it really did drive into it until the map was fixed. Handing a
# visitor that sentence, in that corridor, is handing them the one question you do not want asked.
# The three originals stay here as a FALLBACK -- if office_knowledge.yaml is missing or has no
# `jokes`, the robot still has a sense of humour. The real list comes from the YAML now (see
# CHAT_LINES below), so the humour can be tuned without a colcon build.
ROBOT_LINES = (
    "I'm the guide robot. I know this floor better than anyone, and I never take the lift.",
    "If I stop suddenly, it is not shyness. Someone has left a chair in my way.",
    "I do about a hundred laps of this floor a week. Still no idea what is in the fridge.",
)


def _office_lines(key):
    """A list of lines from office_knowledge.yaml's `office` section. Empty if it is missing.

    `about` = the company facts, the SAME ones the voice brain answers questions with, so they are
              written and vetted in exactly one place.
    `chat`  = the robot's own invitations to ask it something. Kept OUT of `about` on purpose: those
              are facts Claude quotes back to visitors, and "ask me about DSSI" is not a fact.
    """
    try:
        with open(os.path.expanduser("~/ros2_ws/office_knowledge.yaml")) as f:
            kb = yaml.safe_load(f) or {}
        return tuple(str(a).strip() for a in ((kb.get("office") or {}).get(key) or [])
                     if str(a).strip())
    except (FileNotFoundError, OSError, yaml.YAMLError):
        return ()


# `walk`, NOT `about`. `about` is Claude's notes -- third person, so it can quote them without
# inventing. Said out loud they are somebody else's sentences: "It works with tens of thousands of
# care buildings across America." It what? Halfway down a corridor, with no subject. `walk` is the
# same facts written to be SPOKEN, in the robot's own voice. (If `walk` is missing we fall back to
# `about` rather than walking a visitor down a hallway in silence.)
CHAT_LINES = ((_office_lines("walk") or _office_lines("about"))
              + (_office_lines("jokes") or ROBOT_LINES)      # YAML jokes, or the built-in fallback
              + _office_lines("chat"))

# What the robot says when someone asks to be taken SOMEWHERE ELSE while it is already walking them
# somewhere. It lives here, next to the other fixed lines, for one reason: everything in
# NARRATE_LINES gets baked to a WAV up front, and this is a line the robot says WHILE DRIVING.
# Left unbaked, Piper would synthesise it live, mid-trip -- a core for a second, and Nav2 stutters.
# brain.py imports it from here so there is one copy of the words and one WAV for them.
DEFER_LINE = "Let me get you there first. Ask me again when we arrive and I will take you."

NARRATE_LINES = CHAT_LINES + (DEFER_LINE,)    # everything that gets baked to WAV up front

MIN_TRIP_M = 4.0    # below this, don't narrate: there is no "during" to a walk of two steps
FIRST_M = 3.0       # metres covered before the first line -- let the departure line land first
GAP_M = 6.0         # metres between lines, so it is a companion and not a podcast
QUIET_M = 2.0       # closer than this to the goal: silence. The arrival line owns the last seconds.

# The pause between "we've arrived" and the description of the place. A person breathes here.
ARRIVE_PAUSE_S = 1.0
# When this file exists, the robot announces arrivals but does NOT describe the place -- for
# visitors who already know the floor. The web toggles it (/api/explanations).
EXPLAIN_OFF_FLAG = os.path.expanduser("~/.cache/robot_ds/explanations_off")

# WILL THIS LINE FINISH BEFORE WE ARRIVE?
#
# This is the whole reason the narration is not just "say something every N metres". The lines are
# not the same length: the shortest is 1.3 s, the longest 9.7 s. At 0.4 m/s the long one needs four
# metres of corridor to get through. Start it with three metres left and the robot is still talking
# about employee ownership as it pulls up at the kitchen -- and the arrival line, which is NOT
# droppable, then queues up behind it and lands in an awkward silence several seconds late. That
# pause is exactly what was just designed out of the arrival.
#
# So we ask, before opening our mouth: does this one FIT? We know each line's length exactly -- it
# is a baked WAV, we can read its duration. And we use the robot's TOP speed (0.4 m/s) to convert it
# into metres, which is the safe direction to be wrong in: the robot is usually slower than that, so
# it has longer than we assumed, not less.
TOP_SPEED = 0.40    # m/s (nav2_controller_teb.yaml: max_vel_x)
FIT_MARGIN_M = 1.0  # and leave a metre of quiet at the end anyway


def _wav_path(text):
    key = hashlib.md5((PIPER_MODEL + "|" + text).encode("utf-8")).hexdigest()[:16]
    return os.path.join(CACHE_DIR, key + ".wav")


def _cached_wav(text):
    """The baked WAV for this line, or None. Never synthesises -- see _prebake."""
    p = _wav_path(text)
    return p if os.path.exists(p) else None


_secs = {}          # text -> seconds of audio. Read once from the WAV; they never change.


def _line_secs(text):
    """How long this line takes to say, in seconds. None if it has not been baked.

    Read from the baked WAV itself rather than guessed from the word count: the lines run from 1.3 s
    to 9.7 s and a word-count guess is wrong by enough to matter when the question is "will this
    finish before we arrive".
    """
    if text in _secs:
        return _secs[text]
    p = _cached_wav(text)
    if not p:
        return None                     # not baked -> we cannot time it, so we will not start it
    try:
        with contextlib.closing(wave.open(p)) as w:
            _secs[text] = w.getnframes() / float(w.getframerate())
    except Exception:
        _secs[text] = None
    return _secs[text]


def _fits(text, metres_left):
    """Can this line be finished before the robot arrives?"""
    s = _line_secs(text)
    if s is None:
        return False                    # unbaked: saying it would mean synthesising WHILE DRIVING,
                                        # which eats a core and makes Nav2 stutter. Skip it.
    return metres_left >= s * TOP_SPEED + FIT_MARGIN_M


def _prebake(lines):
    """Bake the fixed lines to WAV once. Safe to call every start-up: it skips what it has."""
    if not (os.path.exists(PIPER_BIN) and os.path.exists(PIPER_MODEL)):
        return
    try:
        os.makedirs(CACHE_DIR, exist_ok=True)
    except Exception:
        return
    penv = dict(os.environ)
    penv["LD_LIBRARY_PATH"] = os.path.dirname(PIPER_BIN)   # piper's own libs -- see _say_piper
    for text in lines:
        out = _wav_path(text)
        if os.path.exists(out):
            continue
        try:
            tmp = out + ".part"
            subprocess.run([PIPER_BIN, "--model", PIPER_MODEL, "--output_file", tmp],
                           input=text.encode("utf-8"), stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL, env=penv)
            if os.path.getsize(tmp) > 44:      # bigger than a bare WAV header
                os.replace(tmp, out)           # atomic: a half-written WAV is never played
        except Exception:
            pass


def _play_wav(path):
    try:
        ap = subprocess.run(["aplay", "-q", "-D", SPEAKER_DEV, path],
                            stderr=subprocess.DEVNULL)
        return ap.returncode == 0
    except Exception:
        return False


def _say_piper(text):
    """Speak with Piper -> aplay (raw pipe). Returns True on success."""
    if not (os.path.exists(PIPER_BIN) and os.path.exists(PIPER_MODEL)):
        return False
    try:
        # Piper must use ITS OWN bundled libs (libonnxruntime, libespeak-ng,
        # libpiper_phonemize) -- NOT the ROS/third-party ones on the service's huge
        # LD_LIBRARY_PATH, which cause a "symbol lookup error" (piper exits 127,
        # produces 0 audio -> the robot goes silent ONLY under the systemd service).
        # Pointing LD_LIBRARY_PATH at piper's own dir makes it deterministic.
        penv = dict(os.environ)
        penv["LD_LIBRARY_PATH"] = os.path.dirname(PIPER_BIN)
        _t = time.time()
        # STREAM piper -> aplay, rather than generate-the-whole-clip-then-play. The old code did
        # `capture_output=True`, which waits for piper to FINISH and buffers every byte before aplay
        # even starts -- so a ~1.7 s synthesis was 1.7 s of dead air before the first word. Piped,
        # aplay plays each chunk the instant piper emits it, and the wait for the first word drops to
        # piper's time-to-first-chunk. Same audio, it just starts sooner.
        gp = subprocess.Popen([PIPER_BIN, "--model", PIPER_MODEL, "--output_raw"],
                              stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                              stderr=subprocess.DEVNULL, env=penv)
        ap = subprocess.Popen(["aplay", "-q", "-r", PIPER_RATE, "-f", "S16_LE",
                               "-t", "raw", "-D", SPEAKER_DEV],
                              stdin=gp.stdout, stderr=subprocess.DEVNULL)
        gp.stdout.close()                      # aplay owns the read end now; we must drop our copy
        try:
            gp.stdin.write(text.encode("utf-8"))
            gp.stdin.close()                   # piper starts synthesising as soon as it has the text
        except BrokenPipeError:
            pass
        ap.wait()
        gp.wait()
        if os.environ.get("ROBOT_TIMING"):
            print(f"[t]   piper+aplay streamed {time.time()-_t:.2f}s (was generate-then-play)",
                  flush=True)
        return ap.returncode == 0
    except Exception:
        return False


def _say_espeak(text):
    """Fallback voice if Piper isn't available."""
    if not _ESPEAK:
        return
    try:
        gen = subprocess.Popen([_ESPEAK, "-v", "en-us", "-s", "150", "--stdout", text],
                               stdout=subprocess.PIPE)
        subprocess.run(["aplay", "-q", "-D", SPEAKER_DEV], stdin=gen.stdout,
                       stderr=subprocess.DEVNULL)
        gen.stdout.close()
        gen.wait()
    except Exception:
        pass


def say(text, optional=False):
    # Narrate to the console AND out loud (Piper; espeak as a fallback).
    #
    # Timed, split into GENERATE and PLAY, because "the robot took 10 s to say one line" needs to be
    # attributed before it can be fixed: Piper synthesising the whole clip before a single word comes
    # out is a latency bug we can attack (shorter replies, streaming); the clip's own playback length
    # is just how long the sentence takes to say, and no amount of engineering shortens that. Only
    # the words do. ROBOT_TIMING=1 to see it.
    #
    # optional=True -> a line worth saying only if it is said ON TIME (mid-trip chatter). If the
    # robot is already talking, drop it. Everything else waits its turn. Returns True if spoken.
    print(f"[robot] {text}", flush=True)
    _t = time.time()
    with _Mouth(optional) as mine:
        if not mine:
            print("[robot] (dropped: already speaking)", flush=True)
            return False
        wav = _cached_wav(text)              # a pre-baked line: no Piper, just play it
        if wav:
            ok = _play_wav(wav)
        else:
            ok = _say_piper(text)
        if not ok:
            _say_espeak(text)
        if os.environ.get("ROBOT_TIMING"):
            n = len(text.split())
            print(f"[t] said {n} words in {time.time()-_t:.2f}s "
                  f"({'PRE-BAKED wav' if wav else 'generate'} + play; "
                  f"~0.35 s/word is normal speech)", flush=True)
        # Small gap so PulseAudio drains the buffer before the next phrase -- without it,
        # back-to-back lines (e.g. "I've reached X" + the messenger message) clip/swallow
        # the second one and the message never gets heard. Inside the lock: the gap is part of
        # the line, or the next speaker starts talking into the tail of this one.
        time.sleep(0.4)
    return True


def _describe(name):
    """Rich arrival narration for a place, from the office knowledge base -- the
    SAME one the voice brain uses, so web-driven trips (Take me / Tour) narrate as
    well as voice does. Imported lazily to avoid a circular import (brain imports
    this module at load time). Returns a spoken line, or None if unavailable
    (then go() just skips the extra line)."""
    try:
        try:
            from robot_ds_behavior.brain import describe
        except ImportError:
            from brain import describe
        return describe(name)
    except Exception:
        return None


class Guide(Node):
    def __init__(self):
        super().__init__("guide")
        # Action client = how we send goals to Nav2 and track its progress.
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._start_dist = None      # longest distance-remaining seen: the trip's true length
        self._said = set()           # lines already said THIS trip -- never repeat one
        self._next_line_at = FIRST_M # metres to cover before the next line is due
        # Bake the fixed mid-trip lines while the robot is still standing still.
        threading.Thread(target=_prebake, args=(NARRATE_LINES,), daemon=True).start()

    def _on_feedback(self, fb):
        # Narrate the trip from Nav2's live distance_remaining, so the visitor is not walked down a
        # corridor by a robot that said one line and then went silent for a minute.
        #
        # This runs on the ROS executor thread. It must NEVER block -- say() generates and plays
        # audio for SECONDS, and blocking here stalls the action client and makes Nav2 stutter. So
        # every line goes out on its own thread, and as `optional`: if the robot is already talking,
        # the line is dropped rather than queued behind it.
        try:
            d = float(fb.feedback.distance_remaining)
        except Exception:
            return
        if d <= 0.0:
            return   # Nav2 reports 0.0 while it has no path yet. That is not "we have arrived".
        if self._start_dist is None or d > self._start_dist:
            # The trip length is the LONGEST distance seen, not the first sample. When Nav2 replans
            # around an obstacle -- which the demo does ON PURPOSE -- the remaining distance JUMPS
            # UP. Anchored to the first sample, the robot would think it had gone backwards and the
            # lines would fire late, or never at all.
            self._start_dist = d
        if self._start_dist < MIN_TRIP_M:
            return   # too short to have a "during": a walk of two steps needs no companion
        if d < QUIET_M:
            return   # arrival is seconds away -- shut up and let the arrival line have the floor

        # Keep talking, all the way down the corridor -- but only ever with something worth hearing,
        # and only if it will be FINISHED before we arrive.
        covered = self._start_dist - d
        if covered < self._next_line_at:
            return
        # Never the same line twice on one trip, and never one that will still be running when we
        # pull up (see _fits). On a long corridor that is two or three of them; on a short hop, one;
        # in the last few metres, none -- and that silence is deliberate, not a bug.
        pool = [t for t in CHAT_LINES if t not in self._said and _fits(t, d)]
        if not pool:
            return
        line = random.choice(pool)
        self._said.add(line)
        self._next_line_at = covered + GAP_M
        self._narrate(line)

    def _narrate(self, text):
        # Off the ROS thread, and droppable. See _on_feedback.
        threading.Thread(target=say, args=(text,), kwargs={"optional": True},
                         daemon=True).start()

    def go(self, name, follow=True, arrive_say=None):
        # One trip: send the goal to Nav2 and narrate until arrival.
        # follow=True  -> "follow me" phrasing (leading a visitor)
        # follow=False -> "on my way" phrasing (summon / messenger)
        # arrive_say   -> an extra line spoken once we arrive (messenger message)
        # Reloaded each trip, so points saved while mapping are reachable at once.
        places = load_places()
        if name not in places:
            self.get_logger().error(f"I don't know '{name}'. Options: {list(places)}")
            return False
        x, y, yaw = places[name]   # already in the Nav2 map frame
        self.get_logger().info("Waiting for Nav2...")
        if not self.nav.wait_for_server(timeout_sec=8.0):
            # Don't hang forever: Nav2 isn't up/active or isn't reachable.
            say("Navigation isn't ready. Please launch it and set my position in RViz.")
            self.get_logger().error(
                "navigate_to_pose no disponible (¿Nav2 lanzado y activo?)")
            return False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        qz, qw = _yaw_to_quat(yaw)   # arrive facing the way the point was saved
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        # Reset the narration for this trip. The Guide node survives several trips in a tour, so
        # `_said` MUST be emptied here: left full, every stop after the first would be walked in
        # complete silence, because every line would already count as spoken.
        self._start_dist = None
        self._said = set()
        self._next_line_at = FIRST_M
        phrase = (f"Taking you to {name}. Follow me, this way." if follow
                  else f"On my way to {name}.")
        # Speak the departure line IN PARALLEL with sending the goal, so the robot sets
        # off as soon as Nav2 accepts (~1 s) instead of standing still for the whole
        # ~8 s phrase. The line still gets printed (for the web chat) from the thread.
        threading.Thread(target=say, args=(phrase,), daemon=True).start()
        gh = self.nav.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, gh)
        handle = gh.result()
        if not handle.accepted:
            self.get_logger().error("Nav2 rejected the goal")
            return False

        # arrive_say may be a CALLABLE, and that is the point. The arrival narration comes from
        # Claude, and it used to be fetched as an argument -- i.e. BEFORE go() was even called -- so
        # the robot stood still for the whole round trip to Claude (2 s warm, ~10 s cold) before it
        # so much as set off. That is the "why does it take so long to start" everyone noticed. It is
        # not needed until we ARRIVE, a minute later. Ask for it now, in a thread, and let Claude
        # think while the robot drives.
        narration = {}
        fetch = None
        if callable(arrive_say):
            def fetch_narration():
                try:
                    narration["text"] = arrive_say()
                except Exception:
                    pass          # no narration is fine; a stalled trip is not
            fetch = threading.Thread(target=fetch_narration, daemon=True)
            fetch.start()

        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        status = res.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            # Nav2 did not get there (goal outside the map, no path, etc.).
            # Do NOT announce a false arrival.
            say(f"I couldn't reach {name}.")
            self.get_logger().warn(f"Nav2 finished with status {status} (not SUCCEEDED)")
            return False
        arrived = f"We've arrived at {name}." if follow else f"I've reached {name}."
        if fetch is not None:
            fetch.join(timeout=8.0)          # Claude has had the whole trip to write this
            line = narration.get("text")
        else:
            line = arrive_say

        # EXPLANATIONS CAN BE TURNED OFF. When the visitor is a Direct Supply employee who knows the
        # floor, "and here is the kitchen, where people get coffee" is noise. The web has a switch
        # (see ~/.cache/robot_ds/explanations_off); when it is set we announce the arrival and stop.
        #
        # But ONLY the auto-DESCRIPTION is noise -- and that is exactly the callable arrive_say (the
        # tour/lead lambda that calls _describe). A messenger MESSAGE is a plain string handed in on
        # purpose; it is the whole point of the trip and must be delivered whatever this switch says.
        # `fetch is not None` is precisely "arrive_say was a description", so suppress only that.
        if os.path.exists(EXPLAIN_OFF_FLAG) and fetch is not None:
            line = None

        # A BEAT BEFORE THE DESCRIPTION. Not the old bug -- that was Piper buffering, an unpredictable
        # dead silence while it synthesised. This is a deliberate ~1 s pause between announcing the
        # place and describing it, because a person says "here we are" and then takes a breath before
        # launching into the tour-guide part. Said in one rushed sentence it sounds like a recording.
        # Only when there IS a description to lead into; a bare arrival stays a bare arrival.
        if line:
            say(arrived)
            time.sleep(ARRIVE_PAUSE_S)
            say(line)
        else:
            say(arrived)
        return True

    def come(self, name):
        # "Come here": just go to the point and stay (no leading, no return).
        return self.go(name, follow=False)

    def deliver(self, name, phrase):
        # "Messenger": go to the point, say a custom phrase, then return to base.
        if not self.go(name, follow=False, arrive_say=phrase):
            return False
        base = base_name()
        if base and name != base:
            say(f"Message delivered. Heading back to {base}.")
            self.go(base, follow=False)
        return True

    def tour(self, names):
        # "Tour": visit each point in order, narrating what it is, then return to base.
        for n in names:
            self.go(n, follow=True, arrive_say=(lambda nm=n: _describe(nm)))
        base = base_name()
        if base and (not names or names[-1] != base):
            say("That's the end of the tour. Heading back.")
            self.go(base, follow=False)
        say("Tour complete.")
        return True

    def lead(self, name, return_to_base=True):
        # The COMPLETE guide loop: lead the visitor (narrating the place) and return.
        # lambda, not _describe(name): see go(). Calling it here would ask Claude for the
        # arrival line BEFORE the robot moves, and it would stand there waiting for it.
        if not self.go(name, arrive_say=lambda: _describe(name)):
            return False
        base = base_name()
        if return_to_base and base and name != base:
            say(f"I'll leave you here. Heading back to {base}.")
            self.go(base)
            say("Back at base, ready for the next one.")
        return True


def main():
    # Modes (used by the web app and the command line):
    #   guide <name>                 lead there + return to base   (take me)
    #   guide <name> --solo          go there and stay
    #   guide <name> --come          summon: go there and stay ("on my way")
    #   guide <name> --say "phrase"  messenger: go, say phrase, return to base
    #   guide --tour a b c           tour: visit each narrating, then return
    argv = sys.argv[1:]
    names = [a for a in argv if not a.startswith("-")]
    rclpy.init()
    node = Guide()
    set_scan_pose()               # arm -> scan pose before we move (covers every mode)
    if "--tour" in argv:
        i = argv.index("--tour")
        tour_names = [a for a in argv[i + 1:] if not a.startswith("-")]
        node.tour(tour_names)
    elif "--say" in argv:
        i = argv.index("--say")
        phrase = argv[i + 1] if i + 1 < len(argv) else ""
        before = [a for a in argv[:i] if not a.startswith("-")]
        node.deliver(before[0] if before else "kitchen", phrase)
    elif "--come" in argv:
        node.come(names[0] if names else "kitchen")
    else:
        name = names[0] if names else "kitchen"
        node.lead(name, return_to_base="--solo" not in argv)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
