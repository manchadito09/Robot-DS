#!/usr/bin/env python3
# face_greet.py - watch the forward camera and greet people by name. It speaks; it does not listen.
#
#   python3 ~/ros2_ws/face_greet.py            # run it (Ctrl-C to stop)
#   python3 ~/ros2_ws/face_greet.py --once     # look once, report, exit  (a test, says nothing)
#   python3 ~/ros2_ws/face_greet.py --dry      # run, but print instead of speaking
#
# Two openings, because two kinds of people walk up to a guide robot:
#
#   someone it knows -> "Hi Rodrigo! Tap Talk to me on the phone app if you need anything."
#   anyone else      -> "Hello! I'm Wall-E, the floor guide. Scan the QR code and tap Talk to me."
#
# Both of them point at the phone. A VISITOR is by definition not enrolled, so greeting only the
# staff would mean the one person who actually needs a guide is the one who gets silence.
#
# IT DOES NOT LISTEN. IT USED TO, AND THE MICROPHONE IS WHY IT STOPPED.
#
# The greeter had a full conversation in it: mic open after each hello, Whisper, Claude, a reply.
# The talking half always worked. The listening half never became reliable, and it was the ROOM,
# not the code: the 6-mic array runs its own automatic gain that drifts with how quiet the office
# is (measured: voice sat 1.5x over the noise floor, and Whisper returns '' at that ratio), and
# the array reports VOICE on its own opening transient with nobody in the room (measured 29-jul,
# every single time, at a level of 0.005). Days went into it. The phone does not have this problem
# -- the mic is at the user's mouth, it is not next to the robot's speaker, and the browser gives
# it a clean stream. So the conversation lives in the web app's "Talk to me", where it works, and
# the greeter does the one thing it is good at. This file has no microphone code at all now, on
# purpose: an off switch for a broken feature is still a broken feature to maintain.
#
# It ONLY talks. It never drives, never moves the arm, never touches Nav2. The worst it can do is
# say hello at an odd moment.
#
# THREE THINGS KEEP THIS HONEST:
#
#   CPU      3 fps, not 30, and threads capped in face_lib. A greeter is not worth a core, and
#            this Jetson has been eaten alive once already by a node that span rather than worked.
#   QUIET    the greeting is spoken with --optional, so guide.say() DROPS it if the robot is
#            already talking. A hello must never step on a real answer, or on the narration of
#            a trip someone is being led on.
#   COOLDOWN one greeting per person per COOLDOWN_S. Without it, standing near the robot means
#            being greeted three times a second, which is not charming, it is a fault.
import collections
import os
import random
import subprocess
import sys
import time

import face_lib as fl      # FIRST: it caps the maths threads before cv2 is ever imported
import cv2                 # noqa: E402  -- order matters here, see face_lib

SPEAK = os.path.expanduser("~/ros2_ws/speak.py")
PREBAKE = os.path.expanduser("~/ros2_ws/prebake_lines.py")

FPS = 3.0                  # looks per second
COOLDOWN_DEFAULT_S = 300.0 # 5 min before greeting the SAME person again, if the web hasn't set one
COOLDOWN_FILE = os.path.expanduser("~/.cache/robot_ds/greet_cooldown")  # web writes the seconds here
MIN_FACE_PX = 60           # smaller than this is too far away to recognise reliably
UNSURE_BAND = 0.10         # scoring within this much UNDER the match threshold = "I half know
                           # you". Not enough to use a name; far too much to call them a stranger.
# A VOTE, NOT A STREAK.
#
# First attempt asked for N looks in a ROW agreeing. On a face the robot half-knows the scores
# bounce -- Rodrigo, unsure, Rodrigo, Rodrigo, unsure -- so the streak counter reset forever and
# it took ages to speak, or never spoke at all. A vote over a short window survives the bouncing:
# 3 of the last 5 looks agreeing is a decision; one disagreeing frame is just noise.
VOTE_WINDOW = 5
VOTE_NEEDED = 3
# EVERY GREETING POINTS AT THE PHONE, because the phone is where the robot can actually be talked
# to ("Talk to me" in the web app, reached by the QR code on the robot). A hello that leads
# nowhere is a party trick; a hello that hands someone the thing that works is a receptionist.
#
# Several openings, picked at random, so the robot doesn't say the exact same thing every time it
# sees you. ALL of them are pre-baked per enrolled person (see prebake_lines.py), so every one is
# still instant -- no waiting for Piper. Keep {name} in each; add/remove freely and RE-BAKE:
#   python3 ~/ros2_ws/prebake_lines.py
GREETINGS = [
    "Hi {name}! Tap Talk to me on the phone app if you need anything.",
    "Hey {name}, good to see you! I'm on the app if you need me -- just tap Talk to me.",
    "Hello {name}! Nice to see you again. Ask me anything from the app.",
    "Hi {name}! Scan my QR code and tap Talk to me, and I'll take you anywhere on this floor.",
]


def greeting_for(name):
    """One of the openings, at random, with the name filled in."""
    return random.choice(GREETINGS).format(name=name)

# STRANGERS. The point of this robot is guiding VISITORS -- and a visitor is, by definition, not
# enrolled. Greeting only the people it knows means the office staff get a hello and the one
# person who actually needs a guide gets silence. A visitor also has no idea the app exists, so
# theirs is the one greeting that has to spell the whole thing out.
STRANGER_GREETING = ("Hello! I'm Wall-E, the floor guide. Scan the QR code on me and tap Talk to "
                     "me, and I'll take you where you need to go.")

# ENROLLING A FACE happens in the web app, under "Add me": the person holds the phone, sees what
# the robot sees, and taps. The robot used to ask out loud instead -- "would you like me to
# remember your face?" -- which was lovely consent and a coin-toss of a microphone. The app asks
# the same question of the same person and hears the answer every time.
STRANGER_SAME = 0.5           # "the same face" means this similar. Loose on purpose:
                               # we are not identifying them, only remembering we just spoke.

# How long before greeting the SAME person (known OR unknown) again. The web writes the number of
# seconds into COOLDOWN_FILE; we re-read it only when the file changes, so a slider on the phone
# takes effect within seconds without restarting the greeter. Missing/garbage file -> the default.
_cooldown_cache = {"mtime": None, "val": COOLDOWN_DEFAULT_S}
def cooldown_s():
    try:
        m = os.path.getmtime(COOLDOWN_FILE)
        if m != _cooldown_cache["mtime"]:
            _cooldown_cache["mtime"] = m
            v = float(open(COOLDOWN_FILE).read().strip())
            _cooldown_cache["val"] = v if v >= 0 else COOLDOWN_DEFAULT_S
    except (OSError, ValueError):
        _cooldown_cache["mtime"] = None
        _cooldown_cache["val"] = COOLDOWN_DEFAULT_S
    return _cooldown_cache["val"]
STRANGER_FORGET_S = 600.0      # drop a remembered stranger entirely after this


class RecentStrangers:
    """A few minutes of memory for faces we cannot name.

    A known person has a name to hang a cooldown on. A stranger has nothing -- so without this the
    robot greets the same visitor three times a second for as long as they stand there.

    Kept in RAM and only in RAM. An unknown face is never written to disk: someone who walked past
    a robot has not agreed to anything, and the robot has no business remembering them tomorrow.
    These fade out after STRANGER_FORGET_S and die with the process.
    """

    def __init__(self, rec):
        self.rec = rec
        self.seen = []                       # [(vec, when)]

    def _purge(self, now):
        self.seen = [(v, t) for v, t in self.seen if now - t < STRANGER_FORGET_S]

    def greeted_recently(self, vec):
        now = time.monotonic()
        self._purge(now)
        for v, t in self.seen:
            if (now - t < cooldown_s()
                    and self.rec.match(vec, v, cv2.FaceRecognizerSF_FR_COSINE) > STRANGER_SAME):
                return True
        return False

    def remember(self, vec):
        self.seen.append((vec, time.monotonic()))


def speak(text, dry=False):
    """Say it in its own process, and carry on watching -- a greeting is not worth blocking the
    camera loop for. --optional means guide.say() DROPS the line rather than talk over a real
    answer or the narration of a trip somebody is being led on."""
    print(f"[greet] {text}", flush=True)
    if dry:
        return
    try:
        cmd = ["python3", SPEAK, "--optional", text]
        subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as e:
        print(f"[greet] could not speak: {e}", flush=True)


def look_once(det, rec, db, cap):
    """One look. Returns (name, score, face_px, vec, margin). name is None if nobody known is in
    view. `vec` comes back so the caller can LEARN from it -- see fl.maybe_learn."""
    ok, frame = cap.read()
    if not ok:
        return None, 0.0, 0, None, 0.0
    face = fl.biggest_face(det, frame)
    if face is None:
        return None, 0.0, 0, None, 0.0
    px = int(face[3])
    if px < MIN_FACE_PX:                  # too far: recognising it would be a coin toss
        return None, 0.0, px, None, 0.0
    vec = fl.face_vector(rec, frame, face)
    name, score, margin = fl.identify(rec, vec, db)
    return name, score, px, vec, margin


def main():
    once = "--once" in sys.argv
    dry = "--dry" in sys.argv

    db = fl.load_db()
    if not db:
        sys.exit("Nobody is enrolled yet. Run:  python3 ~/ros2_ws/face_enroll.py \"Your Name\"")
    det, rec = fl.load_models()
    cap, path = fl.open_camera()
    if cap is None:
        sys.exit(path)
    print(f"Camera: {path}")
    print("Knows: " + ", ".join(f"{n} ({len(e['enrolled'])}+{len(e['learned'])})"
                                for n, e in db.items()))

    # BAKE ANYTHING THAT IS NOT BAKED, EVERY TIME WE START. Nobody should have to remember to run
    # prebake_lines.py -- and until now somebody did, whenever the wording of a greeting changed:
    # enrolling from the web bakes the NEW person's lines, but rewriting GREETINGS leaves everyone
    # else's cached under the old text. An unbaked line still gets said, by Piper, live -- it just
    # costs a ~3 s pause in front of a person, which is exactly the pause the baking exists to
    # remove. This skips whatever is already baked, so on a normal start it is a no-op that costs
    # one process, and after a wording change it quietly fixes itself.
    if not dry:
        try:
            subprocess.Popen(["python3", PREBAKE], stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL, start_new_session=True)
        except Exception as e:
            print(f"[greet] could not kick off a re-bake: {e}", flush=True)

    try:
        if once:
            for _ in range(8):            # let auto-exposure settle before judging anything
                cap.read()
            name, score, px, _vec, _m = look_once(det, rec, db, cap)
            if name:
                print(f"✅ that is {name}  (match {score:.3f}, face {px} px)")
            elif px:
                print(f"👤 a face ({px} px) but nobody I know  (best match {score:.3f})")
            else:
                print("🚫 no face in view")
            return

        print(f"Watching at {FPS:.0f} fps. Ctrl-C to stop.{'  [DRY RUN]' if dry else ''}")
        last = {}                          # name -> when we last greeted them
        strangers = RecentStrangers(rec)   # ...and a few minutes of memory for the nameless
        period = 1.0 / FPS

        # ONE FRAME IS NOT AN OPINION.
        #
        # A real log: 0.531, 0.593, 0.470, 0.608, 0.439, 0.383, 0.377, 0.291 -- the same face, one
        # second, bouncing over and under the 0.363 line. Act on each frame and the robot greets
        # Rodrigo by name, then offers to help him as a stranger, then greets him again. It did
        # exactly that, and he had to explain to it that he was trying to scan his face.
        #
        # Two fixes, and they are different fixes:
        #   the UNSURE band  a face scoring just under the line is far more likely to be someone
        #                    it half-knows than a visitor. Treating it as a visitor is the WORST
        #                    guess available. So below the line but near it = say nothing.
        #   agreement        act only when several looks in a row agree. Costs a second; buys a
        #                    robot that does not argue with itself.
        votes = collections.deque(maxlen=VOTE_WINDOW)
        shown = None                       # the last thing we printed a [see] for
        cooled = None                      # the name we last announced a cooldown for
        while True:
            t0 = time.monotonic()
            name, score, px, vec, margin = look_once(det, rec, db, cap)

            if name:
                now_verdict = name
            elif px >= MIN_FACE_PX and score >= fl.MATCH_THRESHOLD - UNSURE_BAND:
                now_verdict = "unsure"        # probably someone we know, seen badly. Say nothing.
            elif px >= MIN_FACE_PX:
                now_verdict = "stranger"
            elif px:
                now_verdict = "far"
            else:
                now_verdict = "nobody"
            votes.append(now_verdict)

            # The window votes. An empty corridor needs no vote -- it is not a judgement call.
            if now_verdict in ("nobody", "far"):
                verdict = now_verdict
            else:
                winner, n = collections.Counter(votes).most_common(1)[0]
                if n < VOTE_NEEDED or winner in ("nobody", "far"):
                    time.sleep(max(0.0, period - (time.monotonic() - t0)))
                    continue                  # nothing has a majority yet -- look again
                verdict = winner
                if verdict != now_verdict:
                    # The vote won, not this frame. Use the frame that agrees with the vote, or we
                    # would learn from and greet on a reading the window just outvoted.
                    time.sleep(max(0.0, period - (time.monotonic() - t0)))
                    continue

            if verdict != shown:              # log the CHANGE, once, or it is 3 lines a second
                shown = verdict
                cooled = None                 # a new person in front -> a new cooldown line is due
                if name:
                    print(f"[see] {name} (match {score:.3f}, {px} px)", flush=True)
                elif verdict == "unsure":
                    print(f"[see] someone I half-recognise ({score:.3f}, {px} px) -- not sure "
                          f"enough to speak", flush=True)
                elif verdict == "stranger":
                    print(f"[see] a face I don't know ({px} px, best {score:.3f})", flush=True)
                elif verdict == "far":
                    print(f"[see] a face but too far ({px} px, need {MIN_FACE_PX})", flush=True)
                else:
                    print("[see] nobody", flush=True)

            if name:
                wait = cooldown_s() - (time.monotonic() - last.get(name, -1e9))
                if wait > 0:
                    if cooled != name:        # ONCE per arrival, on its OWN state -- the [see]
                        cooled = name          # line no longer stamps over this one every frame
                        print(f"[greet] {name}: cooling down, {wait:.0f}s to go", flush=True)
                else:
                    print(f"[greet] {name} (match {score:.3f}, margin {margin:.3f})", flush=True)
                    # LEARN FROM THIS SIGHTING, while we still hold the frame that earned it.
                    # maybe_learn does the deciding; most sightings teach it nothing new and that
                    # is the point. Only a confident, unambiguous, genuinely new angle gets in,
                    # and never over what a human enrolled.
                    if not dry and vec is not None:
                        why = fl.maybe_learn(rec, db, name, vec, score, margin)
                        if why:
                            print(f"[learn] {why}", flush=True)
                    speak(greeting_for(name), dry=dry)
                    last[name] = time.monotonic()
                    shown = cooled = None
                    votes.clear()             # the window is 30 s stale now; start looking fresh

            elif verdict == "stranger" and vec is not None:
                # A FACE IT DOES NOT KNOW -- which is what a visitor looks like, and a visitor is
                # the entire point of the robot. It offers help instead of a name. We do NOT learn
                # from these and we do NOT write them down: someone who walked past a robot has
                # agreed to nothing.
                if strangers.greeted_recently(vec):
                    if cooled != "stranger":
                        cooled = "stranger"
                        print("[greet] stranger: already said hello, leaving them be", flush=True)
                else:
                    print(f"[greet] someone I don't know (best {score:.3f}) -> offering help",
                          flush=True)
                    strangers.remember(vec)          # remember BEFORE speaking, not after
                    speak(STRANGER_GREETING, dry=dry)
                    shown = cooled = None
                    votes.clear()
            # Sleep the REST of the period, not a fixed amount: detection takes 30-60 ms and we
            # want 3 looks a second, not 3 + however long the maths took.
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        cap.release()


if __name__ == "__main__":
    main()
