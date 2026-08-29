#!/usr/bin/env python3
# prebake_lines.py - bake the robot's FIXED spoken lines to WAV, up front, standing still.
#
# WHY THIS IS A SEPARATE TOOL
#
# guide.py already bakes them -- in a daemon thread, started when the Guide node is constructed.
# But guide.py is spawned PER TRIP and dies when the trip ends, taking the thread with it. A short
# trip kills the bake half-way through; the next trip starts from scratch and dies again. The lines
# never finish baking, and 12 of the 13 had never been baked at all.
#
# When a line has no WAV, guide.py falls back to synthesising it live -- with Piper, on this Jetson,
# WHILE THE ROBOT IS DRIVING AND TALKING. Which is precisely the cost pre-baking exists to avoid.
# The visitor gets a robot that pauses mid-corridor to think about what to say.
#
# So bake them from OUTSIDE the trip, once, with nothing else going on. No ROS: just Piper.
#
#   python3 ~/ros2_ws/prebake_lines.py           # bake whatever is missing (skips what it has)
#   python3 ~/ros2_ws/prebake_lines.py --force   # re-bake everything (after editing the lines)
#
# Run it after ANY edit to office_knowledge.yaml's `about` facts or to guide.py's ROBOT_LINES: the
# cache key is a hash of the model + the exact text, so a changed word means a new WAV and the old
# one is dead weight.
import os
import subprocess
import sys

# IMPORT THE INSTALLED guide.py, NOT the source. The robot runs from the colcon BUILD, so that is
# what actually decides which lines come out of the speaker. Import the source instead and, the
# moment someone edits guide.py and forgets `colcon build`, we bake one set of WAVs while the robot
# reads from another -- and every line it wants is a cache miss, synthesised live, while driving.
# Whatever the robot says, we bake. Even if that means baking a stale line: at least it is the line
# it is going to say.
for p in ("~/ros2_ws/install/robot_ds_behavior/lib/python3.10/site-packages",
          "~/ros2_ws/src/robot_ds_behavior"):                   # source only as a fallback
    d = os.path.expanduser(p)
    if os.path.isdir(os.path.join(d, "robot_ds_behavior")):
        sys.path.insert(0, d)
        break

# Import the real thing. Deriving the lines and the hash by hand here would be a second source of
# truth, and it would drift the first time someone edits a line: we would bake WAVs the robot never
# looks for and swear the cache was warm.
from robot_ds_behavior.guide import (            # noqa: E402
    CACHE_DIR, NARRATE_LINES, PIPER_BIN, PIPER_MODEL, _wav_path,
)

print(f"lines from: {os.path.dirname(sys.modules['robot_ds_behavior.guide'].__file__)}")

# THE GREETER'S LINES TOO. "Hi Rodrigo! Tap Talk to me..." is a fixed sentence per person, and
# baking it turns the greeting from a ~3 s Piper synthesis -- the exact "why did it pause to think"
# lag -- into an instant aplay. The stranger hello never changes; the per-name ones are built from
# whoever is enrolled right now, and re-baked whenever someone new is added. Imported from
# face_greet so the text is ONE source of truth: bake a sentence the greeter does not actually say
# and the cache is warm for nothing.
def _greeter_lines():
    try:
        sys.path.insert(0, os.path.expanduser("~/ros2_ws"))
        import face_lib as _fl
        import face_greet as _fg
    except Exception as e:
        print(f"(skipping greeter lines: {e})")
        return []
    lines = [_fg.STRANGER_GREETING]
    for name in _fl.load_db():
        for g in _fg.GREETINGS:              # bake EVERY opening per person, so each is instant
            lines.append(g.format(name=name))
    return lines


force = "--force" in sys.argv
EXTRA_LINES = _greeter_lines()

if not os.path.exists(PIPER_BIN):
    sys.exit(f"no piper binary at {PIPER_BIN}")
if not os.path.exists(PIPER_MODEL):
    sys.exit(f"no piper model at {PIPER_MODEL}")
os.makedirs(CACHE_DIR, exist_ok=True)

# Piper must use ITS OWN bundled libs, not the ROS ones on a service's huge LD_LIBRARY_PATH -- with
# those it dies on a symbol lookup, exits non-zero, and writes zero audio. Same fix as guide.py.
env = dict(os.environ)
env["LD_LIBRARY_PATH"] = os.path.dirname(PIPER_BIN)

baked = skipped = failed = 0
ALL_LINES = list(NARRATE_LINES) + EXTRA_LINES
for text in ALL_LINES:
    out = _wav_path(text)
    if os.path.exists(out) and not force:
        skipped += 1
        print(f"  have  {text[:70]}")
        continue
    tmp = out + ".part"
    try:
        subprocess.run([PIPER_BIN, "--model", PIPER_MODEL, "--output_file", tmp],
                       input=text.encode("utf-8"), stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, env=env, timeout=60)
        if os.path.getsize(tmp) > 44:            # bigger than a bare WAV header
            os.replace(tmp, out)                 # atomic: a half-written WAV is never played
            baked += 1
            print(f"  BAKED {text[:70]}")
        else:
            failed += 1
            print(f"  FAIL  (piper produced no audio) {text[:50]}")
    except Exception as e:
        failed += 1
        print(f"  FAIL  ({e}) {text[:50]}")

print(f"\n{baked} baked, {skipped} already there, {failed} failed "
      f"-- {len(ALL_LINES)} lines total ({len(EXTRA_LINES)} of them the greeter's)")
print(f"cache: {CACHE_DIR}")
sys.exit(1 if failed else 0)
