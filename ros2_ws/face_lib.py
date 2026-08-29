#!/usr/bin/env python3
# face_lib.py - the shared guts of "Hi Jonny": the camera, the two models, and the face store.
#
# Used by face_enroll.py (teach it a face) and face_greet.py (say hello). Plain OpenCV and
# V4L2 -- NO ROS imports on purpose, so these run without sourcing anything and a wedged ROS
# can never take the greeter down with it. Speaking is delegated to speak.py, which does the
# ROS part in its own process.
#
# WHY THESE MODELS: OpenCV 4.10 already ships YuNet (detect) and SFace (recognise). dlib /
# face_recognition would mean a long, fragile ARM compile for no gain. Two ONNX files, ~37 MB,
# and nothing else to install.
import os

# THREADS BEFORE cv2. This is not a style choice -- OpenBLAS once spun six threads in a
# busy-wait inside camera_scan_node and ate the whole Jetson (374 % CPU, load 15). Anything
# doing matrix maths on this robot caps its threads FIRST, at import time, before the library
# reads the environment and decides for itself.
os.environ.setdefault("OMP_NUM_THREADS", "2")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "2")
os.environ.setdefault("MKL_NUM_THREADS", "2")

import glob
import json
import time

import cv2
import numpy as np

cv2.setNumThreads(2)

MODELS = os.path.expanduser("~/ros2_ws/models")
YUNET = os.path.join(MODELS, "face_detection_yunet_2023mar.onnx")
SFACE = os.path.join(MODELS, "face_recognition_sface_2021dec.onnx")

FACES_DB = os.path.expanduser("~/.cache/robot_ds/faces.json")

# The C270's STABLE path. /dev/video2 is a NUMBER, and numbers move -- exactly how the speaker
# ended up talking into a microphone when the mic array renumbered the sound cards. This path
# carries the camera's own serial, so it survives reboots and re-plugging.
CAM_BY_ID_GLOB = "/dev/v4l/by-id/usb-046d_0825_*-video-index0"

W, H = 640, 480
DET_SCORE = 0.7          # below this, it is not a face
MATCH_THRESHOLD = 0.363  # SFace cosine, OpenCV's own recommendation. Above = same person.


def camera_path():
    """The by-id symlink for the C270 -- the one path that carries its serial and so cannot
    be stolen by whatever else enumerates first. Returns None if it is not plugged in."""
    hits = sorted(glob.glob(CAM_BY_ID_GLOB))
    return hits[0] if hits else None


def camera_index():
    """The number OpenCV insists on, found via the name we can trust.

    OpenCV's V4L2 backend CANNOT open a device by path -- it says so out loud ("backend is
    generally available but can't be used to capture by name") and hands back a closed capture.
    But an index is exactly the sort of thing that shifts under you: this camera is video2 today
    and the mic array already proved how that ends (it took card 1 and the speaker silently
    became card 2). So: identify by serial, then resolve to the index at the last moment.
    Returns (index, label) or (None, why-not).
    """
    path = camera_path()
    if path:
        real = os.path.realpath(path)                    # /dev/v4l/by-id/... -> /dev/video2
        if real.startswith("/dev/video"):
            try:
                return int(real[len("/dev/video"):]), f"{real} (by serial)"
            except ValueError:
                pass
    return None, "No C270 found -- is it plugged in?"


def open_camera():
    """MJPG, 640x480. MJPG matters: the same frames raw (YUYV) would be ~18 MB/s -- as much as
    the depth camera -- on a USB 2 bus they SHARE. Compressed it is ~1-2 MB/s and the depth
    camera never notices we are here."""
    idx, label = camera_index()
    if idx is None:
        return None, label
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # always the newest frame, never a stale queue
    if not cap.isOpened():
        # One webcam, one owner. Nine times out of ten this is the web app's "Add me" mirror
        # still holding it, and the fix is to leave that screen -- so say so, rather than making
        # someone go and learn `fuser` to find out.
        return None, ("Could not open the camera. Something else has it -- close the 'Add me' "
                      "screen in the web app (it lets go after ~8 s) and try again.")
    return cap, label


def load_models():
    for p in (YUNET, SFACE):
        if not os.path.exists(p):
            raise SystemExit(
                f"Missing model: {p}\nDownload them into ~/ros2_ws/models (see docs/developing.md).")
    det = cv2.FaceDetectorYN.create(YUNET, "", (W, H), DET_SCORE, 0.3, 5000)
    rec = cv2.FaceRecognizerSF.create(SFACE, "")
    return det, rec


def biggest_face(det, frame):
    """The face we care about is the one in front of the robot -- i.e. the biggest one.
    Returns the raw detection row, or None."""
    det.setInputSize((frame.shape[1], frame.shape[0]))
    _, faces = det.detect(frame)
    if faces is None or not len(faces):
        return None
    return max(faces, key=lambda f: f[2] * f[3])      # w * h


def face_vector(rec, frame, face):
    """The 128-number 'fingerprint' of a face. This -- and never the photo -- is what we store."""
    return rec.feature(rec.alignCrop(frame, face))


# ---- The store: what you TAUGHT it, and what it worked out for itself ------------
#
# Two lists per person, and the split is the whole safety of learning-as-it-goes:
#
#   enrolled  what a human deliberately gave it. SACRED. Never edited, never dropped, never
#             learned over. It is the anchor -- whatever the robot talks itself into later,
#             the truth about this face is still in here.
#   learned   fingerprints it added itself after recognising someone with confidence to spare.
#             Capped, and the oldest falls off the end.
#
# Why the split matters: a system that trains on its own output can poison itself. Mistake Adrian
# for Rodrigo ONCE, learn it, and now Adrian's face is filed under Rodrigo -- so the next mistake
# is likelier, and so is the one after that. Errors compound. Keeping the enrolled set untouchable
# means a bad day can never erase the good data, and capping `learned` means it cannot drown it.
LEARN_MIN = 0.60        # only learn from a match with room to spare (threshold is 0.363)
LEARN_MARGIN = 0.15     # ...and only if the runner-up PERSON is well behind. Ambiguous = no.
LEARN_MAX_SIM = 0.90    # ...and only if it adds a new angle, not a copy of what we have
LEARN_CROSS_MAX = 0.40  # ...and only if it does NOT look like some OTHER enrolled person. Different
                        # people score ~0.2-0.27 against each other; the SAME person 0.5+. So a would-
                        # be "Adrian angle" that matches Rodrigo's ENROLLED face above 0.40 is almost
                        # certainly Rodrigo, about to be mis-filed under Adrian -- the poison. Reject.
LEARN_CAP = 12          # most learned fingerprints kept per person


def _vec(lst):
    return np.array(lst, dtype=np.float32).reshape(1, -1)


def load_db():
    """{name: {"enrolled": [vec], "learned": [vec]}}.

    Reads the old flat format too ({name: [vec]}) and treats those as enrolled -- nobody should
    have to think about a file format because we changed our minds later.
    """
    try:
        with open(FACES_DB) as f:
            raw = json.load(f)
    except (FileNotFoundError, ValueError, OSError):
        return {}
    db = {}
    for name, entry in raw.items():
        if isinstance(entry, list):                      # old format: everything was enrolled
            db[name] = {"enrolled": [_vec(v) for v in entry], "learned": []}
        else:
            db[name] = {"enrolled": [_vec(v) for v in entry.get("enrolled", [])],
                        "learned": [_vec(v) for v in entry.get("learned", [])]}
    return db


def save_db(db):
    os.makedirs(os.path.dirname(FACES_DB), exist_ok=True)
    raw = {n: {"enrolled": [v.reshape(-1).tolist() for v in e["enrolled"]],
               "learned": [v.reshape(-1).tolist() for v in e["learned"]]}
           for n, e in db.items()}
    tmp = FACES_DB + ".part"
    with open(tmp, "w") as f:
        json.dump(raw, f)
    os.replace(tmp, FACES_DB)                # atomic: never leave a half-written store


def vectors_of(entry):
    return entry["enrolled"] + entry["learned"]


def set_enrolled(db, name, vectors):
    """Enrolling (or re-enrolling) REPLACES, and throws away what was learned about them: the
    newest deliberate answer wins, and anything worked out from the old face is now suspect."""
    db[name] = {"enrolled": list(vectors), "learned": []}


def load_faces():
    """Compat view for anything that just wants {name: [all vectors]}."""
    return {n: vectors_of(e) for n, e in load_db().items()}


def save_faces(faces):
    """Compat: treat a flat {name: [vectors]} as a deliberate enrolment."""
    db = load_db()
    for name, vecs in faces.items():
        set_enrolled(db, name, vecs)
    for gone in [n for n in db if n not in faces]:
        del db[gone]
    save_db(db)


def _best_per_person(rec, vec, db):
    """{name: best score against that person's vectors}. Best, not average: someone enrolled from
    five angles should be recognised if ANY of them fits."""
    out = {}
    for name, entry in db.items():
        best = 0.0
        for v in vectors_of(entry):
            s = rec.match(vec, v, cv2.FaceRecognizerSF_FR_COSINE)
            if s > best:
                best = s
        out[name] = best
    return out


def identify(rec, vec, db):
    """Who is this? Returns (name, score, margin).

    `margin` is how far ahead the winner is of the next PERSON -- not of their own other photos.
    A confident match is not just a high score; it is a high score that nobody else came close to.
    Returns (None, best_score, 0.0) if nothing clears the threshold.
    """
    if not db:
        return None, 0.0, 0.0
    if db and isinstance(next(iter(db.values())), list):      # someone passed the compat view
        db = {n: {"enrolled": v, "learned": []} for n, v in db.items()}
    scores = _best_per_person(rec, vec, db)
    ranked = sorted(scores.items(), key=lambda kv: kv[1], reverse=True)
    name, best = ranked[0]
    second = ranked[1][1] if len(ranked) > 1 else 0.0
    if best >= MATCH_THRESHOLD:
        return name, best, best - second
    return None, best, 0.0


def spread(rec, vectors):
    """How DIFFERENT a person's own fingerprints are from each other, 0..1 (higher = more alike).

    All ~1.0 means the same photo several times: one angle, and the robot only recognises that
    exact pose. This is the single number that tells a good enrolment from a useless one, and the
    UI can show it the moment someone uploads their photos -- before they walk away thinking they
    are enrolled when they have taught it one frozen angle.
    """
    vs = list(vectors)
    if len(vs) < 2:
        return 1.0
    sims, n = 0.0, 0
    for i in range(len(vs)):
        for j in range(i + 1, len(vs)):
            sims += rec.match(vs[i], vs[j], cv2.FaceRecognizerSF_FR_COSINE)
            n += 1
    return sims / n if n else 1.0


def enrolment_quality(rec, vectors):
    """A plain verdict for the UI: ('good'|'weak'|'single', spread). 'weak' = near-duplicate shots
    worth one angle; the caller nudges them to scan instead."""
    vs = list(vectors)
    if len(vs) < 2:
        return "single", 1.0
    s = spread(rec, vs)
    if s > 0.95:
        return "weak", s          # basically one photo repeated
    if s > 0.88:
        return "thin", s          # a bit alike -- works, but little margin
    return "good", s


def closest_other(rec, db, vectors, exclude=None):
    """The enrolled person these fingerprints look MOST like (name, score), ignoring `exclude`.

    Run when someone new enrols: if the closest OTHER person scores near the match threshold, the
    two are confusable and the robot will mix them up in front of testers. Better to say so now.
    """
    best_name, best = None, 0.0
    for name, entry in db.items():
        if name == exclude:
            continue
        for a in vectors:
            for b in vectors_of(entry):
                sc = rec.match(a, b, cv2.FaceRecognizerSF_FR_COSINE)
                if sc > best:
                    best_name, best = name, sc
    return best_name, best


def maybe_learn(rec, db, name, vec, score, margin):
    """Add this sighting to what we know about `name` -- but only if it is safe AND useful.

    Returns a short reason string when it learns, or None. The five gates, in order of how badly
    each one bites if you skip it:

      1. confident      a 0.37 match is exactly the one you must not learn from
      2. unambiguous    if the runner-up is close, learning it is how profiles bleed together
      3. not-someone-else  the real danger: a CONFIDENT mis-ID. If this face looks like a different
                        ENROLLED person, it is probably them, and learning it would poison a profile
                        with someone else's face. This is the gate that answers "what if it is sure
                        but wrong". Enrolled-only, because we trust what a human taught, not what we
                        might already have learned in error.
      4. novel          a near-copy adds nothing and pushes a real angle off the end of the cap
      5. enrolled safe  never touched -- handled by only ever appending to `learned`
      6. capped         oldest learned falls off, so this cannot grow without bound
    """
    if score < LEARN_MIN:
        return None
    if margin < LEARN_MARGIN:
        return None
    entry = db.get(name)
    if entry is None:
        return None
    # THE POISON GUARD. Does this "new angle for `name`" actually look like a DIFFERENT enrolled
    # person? If so, we are about to file their face under the wrong name. Refuse.
    for other, entry2 in db.items():
        if other == name:
            continue
        for v in entry2["enrolled"]:
            if rec.match(vec, v, cv2.FaceRecognizerSF_FR_COSINE) > LEARN_CROSS_MAX:
                return None                              # looks like someone else -> never learn it
    for v in vectors_of(entry):
        if rec.match(vec, v, cv2.FaceRecognizerSF_FR_COSINE) > LEARN_MAX_SIM:
            return None                                  # we already know this angle
    entry["learned"].append(vec)
    dropped = ""
    if len(entry["learned"]) > LEARN_CAP:
        entry["learned"].pop(0)                          # oldest out; enrolled never in this list
        dropped = ", oldest dropped"
    save_db(db)
    return f"learned a new angle for {name} ({len(entry['learned'])}/{LEARN_CAP}{dropped})"
