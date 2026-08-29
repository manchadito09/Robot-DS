#!/usr/bin/env python3
# face_check.py - is what the robot knows about faces any GOOD?
#
#   python3 ~/ros2_ws/face_check.py
#
# Enrolling is easy to do badly and impossible to see: six photos and one photo look identical in
# the UI, and both say "enrolled". They are not the same thing. Six copies of one face give you
# ONE angle, and the robot looks up at people from 45 cm through a webcam -- exactly the angle a
# phone photo does not have. This prints the two numbers that decide whether it will actually work:
#
#   SPREAD    how different a person's own fingerprints are from each other.
#             All ~1.0 = the same picture several times = one angle = fragile.
#   SEPARATION how close the nearest OTHER person is.
#             Anything near the 0.363 match threshold is two people the robot may mix up.
#
# It reads the store and nothing else: no camera, no ROS, safe to run any time.
import itertools
import sys

import cv2

sys.path.insert(0, "/home/ubuntu/ros2_ws")
import face_lib as fl


def main():
    db = fl.load_db()
    if not db:
        print("Nobody is enrolled yet.")
        return
    _, rec = fl.load_models()

    def sim(a, b):
        return rec.match(a, b, cv2.FaceRecognizerSF_FR_COSINE)

    print("=" * 68)
    print("  WHO IT KNOWS")
    print("=" * 68)
    weak = []
    for name in sorted(db):
        e = db[name]
        vs = fl.vectors_of(e)
        counts = f"{len(e['enrolled'])} taught"
        if e["learned"]:
            counts += f" + {len(e['learned'])} learned"
        if len(vs) < 2:
            print(f"  {name:10s} {counts:24s} only one fingerprint -- nothing to compare")
            weak.append(name)
            continue
        sims = [sim(a, b) for a, b in itertools.combinations(vs, 2)]
        avg = sum(sims) / len(sims)
        if avg > 0.95:
            verdict = "SAME PICTURE TWICE -- worth one photo. Ask them to scan."
            weak.append(name)
        elif avg > 0.85:
            verdict = "very alike -- little margin. A scan would help."
            weak.append(name)
        else:
            verdict = "good spread"
        print(f"  {name:10s} {counts:24s} spread {avg:.3f} ({min(sims):.2f}-{max(sims):.2f})  {verdict}")

    print()
    print("=" * 68)
    print("  CAN IT TELL THEM APART?   (match threshold is %.3f)" % fl.MATCH_THRESHOLD)
    print("=" * 68)
    if len(db) < 2:
        print("  Only one person enrolled -- nobody to confuse them with.")
    else:
        pairs = []
        for a, b in itertools.combinations(sorted(db), 2):
            worst = max(sim(x, y) for x in fl.vectors_of(db[a]) for y in fl.vectors_of(db[b]))
            pairs.append((worst, a, b))
        pairs.sort(reverse=True)
        for worst, a, b in pairs[:5]:
            if worst > fl.MATCH_THRESHOLD:
                flag = "MAY BE CONFUSED -- re-scan both"
            elif worst > fl.MATCH_THRESHOLD - 0.1:
                flag = "close for comfort"
            else:
                flag = "fine"
            print(f"  {a:10s} vs {b:10s} closest {worst:+.3f}   {flag}")
        if len(pairs) > 5:
            print(f"  ... and {len(pairs)-5} more pairs, all further apart than these.")

    print()
    if weak:
        print("  TO FIX: " + ", ".join(weak))
        print("  Have them open 'Add me' on the web app and scan instead of uploading. It uses")
        print("  the robot's own camera, at the angle it actually sees people from.")
        print("  Or leave it: the greeter learns a real angle the first few times it sees them.")
    else:
        print("  Nothing to fix. Everyone has a decent spread and nobody is close to anyone else.")


if __name__ == "__main__":
    main()
