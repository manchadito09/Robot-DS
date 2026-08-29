#!/usr/bin/env python3
# face_enroll.py - teach the robot a face, so it can greet that person by name.
#
#   python3 ~/ros2_ws/face_enroll.py "Jonny"        # stand ~2 m in front and look at the camera
#   python3 ~/ros2_ws/face_enroll.py --list         # who does it know?
#   python3 ~/ros2_ws/face_enroll.py --remove "Jonny"
#
# PRIVACY, and this is deliberate: we store the 128-number FINGERPRINT of a face, never the
# photo. A face cannot be reconstructed from it, and nobody's picture sits on the robot's disk.
# Enrolment is opt-in by construction -- someone has to stand here and run this.
#
# Takes several shots over a few seconds rather than one. One photo only recognises the exact
# pose it was taken in; a handful, while you move your head a little, recognises a person.
import sys
import time

import face_lib as fl

SHOTS = 6              # fingerprints kept per person
SHOT_GAP = 0.8         # s between shots -- long enough to turn your head a little
WARMUP = 8             # frames thrown away so auto-exposure settles first


def enroll(name):
    faces = fl.load_faces()
    det, rec = fl.load_models()
    cap, path = fl.open_camera()
    if cap is None:
        sys.exit(path)
    print(f"Camera: {path}")
    print(f"\nEnrolling '{name}'. Stand about 2 m in front, look at the camera,")
    print("and turn your head a LITTLE between shots (left, right, up a bit).\n")
    try:
        for _ in range(WARMUP):
            cap.read()
        vectors, tries = [], 0
        while len(vectors) < SHOTS and tries < SHOTS * 8:
            tries += 1
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.1)
                continue
            face = fl.biggest_face(det, frame)
            if face is None:
                print("  ...no face in view -- step into the camera's line of sight")
                time.sleep(0.4)
                continue
            vectors.append(fl.face_vector(rec, frame, face))
            w, h = int(face[2]), int(face[3])
            print(f"  [{len(vectors)}/{SHOTS}] got it  (face {w}x{h} px)")
            time.sleep(SHOT_GAP)
    finally:
        cap.release()

    if len(vectors) < 2:
        sys.exit("\nNot enough shots -- nothing saved. Is the camera aimed at your face?")
    faces[name] = vectors                    # re-enrolling replaces: the newest aim wins
    fl.save_faces(faces)
    print(f"\n✅ '{name}' enrolled with {len(vectors)} fingerprints -> {fl.FACES_DB}")
    print("   (fingerprints only -- no photos are stored)")


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__ or "")
        print("Usage: face_enroll.py \"Name\" | --list | --remove \"Name\"")
        return
    if args[0] == "--list":
        faces = fl.load_faces()
        if not faces:
            print("Nobody enrolled yet.")
            return
        print("The robot knows:")
        for n, v in faces.items():
            print(f"  {n}  ({len(v)} fingerprints)")
        return
    if args[0] == "--remove":
        if len(args) < 2:
            sys.exit("Who? face_enroll.py --remove \"Jonny\"")
        name = " ".join(args[1:])
        faces = fl.load_faces()
        if name not in faces:
            sys.exit(f"'{name}' is not enrolled.")
        del faces[name]
        fl.save_faces(faces)
        print(f"Removed '{name}'.")
        return
    enroll(" ".join(args).strip())


if __name__ == "__main__":
    main()
