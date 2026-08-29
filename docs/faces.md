# Faces — greeting people by name

The robot can recognise a face it has met before and say hello by name — *"Hi
Jonny! Tap Talk to me on the phone app if you need anything."* A stranger gets a
different opening, pointed at the same place. It only ever **talks**: it never
drives, never moves the arm, never listens.

> **It says hello. It does not listen.** Talking to the robot happens in the web
> app, under **Talk to me** — see [why](#why-it-does-not-listen).

- [What it does](#what-it-does)
- [Why it does not listen](#why-it-does-not-listen)
- [The two-camera trick](#the-two-camera-trick)
- [How recognition works](#how-recognition-works)
- [Enrolling someone](#enrolling-someone)
- [The greeting flow](#the-greeting-flow)
- [It learns on its own](#it-learns-on-its-own)
- [Privacy](#privacy)
- [Health and troubleshooting](#health-and-troubleshooting)
- [The files](#the-files)

---

## What it does

```
sees a face ──> knows them? ──yes──> "Hi Jonny! Tap Talk to me on the phone
                    │                 app if you need anything."
                    │
                    └──no──> "Hello! I'm Wall-E, the floor guide. Scan the QR
                             code on me and tap Talk to me, and I'll take you
                             where you need to go."
                                        │
                                        └──> the conversation happens THERE
```

Every greeting hands the person the thing that works: the app. A hello that leads
nowhere is a party trick.

A greeting is instant because each line is **pre-baked** to a WAV, one per enrolled
person (four openings each, picked at random so it does not sound like a recording).
Nobody has to bake them by hand — see [enrolling someone](#enrolling-someone).

Turn it on/off from the phone app: **"Say hello to people I know."** The choice
persists in `~/.cache/robot_ds/greeting_on`, so a reboot keeps it.

---

## Why it does not listen

It used to. The greeter had a whole conversation in it — mic open after each hello,
Whisper, Claude, a spoken reply. **The talking half always worked. The listening half
never became reliable**, and days went into finding out why. It was the room, not the
code:

| Measured | What it does |
|---|---|
| The 6-mic array runs its own **automatic gain**, and it drifts | After a quiet spell it amplifies the office until a voice sits only **1.5×** over the noise. Whisper returns `''` at that ratio. Re-plugging the USB resets it — for a while. |
| Its **opening transient** looks like speech | Open the mic in an empty room and the voice detector says VOICE within 60 ms, at a level of 0.005. The turn then records nothing, Whisper returns `''`, and the chat ends. Reproduced 2 of 2 runs, 29-jul. |
| The speaker sits **next to the mic** | The robot's own voice saturates the mic at 1.0. (This one was solved — the greeter waited for its mouth to close — but it is why a beep had to go, too.) |

The phone has none of these problems: the mic is at the user's mouth, nowhere near
the robot's speaker, and the browser hands over a clean stream. So the conversation
lives in the web app's **Talk to me**, where it works, and the greeter does the one
thing it is good at.

`face_greet.py` now contains **no microphone code at all**. That is deliberate: an
off switch for a broken feature is still a broken feature to maintain.

Diagnosing the mic itself (for the app's voice, which still uses it):
`~/mic_ruido.sh` measures the noise floor against a voice — under **3×** means the
gain has drifted and the USB wants re-plugging.

---

## The two-camera trick

The robot has **two** cameras doing two different jobs at the same time:

```
        Logitech C270  ── tilted UP ──> FACES        (on the arm)
        Orbbec depth   ── looking DOWN ──> chair legs / floor  (the GREEN chain)
```

The arm holds both in a fixed **scan pose**. At that one angle the Orbbec sees the
floor (so navigation still avoids chair legs) *and* the C270 sees faces at standing
height — both at once. The fixed angle between the two cameras is the whole trick.

> The face camera is the **C270**, not the Orbbec. The Orbbec is for obstacles and
> must keep pointing at the floor — see [the vision chain](./developing.md#the-vision-chain).

---

## How recognition works

Two OpenCV models, both in `~/ros2_ws/models/` (kept out of git — download them with
`bash ~/ros2_ws/get_face_models.sh`):

| Model | File | Job |
|---|---|---|
| **YuNet** | `face_detection_yunet_2023mar.onnx` | finds the face in the frame |
| **SFace** | `face_recognition_sface_2021dec.onnx` | turns it into 128 numbers — a "fingerprint" |

Two faces are the **same person** when their fingerprints match above
`MATCH_THRESHOLD = 0.363` (SFace cosine, OpenCV's own recommendation). All the numbers
live in `~/.cache/robot_ds/faces.json`.

**One look is not trusted.** The score bounces frame to frame across the 0.363 line,
so the robot **votes** over a short window (`VOTE_WINDOW`) and acts only on a clear,
unambiguous winner. A face smaller than `MIN_FACE_PX = 60` px is too far away to call —
it waits for them to come closer rather than guess.

---

## Enrolling someone

Easiest is the phone app — **"Add me"**:

- **Scan:** look at the camera and slowly turn your head. Varied angles = a strong
  fingerprint. This is the good way.
- **Upload photos:** works, but a few near-identical photos give it *one* angle, and
  the match then wobbles from across the room. If someone's match is flaky, re-enroll
  them by **scanning** and turning their head.

From a terminal (same models, same store):

```bash
python3 ~/ros2_ws/face_enroll.py         # enroll / re-enroll a person
```

**Baking is automatic — nobody should ever have to run it.** Two moments cover
everything:

| When | Who bakes it |
|---|---|
| Someone new is enrolled from the app | `server.py` kicks off a bake right after saving |
| The greeting *wording* changes in `face_greet.py` | `face_greet.py` bakes on every start |

Both skip whatever is already baked, so the normal case costs nothing. An unbaked
line is still said — Piper just synthesises it live, which is the ~3 s pause in front
of a person that baking exists to remove. By hand, if you ever want it now:

```bash
python3 ~/ros2_ws/prebake_lines.py
```

---

## The greeting flow

```
known face (voted 3-of-5)
   ├─ greeted them in the last 5 min?  ──yes──> stay quiet (COOLDOWN_S = 300)
   └─ no ──> one of four baked openings, at random  ──> back to watching

unknown face
   ├─ said hello to this face in the last few minutes?  ──yes──> leave them be
   └─ no ──> the visitor opening (QR code + Talk to me)  ──> back to watching
```

Two things keep it polite:

- **It never blocks.** The line is spoken in its own process, so the camera keeps
  watching while the robot talks.
- **It yields.** Greetings go out with `--optional`, which means a hello is **dropped**
  rather than spoken over a real answer, or over the narration of a trip somebody is
  being led on right now.

Unknown faces are held in RAM only, for a few minutes, purely so the same visitor is
not greeted three times a second. Nothing about them is written to disk, ever.

---

## It learns on its own

When it recognises you with plenty of room to spare, it can quietly save a **new
angle** of your face, so it knows you better over time. Three locks keep this safe:

- Only from a confident match (`LEARN_MIN = 0.60`, well above the 0.363 threshold).
- **Anti-poison** (`LEARN_CROSS_MAX = 0.40`): before learning "an angle of B" it checks
  the angle does not also look like some **other** enrolled person. If it does — *"this
  is A, I will not learn it as B"* — it refuses. This stops one person's face slowly
  leaking into another's.
- What a **human enrolled is sacred** and never touched. Learned angles live on a
  separate stack, capped at 12, so drift can always be thrown away without losing the
  real enrolment.
  
---

## Privacy

**This is employee biometrics. Treat it as such.**

- We store **fingerprints (128 numbers), never photos.** A fingerprint cannot be turned
  back into a face.
- Enrolling is **voluntary**: the person holds the phone and taps **Add me** themselves.
  The robot never asks a face to be remembered and never enrolls anyone on its own.
- Before pointing this at a whole floor of staff, **talk to whoever owns privacy.**
  Consent for a demo is not consent for a standing deployment.

---

## Health and troubleshooting

```bash
python3 ~/ros2_ws/face_check.py     # one-shot health: cameras, models, store, greeter
```

| Symptom | Likely cause |
|---|---|
| Greets no one | Greeting switch off, or the C270 is unplugged / not in scan pose |
| "Models not found" | Run `bash ~/ros2_ws/get_face_models.sh` (they are not in git) |
| Match wobbles from across the room | Enrolled from photos = one angle. Re-enroll by scanning, turning the head |
| Greets the wrong name | Two people confusable; re-enroll both with more varied angles |
| Greeter keeps dying | It is meant to: the watchdog in `server.py` (`_face_idle_watch`) revives it in ~2 s |
| Talked to it after a hello and it ignored you | Working as intended — it does not listen. Use **Talk to me** in the app ([why](#why-it-does-not-listen)) |
| A greeting pauses ~3 s before speaking | That line is not baked yet. It bakes itself on the next greeter start; `python3 ~/ros2_ws/prebake_lines.py` does it now |

The greeter yields the camera whenever the mirror/preview needs it, so **"not running
this second" is normal** and not a fault — the switch reflects what you asked for, not
whether the process happens to be up right now.

---

## The files

All standalone — **no `colcon build`** (see [developing.md](./developing.md#-what-needs-a-build-and-what-does-not)):

| File | Job |
|---|---|
| `face_lib.py` | The engine: detect, fingerprint, match, learn. Owns the thresholds. |
| `face_greet.py` | The loop: watch, vote, greet. Speaks only — no microphone code at all. |
| `face_check.py` | Health check. Run it before a demo. |
| `face_enroll.py` | Enroll / re-enroll from a terminal. |
| `get_face_models.sh` | Downloads the two `.onnx` models into `models/`. |
| `~/.cache/robot_ds/faces.json` | The fingerprints. Numbers only. |
| `~/.cache/robot_ds/greeting_on` | The on/off switch, so it survives a reboot. |

Quick tests:

```bash
python3 ~/ros2_ws/face_greet.py --once    # look once, report, say nothing
python3 ~/ros2_ws/face_greet.py --dry     # run, but print instead of speaking
```
