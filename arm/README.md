# arm/ — the robot's hand-built arm gestures

The arm is for **expression**, not for picking things up. This folder is the **backup of
the gestures we built by hand**, so a reflash or a dead SD card doesn't take them with it.

> ### 🔴 The robot does not play them from here
> `arm_gesture.py` reads from **`/home/ubuntu/software/arm_pc/ActionGroups/`** — the
> vendor's folder, which also holds ~40 of Hiwonder's own action groups (pick, place,
> garbage sorting…). Editing a `.d6a` in *this* folder changes nothing on the robot.
> Copy it into the vendor folder to make it real. Same trap as
> [the three copies of the source](../docs/developing.md#-read-this-first-three-copies-of-the-source).

## What a `.d6a` is

A small **SQLite** file, one row per step of the movement:

```
ActionGroup(Index, Time, Servo1..Servo6)      Time = milliseconds for that step
```

Servo columns map to bus IDs `1,2,3,4,5,10` (Servo6 is the gripper, ID 10). The measured
safe ranges — and the rule about ramping every move so the arm doesn't jerk the chassis —
are in [developing.md → Add an arm gesture](../docs/developing.md#add-an-arm-gesture).

## Which gesture plays which file

Mapped in the `GESTURES` dict in `~/ros2_ws/arm_gesture.py`:

| Gesture (what you ask for) | File | |
|---|---|---|
| `wave` / `greet` | `wave.d6a` | elbow lifts, wrist waves — a real hello |
| `bigwave` | `big_wave.d6a` | arm up high, base swings wide |
| `raise` | `raise_arm.d6a` | rises and holds — "presenting" |
| `look` | `look.d6a` | turns one way, pauses, then the other |
| `nod` | `nod.d6a` | wrist bobs — "yes" |
| `shake` | `shake.d6a` | base wiggles — "no" |
| `dance` | `dance.d6a` | a silly little tremor |
| `twirl` | `twirl.d6a` | arm up, hand spins |
| `mouth` | `mouth.d6a` | gripper opens/closes like a talking mouth |
| `rest` / `home` | `init` | the vendor's neutral pose (not ours, not backed up here) |
| **`scan`** | **`self_drive`** | 🔴 **the camera pose — see below** (vendor file) |

## 🔴 `scan` is the one you must not casually change

`scan` is the pose that tilts the wrist down so the depth camera sees the floor just
ahead — the pose that lets the robot spot **chair legs**. It is the whole `GREEN` chain,
and `camera_calib.yaml` is measured **against this exact pose**.

> Change what `scan` maps to, or edit that action group, and you **must recalibrate the
> camera**. Every trip sets this pose before moving, which is also why gestures can't
> leave the camera aimed at the ceiling.

## Adding a gesture

1. Build the movement in the vendor's arm app, saving `<name>.d6a` into
   `/home/ubuntu/software/arm_pc/ActionGroups/`.
2. Add a `"<friendly name>": "<file name>"` line to `GESTURES` in `arm_gesture.py`
   (no build needed — it's a standalone script).
3. Copy the `.d6a` into this folder so it survives a reflash.
4. Test it **with the battery in** — no battery means no servos.
