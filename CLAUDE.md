# Robot-DS — Project Instructions

> Read alongside the global `~/.claude/CLAUDE.md`. This file overrides or extends the global rules only where called out. Always read `plan.md` first.

## What this is

A Claude-driven **indoor guide robot for floor 4**: on demand (by voice or the 7" touchscreen) it leads a visitor to a meeting room/office, a person/team, or an amenity — navigating autonomously and talking along the way — and can also run a guided **tour mode**. ~8-week build, **primarily one full-time developer (Rodrigo)** with part-time help from Adrián, demo recorded **~2026-07-28**. It is a **floor-4 guide robot** (superseding an earlier Direct Supply senior-care *delivery* concept). Used for customer tours, hackathons, and giving our Claudes eyes and legs.

## Where to look first

1. [`plan.md`](./plan.md) — single source of truth for current step and scope.
2. [`README.md`](./README.md) — what the robot is, how to start it, and the hardware it's made of.

If `plan.md` says we're on Step N, don't do work belonging to Step N+1 unless explicitly asked.

## Hardware reality (drives every code decision)

- **Compute:** team-owned Jetson Orin Nano Super 8GB (the JetRover is ordered **without** its bundled Jetson) + on-hand Jetson Nano 4GB as backup.
- **Chassis/platform:** Hiwonder JetRover Developer Kit — **tank/tracked chassis** (grips carpet, rolls over cables, pivots in place), with encoders, IMU, 2D LiDAR (A1), 3D depth cam, **6-mic far-field array + speaker**, **7" LCD**, and micro-ROS on the chassis MCU.
- **Arm:** a 6-DOF arm ships on board, but **autonomous pick-up is Phase 2** — not part of the v1 guide demo.

If a code change implies hardware we don't have, stop and flag it.

## Tech-stack additions over the global CLAUDE.md

| Layer | Tool | Notes |
|---|---|---|
| Robot OS | **ROS 2 Humble** on Ubuntu 22.04 | Workspace lives in `ros2_ws/` |
| Navigation | **Nav2 + SLAM Toolbox**, TEB controller | `nav2_controller_teb.yaml` is the file that matters — see [movement-nav.md](./docs/movement-nav.md) |
| Low obstacles | our own `camera_scan_node.py` — depth cloud → `/camera_scan` → costmap | The **GREEN chain**; this is what stops it hitting chair legs — see [developing.md](./docs/developing.md#the-vision-chain) |
| Faces | **OpenCV** YuNet (detect) + SFace (recognise) | see [faces.md](./docs/faces.md) |
| Speech | **Whisper** (listen) + **Piper** (speak), both local and kept warm as daemons | see [voice-guide.md](./docs/voice-guide.md) |
| Reasoning | **Claude**, kept warm as a local daemon (~2 s) | falls back to `claude -p` if the daemon is down |
| Chassis firmware | **micro-ROS** on the JetRover MCU (vendor-provided) | Don't fork the firmware unless we have to |
| Web app | Small Python HTTP server (`robot_web/`) + plain HTML/JS | No framework, no backend, no database |

## Repo layout (extends global)

```
.
├── README.md                     # Start here — what it is, how to run it, the hardware
├── plan.md                       # The whole project on one page
├── CONTRIBUTING.md               # How to propose or build an idea
├── CLAUDE.md                     # This file
├── docs/                         # operating, developing, faces, voice-guide, movement-nav, bringup
├── plans/                        # Working notes for specific pieces of work
├── ros2_ws/                      # ROS 2 workspace — mirrors the live one on the robot
│   └── src/robot_ds_behavior/    # Our behaviour package (guide, brain, talk)
├── robot_web/                    # The phone/laptop web app (server.py + index.html)
├── voice_prototype/              # Voice front-end + the live STT daemon
├── arm/                          # Arm gesture action groups (.d6a)
└── system/                       # systemd units + wifi watchdog (copies of /etc)
```

ROS 2 package names: `robot_ds_<area>` (snake_case, prefix locked).

## Build / scope rules (project-specific)

- **Demo > polish.** This is a pitch demo, not a product. Pick the simplest thing that survives a recorded run.
- **Off-the-shelf wins ties.** Vendor stack > our stack any time they're comparable.
- **No manipulator, no clinical claims, no HIPAA-relevant data.** v1 stays out of regulated territory.

## Definition of "done" for this project

A video and a live walkthrough of the robot:
1. Receiving a request — by voice or the 7" touchscreen — to be taken somewhere ("take me to the design team"),
2. Confirming, then leading the visitor autonomously across a hallway with at least one obstacle, narrating along the way,
3. Arriving at the goal and announcing arrival — all without teleop intervention.

(Return-to-base and the one-page business case were dropped from scope by Rodrigo on
2026-07-11 — not wanted.)

## Things I should not do without asking

- Place hardware orders (humans place orders).
- Change the use case (the floor-4 guide robot).
- Add autonomous arm/manipulation behavior to v1 (the arm is on board, but pick-up is Phase 2).
- Bring in a second compute platform (RPi, x86 host) on top of the Jetsons.
- Start writing the backend / frontend before Step 7 of `plan.md`.
