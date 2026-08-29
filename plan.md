# Robot-DS — Plan

**Status: working.** The v1 guide robot has been demoed live, end to end, without
teleoperation.

```
ask ──> understand ──> confirm ──> drive ──> avoid ──> arrive ──> narrate
 voice/screen   Claude              Nav2    chair legs         Piper
```

---

## Goal

A working, Claude-driven **guide robot for floor 4**: a visitor who doesn't know the
floor asks — **by voice or via the 7" touchscreen** — to be taken to a **meeting
room/office, a person/team, or an amenity** (kitchen, restrooms, reception), and the
robot **leads them there autonomously**, talking along the way. It also runs a **tour
mode** — a set route with fun facts about the office. It works **on demand**: it waits
and reacts when someone approaches or speaks to it. Built **primarily by one full-time
developer (Rodrigo)** with part-time help from **Adrián**, on the team's own **Jetson
Orin Nano** + a **Hiwonder JetRover Developer Kit** (tank chassis; arm, far-field
mic+speaker and 7" LCD included). Used for **customer tours, hackathons, and
giving our Claudes eyes and legs.**

---

## What's built

| Capability | State |
|---|---|
| **Take me to X** — voice or 7" screen → leads you there, narrating | ✅ working, demoed |
| **Tour mode** — set route, fun facts at each stop | ✅ working |
| **Answers questions** about the office while stopped | ✅ working |
| **Avoids low obstacles** (chair legs) via the depth-camera vision chain | ✅ working — the `GREEN` chain |
| **Voice** — Whisper (listen) + Claude (warm daemon) + Piper (speak), all local | ✅ working |
| **Arm gestures** — wave, dance, yes/no… (expression only, no pick-up) | ✅ working on hardware |
| **Greets people by name** — "Hi Jonny!", pointing them at the app to talk | ✅ working — see [faces.md](./docs/faces.md) |
| **Runs on our own map of floor 4**, glass walls and all | ✅ working |

How each half works, in depth: [voice-guide.md](./docs/voice-guide.md) (listening and
talking), [movement-nav.md](./docs/movement-nav.md) (mapping and driving),
[faces.md](./docs/faces.md) (greeting by name), [bringup.md](./docs/bringup.md) (how the
robot was brought up from a box). The traps that cost us days are in
[developing.md](./docs/developing.md#traps-that-have-cost-us-days).

---

## Next — small, high-value, mostly wiring together what we already have

| Idea | What it adds | Notes |
|---|---|---|
| **"Shall I take you to your desk?"** | Recognises you → offers to lead you to your own desk | Joins two things we already have: faces + waypoints. Plan in [`plans/`](./plans/). The offer and the person→desk map are safe to build; the *driving* part needs a person in front of the robot to test. |
| **Re-enroll the weak faces** | People enrolled from photos (one angle) match badly from across the room | Re-scan them turning their head — see [faces.md](./docs/faces.md#enrolling-someone). |
| **Say goodbye, not just hello** | Close a greeting gracefully when someone walks off | Small change to the greeter loop. |

---

## Later — Phase 2 and beyond

Parked on purpose to protect the v1 scope. Not started.

- **Empty-room / lights-off concierge** — patrol, spot empty rooms (Claude vision), turn
  off lights via the building API.
- **Proactive guiding** — roam and approach visitors who look lost, instead of waiting.
- **Arm pick-up** — the 6-DOF arm is on board; add Claude-driven grabbing of light
  objects. This is the deliberate Phase-2 line; v1 stays out of manipulation.

---

## Non-goals

- Production-grade reliability, regulatory clearance, HIPAA-compliant data handling.
- Custom PCB or custom-machined chassis — off-the-shelf wherever possible.
- A full software platform (fleet management, multi-robot coordination).
- Multi-floor / elevator autonomy in v1.

---

## Product potential

The same platform is a starting point for something Direct Supply could deploy on other
floors and offer to third parties:

- **Reusable concierge** — the see / hear / speak / navigate stack ports to any floor
  with a fresh map.
- **Configurable behaviours** — guidance, patrol, room-availability and reporting are
  software modules that can be switched on per client.
- **Manipulation upsell** — the on-board arm (Phase 2) opens delivery and fetch-and-carry
  use cases from the same base.
- **Low marginal cost** — off-the-shelf hardware per unit, with the intelligence running
  on Claude.

---

## Standing risks

- **Single developer.** Almost all of this rests on Rodrigo. Mitigation: strict
  demo-first scope, and everything documented so someone else can pick it up — start at
  [CONTRIBUTING.md](./CONTRIBUTING.md).
- **Battery autonomy.** Arm + 7" LCD + Jetson draw a lot → ~2 h runtime, ~3 h recharge.
  Mitigation: two batteries, hot-swap, so testing and demos never wait on a charge.
- **Faces = employee biometrics.** Fingerprints not photos, opt-in only; clear it with
  whoever owns privacy before any standing deployment — see
  [faces.md](./docs/faces.md#privacy).
- **Destination upkeep.** Named waypoints (people/teams especially) drift as the office
  changes — re-record before any demo with the web app or `waypoint_tool.py`.
- **Tank chassis on wood.** Tracks can scrub the wood/tarima areas — drive gently and
  turn wide there. Most of the route is carpet, the tank's strength.
