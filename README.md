# Robot-DS — the floor-4 guide robot

> **Public snapshot.** A few things are deliberately left out of this repo — the
> office floor map, the vendor's unlicensed packages, and our remote-access details.
> See [PUBLIC_SNAPSHOT.md](./PUBLIC_SNAPSHOT.md).


A Claude-driven **indoor guide robot**. Ask it — out loud or by typing — to take you
somewhere, and it leads you there on its own: planning its route, driving around
obstacles, and talking to you along the way. It also runs **guided tours** of the floor.

**Status: working.** It has been demoed live, end to end, without teleoperation.

```
        "Take me to the kitchen"
                 |
    understands -> plans -> drives -> avoids -> arrives -> tells you about it
```

Built on a Hiwonder JetRover (tank chassis) + Jetson Orin Nano.

---

## Start it in 5 minutes

You do not need to know ROS, or type a single command.

| # | Do this | Why it matters |
|---|---|---|
| Optional | **Plug the Jetson's power cable into the board.** | For **development only**: powers the Jetson so you can edit and build code without draining the batteries. It does **not** power the servos or lidar, so the robot still needs the **battery** to move or see — this is just to save battery while coding. |
| 1 | **Put the battery in.** | No battery = no servos and no lidar. Nothing works on cable power. |
| 2 | Double-click **NAVIGATION** on the desktop. | Starts the robot's brain, the map and the navigation stack. |
| 3 | Double-click **Show QR (phone app)**. | Aims the camera, starts the vision chain, opens the phone app QR. |
| 4 | **Wait for the pop-up: `Robot camera: GREEN`.** | **The single most important step.** |
| 5 | Open the web app (scan the QR, or open it on the laptop). | This is the remote control. |

> ### The one rule: no GREEN, no go.
> The phone showing **video** does **not** mean the robot can **see obstacles**. Those are
> two different things, and the difference is a chair the robot drives straight into.
> `GREEN` is the only proof the whole vision chain is alive. Never start a demo without it.

**What the colours mean:**

| Verdict | What it means | Do |
|---|---|---|
| 🟢 **GREEN** | The whole vision chain is alive — the depth camera is turning what it sees into obstacles the robot can avoid, chair legs included. | Good to go. |
| 🟡 **BLOCKED** | Something is right in front of the lens. | Move it, and it re-checks. |
| 🔴 **RED** | The chain is down. The robot would drive **blind** to low obstacles. | Do **not** go. Re-run **Show QR**. |

**How the check runs.** It runs **automatically** when you double-click **Show QR (phone
app)** — the same icon also shown as **Robot Web QR**. That icon aims the camera, starts
the vision chain, and pops up the verdict. You can also run the
same check yourself, any time, in the robot's terminal:

```bash
bash ~/ros2_ws/camera_check.sh    # 0 = GREEN, 1 = RED, 2 = BLOCKED — and it repairs what it can
```

## Use it

Talk to it (mic or the text box) — plain language works:

```
"Take me to the kitchen"          -> leads you there, narrating
"Give me a tour"                  -> visits each stop and explains them
"Where can I get coffee?"         -> answers from what it knows about the office
"What is the glass room?"
```

It can also **greet people it knows by name** — *"Hi Jonny! Tap Talk to me on the phone
app if you need anything."* It greets; the talking back happens in the app. Add yourself
from the web app (**Add me**). How it works, and the privacy side, is in
[`docs/faces.md`](./docs/faces.md).

Or use the web app: **Map** (tap to go / save a point), **Tour** (pick stops, hit Start),
**Arm** (Wave, Dance, Yes/No…), **Talk to me**, **Add me** (greeting), and a big red **STOP**.

**Two habits that keep it smooth:**
- **Be clear.** "Take me to the kitchen" beats "I want to do stuff" (vague asks make it
  guess, and it may set off on a whole tour).
- **One thing at a time.** Let each trip finish.

## If something looks wrong

| What you see | What to do |
|---|---|
| Camera **BLOCKED** | Something is in front of the lens. Move it. |
| Camera **RED** / **stale** | It repairs itself in ~30 s. Otherwise double-click **Show QR** again. |
| It bumps low things (chair legs) | The vision chain is down. You skipped `GREEN`. Re-run **Show QR**. |
| Web app looks odd | Reload the page. This never stops the robot. |
| It is about to hit something | Press **STOP** in the web app. |

Full guide: [`docs/operating.md`](./docs/operating.md).

## Documentation

| Doc | For whom |
|---|---|
| [**docs/operating.md**](./docs/operating.md) | **Anyone using the robot.** Start it, drive it, run tours, fix common problems. No code. |
| [**docs/developing.md**](./docs/developing.md) | **Anyone changing it.** Architecture, where the code really lives, how to add places/phrases/gestures — and the traps that cost us days. |
| [**CONTRIBUTING.md**](./CONTRIBUTING.md) | **Got an idea?** How to propose one (no code needed) or build it yourself. Start here. |
| [docs/faces.md](./docs/faces.md) | How greeting people by name works — the two cameras, enrolling, privacy. |
| [docs/voice-guide.md](./docs/voice-guide.md) | How the listening/talking half works, in depth. |
| [docs/movement-nav.md](./docs/movement-nav.md) | How mapping (SLAM) and driving (Nav2) work, in depth. |
| [plan.md](./plan.md) | The whole project on one page: what it is, what's built, what's next, what's out of scope. |

> **Changing the code? Read [`docs/developing.md`](./docs/developing.md) first.** There are three
> copies of the source on the robot and one of them is a fossil. That doc tells you which is which.

## What it is made of

```
 you ──speak──> mic array ──Whisper──> text
                                        │
                                   Claude (brain.py)    "which place do they mean?"
                                        │
                                    guide.py ──goal──> Nav2 ──> wheels
                                        │
                          lidar ────────┤ sees walls, people, tall things
                          depth camera ─┘ sees LOW things (chair legs)  ← the GREEN chain
                                        │
                                    Piper TTS ──speaks──> "We've arrived at the kitchen."
```

| Layer | What we use |
|---|---|
| Robot OS | ROS 2 Humble (Ubuntu 22.04) |
| Navigation | Nav2 + SLAM Toolbox, TEB controller |
| Reasoning | Claude (kept warm as a local daemon, ~1.6 s answers) |
| Speech | Whisper (listen) + Piper (speak) |
| Remote control | Small Python web app (`robot_web/`), opened by phone or laptop |

**The hardware** — and what to buy if a part ever needs replacing:

| Part | What it is |
|---|---|
| **Hiwonder JetRover** — *Tank chassis, no Jetson board* | The robot itself. Ships with the 6-DOF arm, depth camera, LiDAR (Slamtec A1), 7" LCD, 6-mic far-field array and speaker, IMU. |
| **Jetson Orin Nano** | The compute, mounted on the JetRover carrier. |
| **Battery (keep a spare)** | The arm + LCD + Jetson drain it in ~2 h. Two batteries let you hot-swap, so testing and demos never wait on a charge. |
| **Logitech C270 webcam** | The second camera, on the arm, that sees faces — see [docs/faces.md](./docs/faces.md). |

Where to buy: [JetRover product page](https://www.hiwonder.com/products/jetrover) ·
[JetRover Orin Nano docs](https://docs.hiwonder.com/projects/JetRover/en/jetson-orin-nano/) ·
[Amazon Developer Kit](https://www.amazon.com/HIWONDER-Education-Scenarios-Navigation-Developer/dp/B0DHVKF3WQ) ·
[RobotShop (Tank chassis / LiDAR A1)](https://www.robotshop.com/products/hiwonder-jetrover-ros-robot-car-with-vision-robotic-arm-powered-by-jetson-nano-support-slam-mapping-navigation-advanced-kit-tank-chassis-lidar-a1)
