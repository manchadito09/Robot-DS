# Developing the robot

For anyone about to change the robot. Read the first two sections before you touch
anything — they are where the days go.

- [🔴 Read this first: three copies of the source](#-read-this-first-three-copies-of-the-source)
- [🔴 What needs a build and what does not](#-what-needs-a-build-and-what-does-not)
- [How it all fits together](#how-it-all-fits-together)
- [Recipes: how to change things](#recipes-how-to-change-things)
- [The vision chain](#the-vision-chain)
- [The arm](#the-arm)
- [Diagnostic tools](#diagnostic-tools)
- [Traps that have cost us days](#traps-that-have-cost-us-days)

---

## 🔴 Read this first: three copies of the source

There are **three** copies of this code on the robot. They are not interchangeable, and
one of them is a fossil. Edit the wrong one and your change simply does nothing, silently,
for as long as you are willing to stare at it.

```
~/ros2_ws/            ← THE ROBOT RUNS FROM HERE.
                        Edit here. Build here. This is the live ROS workspace.

~/robot-ds-clone/     ← GIT LIVES HERE.
                        Commit and push from here. Keep it in sync with ~/ros2_ws by hand.

~/Robot-DS/           ← MIXED. Careful.
    robot_web/        ← ✅ LIVE. The web app really runs from here (systemd robot-web).
    voice_prototype/  ← ✅ LIVE. The STT daemon runs from here (the web app's Talk to me).
    ros2_ws/src/      ← 💀 FOSSIL. Months out of date. Ignore it. Never edit it.
    code/ sim_agent/ web.html … ← 💀 old junk, same as the repo used to carry.
```

> ### 🔴 Never delete `~/Robot-DS`
> It *looks* like an outdated copy — most of what's inside it is dead junk. But
> **`robot_web/` and `voice_prototype/` run live from here** (systemd `robot-web` and
> `stt-daemon`). Delete the folder and you kill the web app and voice recognition in one
> move -- and with the web app goes the greeter, which it starts and revives. Only `~/Robot-DS/ros2_ws/src` is the
> fossil. This split is an accident of history, not a design — the plan to collapse it
> into one folder is in [`plans/unify-the-copies.md`](../plans/unify-the-copies.md), to be
> done in person at the robot (it repoints live services).

To see the fossil for yourself:

```bash
wc -l ~/ros2_ws/src/robot_ds_behavior/robot_ds_behavior/guide.py          # ~690 lines: the real one
wc -l ~/Robot-DS/ros2_ws/src/robot_ds_behavior/robot_ds_behavior/guide.py # ~295 lines: the fossil
```

### 🔴 Always `colcon build` from `~/ros2_ws` — never from `~`

The three copies bite the build, not just the edit. `colcon build` from your home
directory scans ALL of them, sees the same package name three times, and dies with
`Duplicate package names not supported` — having built nothing. It cost a real
build once. So:

```bash
cd ~/ros2_ws && colcon build --packages-select robot_ds_behavior   # ✅ sees one copy
cd ~        && colcon build --packages-select robot_ds_behavior   # ❌ sees three -> fails
```

As a safety net on THIS robot, the two non-live copies (`~/Robot-DS/ros2_ws` and
`~/robot-ds-clone/ros2_ws`) carry an empty `COLCON_IGNORE`, so colcon skips them
even from `~`. Those markers are local and git-ignored on purpose: a fresh clone of
this repo has only ONE copy and must stay buildable, so the ignore file must never
be committed.

**The rule:**

| You are changing… | Edit it in… | Then commit it from… |
|---|---|---|
| Robot behaviour (`guide.py`, `brain.py`, launch files, nav2 YAML) | `~/ros2_ws/` | `~/robot-ds-clone/` |
| The web app (`server.py`, `index.html`) | `~/Robot-DS/robot_web/` | `~/robot-ds-clone/` |
| Standalone tools (`arm_gesture.py`, `waypoint_tool.py`, …) | `~/ros2_ws/` | `~/robot-ds-clone/` |

Copy your change into `~/robot-ds-clone/` and commit it there. Nothing pushes itself.

---

## 🔴 What needs a build and what does not

`guide.py` and `brain.py` run **from the colcon build**, not from the file you just
edited. Forgetting this is the single most repeated mistake on this robot: the file on
disk is right, the robot is running the old one, and everything you conclude from that
point on is wrong.

```bash
colcon build --packages-select robot_ds_behavior     # after editing guide.py / brain.py
```

| File | Needs a build? | Takes effect… |
|---|---|---|
| `guide.py`, `brain.py`, `talk.py`, `pois.py` | 🔴 **YES** | after `colcon build` |
| `office_knowledge.yaml` (facts, jokes) | No | next trip |
| nav2 YAMLs (`nav2_params.yaml`, `nav2_controller_teb.yaml`) | No — read from source | next `NAVIGATION` start |
| The map + waypoints | No | next trip |
| `arm_gesture.py`, `arm_probe.py`, `*.d6a` gestures | No — standalone scripts | immediately |
| `index.html` | No | when the browser reloads |
| `server.py` | No build, but **restart** `robot-web` | after `sudo systemctl restart robot-web` |

**After editing spoken lines**, re-bake them or Piper synthesises them live while driving
(which is exactly what pre-baking exists to avoid):

```bash
python3 ~/ros2_ws/prebake_lines.py
```

---

## How it all fits together

```
  voice ──> mic array ──> listen_toggle.py (Whisper) ──> text ──┐
                                                                │
  web app (robot_web/) ── typed text ───────────────────────────┤
                                                                v
                                                    brain.py  ── route() ──> Claude
                                                                │            (warm daemon,
                                                                │             ~1.6 s)
                                        ┌───────────────────────┤
                                        │                       │
                                  action = "say"          action = "go" / "tour"
                                        │                       │
                                     Piper                   guide.py
                                     speaks                     │
                                                          NavigateToPose
                                                                v
                                                              Nav2  ──> wheels
                                                                ^
                              lidar  ──/scan──────────────────┐ │  walls, people, tall things
                                                              ├─┘
        depth cam ──> camera_scan_node ──/camera_scan─────────┘    LOW things (chair legs)
```

**The pieces:**

| Piece | Where | Job |
|---|---|---|
| `brain.py` | `ros2_ws/src/robot_ds_behavior/` | Understands the request. Asks Claude which place is meant, or answers a question. |
| `guide.py` | same | Drives one trip: sends the Nav2 goal, narrates on the way, announces the arrival. |
| `office_knowledge.yaml` | `ros2_ws/` | What the robot knows about the office — facts, descriptions, jokes. Editable without a build. |
| `camera_scan_node.py` | `ros2_ws/` | Turns the depth image into a fake 2D laser at floor level. **This is what stops the robot hitting chair legs.** |
| `claude_daemon.py` | `ros2_ws/` | Keeps Claude warm (~1.6 s per answer instead of ~10 s). Falls back to `claude -p`. |
| `server.py` + `index.html` | `Robot-DS/robot_web/` | The web app, plus the camera watchdog. |
| `arm_gesture.py` | `ros2_ws/` | Plays an arm gesture by name. Standalone, no build. |
| `face_greet.py` + `face_lib.py` | `ros2_ws/` | Recognises people and greets them by name. Talks only, never drives. See [faces.md](./faces.md). |

**Two brains, on purpose.** The robot can answer a question *while* it is walking you
somewhere. The trip runs one process; the answer runs a second `brain.py` launched with
`--no-drive`, which may talk but may never send a Nav2 goal. Two goals at once would mean
abandoning the visitor mid-corridor. The web checks the installed brain really understands
`--no-drive` before offering this — a brain built before the flag silently ignores it and
drives.

**Navigation is TEB, not DWB.** `use_teb` defaults to `true`, so **`nav2_controller_teb.yaml`
is the file that matters**. `nav2_controller_dwb.yaml` is not used — editing it does nothing.

---

## Recipes: how to change things

### Add a place the robot can go to

Use the web app (**Map → Add a new point**). It saves the point *and* asks what the place
is, which is what the robot says on arrival and uses to answer questions. From a terminal:

```bash
python3 ~/ros2_ws/waypoint_tool.py save "Design team"    # saves the robot's current spot
python3 ~/ros2_ws/waypoint_tool.py del  "Design team"
```

### Change what it knows or says about a place

Edit `~/ros2_ws/office_knowledge.yaml`. No build. Jokes live under the `jokes` key.

```bash
python3 ~/ros2_ws/prebake_lines.py     # only if you changed FIXED spoken lines
```

### Add an arm gesture

Gestures are SQLite files (`.d6a`) in `~/software/arm_pc/ActionGroups/`, one row per step:

```
ActionGroup(Index, Time, Servo1..Servo6)      Time = ms for that step
```

Servo columns map to bus IDs `1,2,3,4,5,10` (Servo6 is the gripper, ID 10). **Measured safe
ranges** (measured on the real arm, with a finger on the stop button — do not exceed them
without measuring again):

| Servo | Range | What it does |
|---|---|---|
| S1 base | free (no collision) | rotates the whole arm |
| S2 shoulder | 600 – 765 | 765 = rest, 600 = up |
| S3 elbow | 15 – 500 | **this is what lifts the arm.** Not the shoulder. |
| S4 wrist | 220 – 425 | tilts up/down |
| S5 wrist roll | centre 500 | spins the hand |
| S6 gripper | 0 open … 1000 closed | the "mouth" |

Write the file, then map the name in `arm_gesture.py` (`GESTURES` dict). No build.

> **Ramp your moves.** Driving the elbow 15 → 500 in one step makes the whole robot hop.
> Split every rise and return into two steps (15 → 260 → 500). Every gesture in the repo
> does this.

`arm_probe.py` finds a servo's real limit safely: it moves **one** servo **one small step**
per keypress, so you can stop before anything collides.

### Add or fix a face the robot greets

Use the phone app (**Add me**) and **scan** — look at the camera and slowly turn your
head. Photos give it one angle and the match wobbles; a head-turn scan is what makes it
solid. Full detail — models, thresholds, privacy, the self-learning locks — is in
**[faces.md](./faces.md)**.

```bash
python3 ~/ros2_ws/face_enroll.py     # enroll / re-enroll from a terminal
python3 ~/ros2_ws/face_check.py      # health check before a demo
bash    ~/ros2_ws/get_face_models.sh # fetch the .onnx models (kept out of git)
```

### Edit the map

```bash
python3 ~/ros2_ws/map_crop.py      # erase / line / clean / autocrop
```

⚠️ Saving a map from SLAM (`map_save`) **regenerates the whole file** and overwrites your
edits, and it always writes `map_01`. Changing the map origin invalidates every saved
waypoint — they have to be re-recorded.

---

## The vision chain

The lidar sweeps at ~16.8 cm. Chair legs, table feet and bin edges live **below** it. The
depth camera is the only thing that sees them.

```
depth driver ──> /depth_cam/depth/points ──> camera_scan_node ──> /camera_scan ──> costmap
     ^                                             ^
 video for the phone                    the part that stops it hitting chair legs
```

**Half this chain looks exactly like all of it.** The driver alone gives you video on the
phone and a robot that is blind to anything low. This has cost us a morning more than once.

```bash
bash ~/ros2_ws/camera_check.sh      # probes AND repairs the whole chain. 0=GREEN 2=BLOCKED 1=RED
pgrep -af camera_scan_node          # must print exactly ONE line. Empty = blind. Two = possessed.
```

`camera_check.sh` is safe to run when the camera is already up: it probes first and only
touches what is broken. The `Show QR (phone app)` icon runs it for you.

**Why our own node, and not `pointcloud_to_laserscan`:** the off-the-shelf node filters by
height using the arm's TF, and the arm does not repeat its angle to the degree needed. It
decides the floor is high, marks *the floor* as an obstacle, and the robot spins in a cage
of its own making. `camera_scan_node.py` ignores the TF and fits the real floor plane live,
every frame. In its log, `live floor fit active` is good; `fallback … deg` means it has not
seen enough floor — usually the arm is not in the scan pose.

**The camera watchdog** (in `server.py`) subscribes to `/camera_scan` — the *end* of the
chain, so one topic proves the whole path. If it goes quiet:

- **always:** run `camera_check.sh` and bring the camera back (one repair at a time).
- **during a trip:** also stop the robot and say so. It will not resume on its own.

To drive deliberately without a camera (otherwise the watchdog stops every trip a few
seconds in):

```bash
sudo mkdir -p /etc/systemd/system/robot-web.service.d
echo -e '[Service]\nEnvironment=ROBOT_CAM_WATCHDOG=0' | \
  sudo tee /etc/systemd/system/robot-web.service.d/nocam.conf
sudo systemctl daemon-reload && sudo systemctl restart robot-web
# undo: sudo rm -rf /etc/systemd/system/robot-web.service.d && sudo systemctl daemon-reload && sudo systemctl restart robot-web
```

**Two filters that erased chair legs.** A chair leg is ~1 costmap cell, and two separate
"noise" filters threw it away as too small: nav2's `denoise_layer` (now `enabled: False`)
and TEB's DBSCAN costmap converter (now `costmap_converter_plugin: ""`). The camera was
never lying — nothing was listening. If lidar ghosts ever appear, try
`minimal_group_size: 1` rather than turning `denoise_layer` back on.

---

## The arm

The arm carries the depth camera, so **where the arm points is where the robot can see**.

- Every trip calls `set_scan_pose()` before moving, so gestures cannot leave the camera
  aimed at the ceiling. Play as many as you like.
- The scan pose needs **servos**, which need the **battery**. On cable power it fails and
  says so — and then the floor fit falls back, phantom obstacles appear, and the robot
  cages itself in an empty corridor.
- Autonomous manipulation is Phase 2 and out of scope for v1. Gestures are expression only.

---

## Faces: "Hi Jonny"

A second camera — a Logitech C270 on the arm, angled up — watches for faces while the depth
camera keeps looking at the floor. One arm, two jobs, because the two cameras are bolted at a
fixed angle to each other: put the arm in its scan pose and the depth camera sees the floor
*and* the C270 sees a standing person's face.

```
C270 --> YuNet --> SFace --> faces.json --> "Hi Jonny! Tap Talk to me on the app."
 3 fps  detect   128 nums   fingerprints              |
                                                      v
                                            back to watching -- it does not listen
```

**The greeter has no microphone code.** It used to hold a whole conversation, and the listening
half never became reliable: the far-field array's automatic gain drifts, and its opening
transient reads as speech to any voice detector (measured — the numbers are in
[faces.md](./faces.md#why-it-does-not-listen)). Talking back moved to the web app's **Talk to
me**, where the phone's own mic does the listening. Deleted rather than switched off: an off
switch for a broken feature is still a broken feature to maintain.

**A greeting can never steal a trip.** It speaks with `--optional`, so `guide.say()` DROPS the
hello rather than talk over a real answer or the narration of a walk somebody is on. And it
never sends a Nav2 goal, so it cannot abandon a visitor mid-corridor.

| Piece | What it does |
|---|---|
| `face_lib.py` | Camera, models, the fingerprint store. No ROS imports — a wedged ROS cannot take it down. |
| `face_enroll.py` | Enrol from the command line. `--list`, `--remove`. |
| `face_greet.py` | The watcher: look, recognise, greet. `--once` to test, `--dry` to print instead of speak. |
| `models/get_face_models.sh` | Fetches the two ONNX models (not in git: 37 MB of unchanging binary). |
| Web **Add me** | The same enrolment, self-service, with a live mirror. `server.py` + `index.html`. |

**Why OpenCV's own models.** OpenCV 4.10 already ships YuNet and SFace. dlib / `face_recognition`
would mean a long, fragile ARM compile for nothing. Two ONNX files and no new library.

**Why enrol from the robot's camera, not uploaded photos.** A selfie is level, bright and
high-resolution. This robot looks *up* at you from 45 cm through a 640x480 webcam. Enrol through
the lens that has to do the recognising. It also means no employee photo is stored anywhere —
only the fingerprint.

**The rules that keep it cheap and polite:**

- **3 fps, threads capped** (`face_lib` sets `OMP_NUM_THREADS` *before* importing cv2 — see the
  OpenBLAS row in the traps table).
- **`speak.py --optional`**, so `guide.say()` DROPS a greeting when the robot is already talking.
  A hello must never step on an answer or on the narration of a trip.
- **One greeting per person per 5 minutes.** Without it, standing near the robot means being
  greeted three times a second.
- **It only talks.** No driving, no arm, no Nav2.

**One camera, one owner.** The greeter and the web's mirror both want `/dev/video*`, and two
processes cannot stream one webcam. The web opens it on demand and releases it 8 s after you
leave the screen. If you later run `face_greet.py` as a service, give it the camera and have the
web ask *it* for frames — do not open the device twice.

## Diagnostic tools

All in `~/ros2_ws/`. The ladder below is what actually solved the chair-leg case: climb it,
and the first rung that fails names the culprit.

| Tool | Answers |
|---|---|
| `camera_check.sh` | Is the whole camera chain up? (and repairs it) |
| `low_obstacle_test.py` | Does the **camera** see the chair leg? (`0/0 sweeps` = nothing arriving, not "cannot see it") |
| `costmap_front_check.py` | Does the mark reach the **local** costmap? (what TEB uses) |
| `global_front_check.py` | Does it reach the **global** costmap? (what the planner uses) |
| `floor_probe.py` | Is the floor being fitted correctly, or extrapolated into ghosts? |
| `nav2_errors.sh` | How many Nav2 failures — a **number**, not a feeling |
| `pose_check.py` | Is the robot really where AMCL claims? (AMCL always publishes a pose, even a wrong one) |
| `scan_vs_map.py` | What fraction of lidar rays the map explains — the glass-wall detector |
| `costmap_view.py` | The costmap as text, no RViz (RViz costs ~45 % CPU) |
| `camera_scan_count.py` | How many bearings is `/camera_scan` actually publishing? (read-only) |
| `near_range_check.py` | In the scan pose, how **close** and how **far** does the camera see? |
| `localization_watch.py` | Why does the robot teleport? Watches the pose for jumps |
| `demo_check.sh` | One-shot pre-demo verdict |
| `face_check.py` | Is the face pipeline healthy? Cameras, models, store, greeter — see [faces.md](./faces.md) |

All of the probes above are **read-only** — they look, they never move the robot.

**Two older scripts are kept but superseded.** They still run; they are just not the ones
to reach for:

| Superseded | Use instead |
|---|---|
| `prebake_voice.py` | `prebake_lines.py` — the one the startup scripts and `server.py` call |
| `explore_simple.py` | `explore.py` / `explore_auto.py` — used by the auto-mapping stack |

---

## Traps that have cost us days

| Trap | What actually happens |
|---|---|
| **Editing the fossil** (`~/Robot-DS/ros2_ws/src/`) | Your change does nothing. Ever. See the top of this doc. |
| **Forgetting `colcon build`** | `guide.py`/`brain.py` run from the build. The robot runs the old code while you read the new one. |
| **Trusting the phone's video** | Video ≠ obstacle detection. Half the chain looks identical to all of it. |
| **`kill -9` on the camera** | Wedges the depth sensor. Use SIGINT and wait. |
| **Ctrl-C in the terminal that started the camera** | SIGINT hits the whole process group and takes the camera with it. This is why it "died on its own". Everything is launched with `setsid nohup … </dev/null &` now. |
| **No battery** | No servos, no lidar. The arm cannot aim the camera → phantom obstacles. Check for 11–12.6 V, not ~3.8 V. |
| **Two `camera_scan_node`s** | Two `/camera_scan` streams into one costmap, out of step. The robot behaves like it is possessed. `camera_scan.sh` kills any existing one first. |
| **Editing `nav2_controller_dwb.yaml`** | Not used. The controller is TEB. |
| **A Python process over 100 % CPU** | Count its threads before anything else. `camera_scan_node` once ate the Jetson three times over — it was OpenBLAS spinning six threads in a busy-wait, not the workload. 374 % → 33 %. |
| **Glass walls** | The lidar goes straight through. Rays land in unmapped grey, AMCL slides, the robot drifts into the wall. The fix was **filling the room in on the map** (31.5 % → 93.2 % of rays explained), not AMCL tuning. |
| **"Zero ghosts" as proof** | A blind camera also produces zero ghosts. Always test **both**: that it avoids a real obstacle *and* that it does not invent one in an empty corridor. |
| **`except Exception:` → feature off** | It hides the real error. "Add me" shipped broken because `server.py` had no `import sys`; the only symptom was a button that never appeared. An optional feature may switch itself off — it may not do it **silently**. Print why. |
| **`stdout=DEVNULL` on a worker** | The greeter ran, held the camera, and greeted nobody — and could not be debugged, because its output went to /dev/null. It now logs to `journalctl -u robot-web`, and says what it *sees* ("nobody" / "too far" / "a face I don't know" / a cooldown), because those four need four different fixes. A background process that cannot say what it is doing cannot be fixed. |
| **OpenCV cannot open a camera by path** | `VideoCapture("/dev/v4l/by-id/…", CAP_V4L2)` fails with "can't be used to capture by name" and hands back a closed capture. But the index moves. So: find it by serial via `by-id`, `realpath` it, and pass the *number* — stable identification, working capture. |
| **Device numbers** (`/dev/video2`, `card 1`) | They move. Plugging the C270 in gave its microphone card 1 and pushed the speaker to card 2 — harmless only because the voice already asks for `CARD=Device` by NAME. Use `/dev/v4l/by-id/…` and card names, never indices. |
| **USB power, not USB bandwidth** | The C270 kept enumerating and dropping. It was not the crowded bus: it was three chained hubs with no battery in, so the volts sagged. Battery in → rock solid. Suspect power before bandwidth. |

---

## Working agreement

- Branch per task. `main` is protected — merge through a reviewed PR, never commit directly.
- Humans place hardware orders — never order parts on someone's behalf.
- Measure, do not assume. Most of the table above is there because someone assumed.
