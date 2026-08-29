# Voice Guide — how the talking guide robot works

The robot listens, understands a free-form request, **reasons** which mapped place
you mean, drives there with Nav2, and narrates out loud in a natural voice. This
doc is the map of that system: what each piece is and how to run it.

---

## 1. The pipeline

```
  you speak → mic → STT → brain: pick a place → guide: drive → TTS: speak → speaker
              6-mic  │      │                    guide.py       guide.say()   plughw:1,0
                     │      │                    (navigate_to_pose)  (Piper → aplay)
                     │      └ brain.py, reasons via the WARM Claude daemon
                     │        (claude_daemon.py, socket, ~2 s; falls back to `claude -p`)
                     └ the WARM Whisper daemon (stt_daemon.py, socket, ~0.15 s;
                       falls back to loading the model in-process)
```

- **Two warm daemons carry the speed.** Whisper and `claude` are each kept hot in a
  background process (systemd `stt-daemon` and `claude-daemon`) and answer over a unix
  socket, instead of cold-starting every request. If a daemon is down, the code falls
  back to the slow path on its own — the robot never breaks, it just gets slower. The
  remaining ~5 s end-to-end is Whisper + Claude + Piper, already squeezed.
- **Same pipeline, several front-ends.** The web app's **Talk to me**, the voice scripts
  (see section 3), and the **face greeter** ([faces.md](./faces.md)) all feed this same
  STT → brain → Piper path.
- **Destinations = the named waypoints you save while mapping.** They are the single
  source of truth; the brain reasons over their names (see section 4). Saved by
  `waypoint_tool.py` into `map_01.waypoints.yaml`, already in the Nav2 `map` frame.
- **The brain reasons** — it is NOT a hardcoded "hungry→kitchen" table. It reads the
  real waypoint names and infers the best match from world knowledge, so it keeps
  working if you rename or add waypoints.

---

## 2. Files

Robot brain (ROS 2 package `robot_ds_behavior`, runs in the ROS env):

| File | What it does |
|---|---|
| `robot_ds_behavior/brain.py` | `pick_poi(text)` → asks Claude **via the warm `claude-daemon`** (falls back to `claude -p`) which waypoint the request means (or `none`). `main()` runs the guide with `--stay`. |
| `robot_ds_behavior/guide.py` | `Guide` node: sends the goal to Nav2 (`navigate_to_pose`), narrates via `say()`, returns to base. `say()` = **Piper** TTS (espeak fallback). `load_places()` reads the waypoints. |
| `robot_ds_behavior/talk.py` | Typed loop: you type instead of speaking. |
| `robot_ds_behavior/pois.py` | Fallback sim POIs + world→map calibration (only used if no waypoints file). |
| `waypoint_tool.py` | Save/pick/list/go/del named waypoints → `map_01.waypoints.yaml`. |

Voice front-end (`voice_prototype/`, runs in its own Python venv):

| File | What it does |
|---|---|
| `voice_robot_wake.py` | **Hands-free**: say "hey jarvis" → beep → speak → it acts. (openWakeWord) |
| `voice_robot.py` | **Press-ENTER**: ENTER, speak, ENTER → it acts. |
| `record.py` | Mic capture → `audio.wav`. |
| `stt.py` | `audio.wav` → text (faster-whisper). This is the **in-process fallback**; on the robot the warm daemon below does it. |
| `tts.py` | Standalone TTS helper (espeak/pyttsx3) — the robot narration itself lives in `guide.py`. |
| `concierge.py` / `voice_talk.py` | Older laptop-only prototypes (no robot / SSH bridge). Kept for history. |
| `~/voice_nav.sh` | Glue: sources ROS and runs `brain.py --stay "<text>"`. The voice scripts call this. Lives in **home** (`~/`); a copy sits in `voice_prototype/`. |
| `setup_piper.sh` / `requirements.txt` | Reproduce the STT/TTS deps on a fresh machine: `pip install -r requirements.txt`, then `./setup_piper.sh`. |

Warm daemons — where the speed comes from (run as systemd services, always hot):

| File | Where | What it does |
|---|---|---|
| `stt_daemon.py` | `~/Robot-DS/voice_prototype/` (service `stt-daemon`) | Keeps Whisper loaded and transcribes over a socket (`/tmp/robot_ds_stt.sock`) in ~0.15 s, instead of a cold model load each time. |
| `claude_daemon.py` | `~/ros2_ws/` (service `claude-daemon`) | Keeps `claude` warm and answers over a socket (`/tmp/robot_ds_claude.sock`) in ~2 s, instead of ~10 s to cold-start `claude -p`. Falls back to `claude -p`. |

> **Mind the copies.** The robot behaviour runs live from
> `~/ros2_ws/src/robot_ds_behavior/`; this repo mirrors it under
> `ros2_ws/src/robot_ds_behavior/` — edit the live one and copy it back by hand. The
> `voice_prototype/` folder also runs **live from `~/Robot-DS/voice_prototype/`** (systemd
> `stt-daemon`), and this repo mirrors it too. See
> [developing.md — three copies](./developing.md#-read-this-first-three-copies-of-the-source),
> and **never delete `~/Robot-DS`**.

---

## 3. How to run it (on the robot)

Prereqs every time:
1. **Stop the gamepad service** (it fights Nav2 for the hardware):
   `sudo systemctl stop start_app_node.service`
2. **Launch navigation** with `map_01`, wait ~15 s, then check it is up:
   `ros2 action list | grep navigate_to_pose`  → must print `/navigate_to_pose`.
3. **Localize** the robot in RViz with **2D Pose Estimate**.

Then pick one:
```bash
cd ~/Robot-DS/voice_prototype
.venv/bin/python voice_robot_wake.py     # hands-free: say "hey jarvis", then your request
.venv/bin/python voice_robot.py          # press ENTER, speak, ENTER
# or, typed (no mic), in the ROS env:
python3 ~/ros2_ws/src/robot_ds_behavior/robot_ds_behavior/talk.py
```
Say things in **English**: "take me to the kitchen", "I'm hungry", "I need to answer emails".

> You don't start the warm daemons — `stt-daemon` and `claude-daemon` run as systemd
> services and are already hot, so the very first request is fast.

---

## 4. Destinations & the reasoning brain

- Save a place while mapping:
  `python3 ~/ros2_ws/waypoint_tool.py save kitchen`   (or `pick <name>` to click it in RViz)
- List / go / delete: `... list` / `... go kitchen` / `... del kitchen`
- Stored in `~/ros2_ws/src/slam/maps/map_01.waypoints.yaml`. Each entry is
  `{x, y, yaw}` in the Nav2 `map` frame.
- The brain (`brain.pick_poi`) lists those names to Claude and asks it to reason the
  best match, or reply `none`. On `none` the robot says *"Sorry, I didn't get that…"*
  and does **not** move — it never guesses a wrong room.

---

## 5. The voice (TTS)

- The robot speaks with **Piper** (neural, natural), voice `en_US-ryan-medium` (male).
- Implemented in `guide.py` `say()` → `_say_piper()`:
  `piper --model <voice>.onnx --output_raw | aplay -r 22050 -f S16_LE -t raw -D plughw:1,0`.
- espeak stays as an automatic fallback if Piper is missing.
- Change the voice: swap the `.onnx` or set `ROBOT_PIPER_MODEL`. Downloaded voices live
  in `voice_prototype/piper/models/` (see `setup_piper.sh` for more).

---

## 6. Config knobs (env vars)

| Var | Default | Meaning |
|---|---|---|
| `ROBOT_SPEAKER` | `plughw:1,0` | ALSA output device (the USB speaker = card 1). |
| `ROBOT_PIPER_MODEL` | `…/en_US-ryan-medium.onnx` | TTS voice model. |
| `ROBOT_WAKEWORD` | `hey_jarvis` | Wake word (`alexa`, `hey_mycroft`, `hey_rhasspy`). No "hey robot" without custom training. |
| `ROBOT_WAKE_THRESHOLD` | `0.5` | Wake sensitivity (higher = less sensitive). |
| `MIC_DEVICE` (in the scripts) | `0` | The 6-mic array. Find yours with `mic_check.py`. |

---

## 7. Gotchas (learned the hard way)

- **Wireless vs Nav2** — the gamepad service `start_app_node.service` grabs the LiDAR/motors.
  Stop it before running voice/navigation.
- **STT language** — Whisper is forced to English in `voice_robot*.py` (`STT_LANG="en"`)
  because the demo is English; letting it auto-detect made it hallucinate random
  languages. Speak English, or set it to `es` if you want Spanish input.
- **onnxruntime warning / hangs** — the "GPU device discovery failed" line is silenced
  only at *import* time. Do NOT redirect stderr around `transcribe()` — that hangs it.
- **Mic level** — the array is index 0; a healthy capture reads ~0.3–0.5 peak. If it
  can't hear you it says "didn't catch that".
- **DDS** — the brain must run from a terminal that sourced the normal robot env
  (`~/ros2_ws/.robotrc`); `~/voice_nav.sh` sources ROS for it. RMW is FastDDS (default).

---

## 8. The companion app (built)

The voice path above now sits next to a **web app**. On the robot, double-click **Show
QR (phone app)** and scan the QR with a phone, or open it on a laptop on the same
network. It sends the same requests as voice, plus **Map** (tap to go / save a point),
**Tour**, **Arm**, **Add me** (greeting), and a big red **STOP**. It runs from
`robot_web/` (systemd `robot-web`). This is the "by voice **or** touchscreen" half of the
guide robot — see [operating.md](./operating.md) for how to use it.
