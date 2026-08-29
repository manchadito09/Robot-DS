# voice_prototype/ — the robot's voice front-end (mic → text → speech)

> ### 🔴 Do not delete this folder. It is live.
> The name is historical — this started as a laptop prototype, and it is now what the
> robot actually listens with. Live things that break if it goes:
>
> | Depends on it | What breaks |
> |---|---|
> | `stt-daemon` (systemd, **active**) | Speech-to-text. The robot stops understanding you. |
> > | `robot_web/server.py` | The web app's **Talk to me**. |
> | `robot_ds_behavior/guide.py` | Narration during a trip. |
> | `voice_check.py` | The voice health check. |
>
> It lives on the robot at `~/Robot-DS/voice_prototype/` — see
> [three copies of the source](../docs/developing.md#-read-this-first-three-copies-of-the-source).

---

## What runs today

```
you speak → mic → STT (warm Whisper daemon, ~0.15 s) → brain → Piper speaks
                  stt_daemon.py + stt.py                        (in guide.py)
                  over /tmp/robot_ds_stt.sock
```

| File | Job |
|---|---|
| `stt_daemon.py` | **The warm Whisper daemon.** Keeps the model loaded and transcribes over a unix socket, instead of a cold load every time. Started by `stt-daemon.service`. |
| `stt.py` | The actual transcription (faster-whisper, VAD filter). The daemon wraps it; it is also the in-process fallback. |
| `listen_toggle.py` | "Listen once and give me the text." What `server.py` calls. |
| `record.py` | Mic capture → `audio.wav` (16 kHz mono). |
| `voice_robot.py` | Press-ENTER voice loop: ENTER, speak, ENTER → it acts. |
| `voice_robot_wake.py` | Hands-free: say "hey jarvis" → beep → speak → it acts. |
| `voice_nav.sh` | Glue: sources ROS and runs `brain.py --stay "<text>"`. (The one actually called is `~/voice_nav.sh`; this is the copy.) |
| `requirements.txt`, `setup_piper.sh` | How to rebuild the venv and the Piper voice on a fresh machine. |
| `.venv/` | Not in git. The STT daemon runs from this interpreter — if you recreate it, recreate it here. |

The robot **speaks** with Piper, from `guide.py` — not from this folder. Full picture in
[docs/voice-guide.md](../docs/voice-guide.md).

## Legacy — the laptop prototype it grew from

Kept because they still run and document how the pipeline was built, but nothing on the
robot calls them:

| File | What it was |
|---|---|
| `concierge.py` | The original laptop loop: press ENTER, speak, it replies. |
| `voice_talk.py` | Laptop mic → SSH to the sim box → robot brain → spoken reply. Superseded: on the robot everything runs on one machine. |
| `tts.py` | pyttsx3 speech (Windows SAPI voices). The robot uses **Piper** now. |
| `brain.py` | The laptop-side brain. The robot's brain is `ros2_ws/src/robot_ds_behavior/brain.py`. |
| `mic_check.py`, `test_1_oir.py` | Bench checks: list audio devices, live level meter, record-and-transcribe. |

## Audio files

| File | In git? | What it is |
|---|---|---|
| `beep.wav` | ✅ yes | The "I'm listening" tone `voice_robot_wake.py` plays after the wake word. An asset the robot needs — a fresh setup without it has no beep. |
| `audio.wav`, `greet_listen.wav`, `web_listen.wav` | ❌ never | **Recordings.** The mic dumps into these while the robot listens, so each one holds whatever the last person said. |

`.gitignore` ignores `*.wav` and makes `beep.wav` the one explicit exception, so a
careless `git add .` can't publish somebody's voice. If you add a new recording buffer,
it is ignored automatically; if you add a new *asset*, add an exception for it.

> Same principle as [faces.md](../docs/faces.md#privacy): we keep what the robot needs to
> work, not recordings of the people it works with.

## Gotchas

- **Mic too quiet → Whisper hallucinates** (invents sentences from near-silence). Find a
  working input with `mic_check.py` and set its index in `record.py` (`MIC_DEVICE`). The
  VAD filter in `stt.py` suppresses most of it.
- **Do not move the `.venv`** — a virtualenv has absolute paths baked in. Recreate it in
  place instead (`python3 -m venv .venv` + `pip install -r requirements.txt`).
- **The daemon is already running.** You do not start it by hand; `stt-daemon.service`
  keeps it warm so the first request is fast. `systemctl status stt-daemon` to check.
