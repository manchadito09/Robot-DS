# Collapse the three copies of the code into one

## Why
The code lives in three places today and they have to be synced by hand — the worst trap
in the project (you edit the wrong copy and nothing happens, silently). Goal: **one
folder that is both the git working tree and what the robot runs from.**

```
TODAY                                       TARGET
 ~/ros2_ws/           runs (ROS)             ~/robot-ds-clone/   = THE folder (git + runs)
 ~/robot-ds-clone/    git                       ~/ros2_ws            → symlink to it
 ~/Robot-DS/          web + voice (+ junk)      ~/Robot-DS/robot_web → symlink to it
                                                ~/Robot-DS/voice_prototype → symlink to it
```

## The idea: symlinks, not moving anything
The hardcoded paths (dozens of `~/ros2_ws/...`, and the systemd units pointing at
`~/Robot-DS/robot_web` and `~/Robot-DS/voice_prototype`) are **left untouched**. Instead
of moving files, the live paths become **symlinks** into the repo copy. That leaves one
real copy (the git one) with everything else pointing at it.

Big advantage: **reversible with one command** (remove the symlink, restore the `.orig`
folder). That is why it is symlinks and not a brute-force move.

## 🔴 Rule: in person, with the robot in front of you
This repoints live systemd services (`robot-web`, `stt-daemon`). If something breaks, the
robot loses its web app and its voice. Do it **at the robot, with a finger on STOP**, one
service at a time, verifying after each step. Not blind over SSH.

## Measured traps — respect them
1. **COLCON_IGNORE**: `~/robot-ds-clone/ros2_ws/COLCON_IGNORE` exists so that copy is not
   built. When unifying it must be **deleted** (otherwise colcon refuses to build).
2. **build/install/log**: after `colcon build` these appear inside
   `~/robot-ds-clone/ros2_ws/` — confirm they are in `.gitignore` so they never get
   committed.
3. **The voice_prototype `.venv`** is not in git and has absolute paths baked inside it.
   Symlinking would leave the venv behind. Safest: **recreate the venv** at the new
   location (`python -m venv` + `pip install -r requirements.txt`), do not move it.
4. **Drift**: before symlinking `~/ros2_ws`, the repo's source must be identical to the
   live `~/ros2_ws`. If there are live edits that were never copied across
   (`face_greet`, `prebake`…), they are lost. Run `diff -r` first and commit the
   difference. That is Phase 0.

## Steps

### Phase 0 — reconcile (CAN BE DONE REMOTELY)
- `diff -rq ~/ros2_ws ~/robot-ds-clone/ros2_ws` (ignoring `build/`, `install/`, `log/`, `.git`).
- Any real difference: copy from live into the repo and commit, until the source is
  **identical**. Without this, everything after it is dangerous.

### Phase 1 — the web app (IN PERSON)
- `mv ~/Robot-DS/robot_web ~/Robot-DS/robot_web.orig`
- `ln -s ~/robot-ds-clone/robot_web ~/Robot-DS/robot_web`
- `sudo systemctl restart robot-web` → **check the phone app** (STOP, camera, map).
- Broken? Remove the symlink, `mv robot_web.orig robot_web`, restart. Undone.

### Phase 2 — the ROS workspace (IN PERSON)
- Delete `~/robot-ds-clone/ros2_ws/COLCON_IGNORE`; confirm `build/ install/ log/` are in `.gitignore`.
- `mv ~/ros2_ws ~/ros2_ws.orig`
- `ln -s ~/robot-ds-clone/ros2_ws ~/ros2_ws`
- `colcon build --packages-select robot_ds_behavior` from `~/ros2_ws`.
- Verify: NAVIGATION starts, Show QR → `GREEN`, faces still respond.
- Broken? Remove the symlink, `mv ~/ros2_ws.orig ~/ros2_ws`, rebuild. Undone.

### Phase 3 — voice (IN PERSON)
- Recreate the venv inside `~/robot-ds-clone/voice_prototype/.venv` (gitignored).
- `mv ~/Robot-DS/voice_prototype ~/Robot-DS/voice_prototype.orig`
- `ln -s ~/robot-ds-clone/voice_prototype ~/Robot-DS/voice_prototype`
- `sudo systemctl restart stt-daemon` → check it hears you, and that faces still work.
- Broken? Undo the same way as the others.

### Phase 4 — clean up (only once everything is verified)
- Delete the `*.orig` folders and the dead junk still inside `~/Robot-DS` (`code/`,
  `sim_agent/`, `web.html`, the fossil `ros2_ws/`, PDFs, pptx) — the same junk already
  removed from the repo.

## Result

```
One real copy (the git one). You edit → git sees it → you commit right there.
No hand-syncing. The old paths keep working (symlinks). Reversible phase by phase.
```

## Out of scope
- Renaming folders or "properly" moving paths — that would break dozens of hardcoded
  paths. Symlinks first; the pretty rename, if ever, much later.
