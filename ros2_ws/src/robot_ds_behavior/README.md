# robot_ds_behavior

The robot's **brain + legs**: turn a request into autonomous navigation — the part
that decides where to go and drives there.

## Files

| File | What it does |
|---|---|
| `pois.py` | The list of places (POIs) in the scene's **world** frame, plus the **world→map calibration** (`MAP_YAW`, `MAP_OFFSET`). Edit coordinates here, not in the code. |
| `guide.py` | The **Guide node**: turns a destination name into a Nav2 `NavigateToPose` goal, **narrates** the trip (halfway / almost there / arrival), and **returns to base**. `go()` = one trip, `lead()` = trip + return. |
| `brain.py` | **Claude picks a POI** from a free-form sentence (`claude -p`), then guides there. One-shot. `pick_poi()` lives here (single source of truth). |
| `talk.py` | Same brain, **interactive loop**: type orders one after another, the robot stays where it arrives. |
| `explore.py` | **Autonomous frontier exploration** to map the whole floor so every POI becomes reachable. |

## Run (with Nav2 + SLAM running)

```bash
python3 guide.py kitchen          # go to a named POI, then return to base
python3 guide.py kitchen --solo   # go, don't return (debug)
python3 brain.py "i'm hungry"     # Claude picks the POI, one-shot
python3 talk.py                   # conversational loop ("i'm hungry" -> kitchen)
python3 explore.py                # map the whole floor
```

## World frame vs map frame

On the robot this is the **identity**: destinations are tagged straight in the SLAM
**map** frame, so `guide.py` passes them to Nav2 unchanged (`MAP_YAW=0`,
`MAP_OFFSET=(0,0)` in `pois.py`). The conversion below only mattered when goals came
from a separate world frame:

```
map = R(MAP_YAW) · world + MAP_OFFSET
```

## Real robot notes

- Hardware-agnostic: the robot publishes `/scan` and takes `/cmd_vel` natively; only
  the map and the destination coordinates change.
- `say()` in `guide.py` speaks with **Piper** (espeak fallback) — see
  [voice-guide.md](../../../docs/voice-guide.md).
