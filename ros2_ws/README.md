# ros2_ws/ — the ROS 2 workspace

Everything the robot's body and brain run on: our behaviour package, the vendor packages
we had to tune, the floor map, and the loose tools we use to diagnose it.

> ### 🔴 This is a mirror, not the live workspace
> The robot builds and runs from **`~/ros2_ws/`**. This copy exists so the work is
> versioned — keep them in sync by hand. And `guide.py` / `brain.py` run from the
> **colcon build**, not from the file you just saved:
>
> ```bash
> colcon build --packages-select robot_ds_behavior     # after editing guide.py / brain.py
> ```
>
> Full rules — which copy to edit, what needs a build — in
> [developing.md](../docs/developing.md#-read-this-first-three-copies-of-the-source).

## What's ours vs what's the vendor's

The JetRover ships with ~15 ROS packages. **Only the ones we actually changed are kept
here** — the untouched vendor stack stays on the robot.

| Under `src/` | Whose | What we changed |
|---|---|---|
| `robot_ds_behavior/` | **ours** | The brain and legs: `guide.py`, `brain.py`, `talk.py`, `explore.py`. See [its README](src/robot_ds_behavior/README.md). |
| `navigation/` | vendor, tuned | The Nav2 params and the TEB controller tuning — this is where the robot's driving behaviour actually lives. `nav2_controller_dwb.yaml` is **not** used (the controller is TEB). |
| `slam/` | vendor, tuned | SLAM launch files, the auto-mapping stack, and **the floor-4 map itself** (`maps/map_01.*` + the named waypoints). |
| `bringup/` | vendor, tuned | The launch files and the desktop scripts that start everything (`scripts/navigation.sh`, `slam.sh`, …). |
| `peripherals/`, `driver/` | vendor, one file each | The depth-camera launch (`dabai_dcw.launch.py`, where the depth fps is capped) and the odometry publisher. |

## The loose files at the top

Standalone scripts — **no `colcon build` needed**, they take effect immediately.

| | |
|---|---|
| **Runs the robot** | `camera_scan_node.py` (depth → `/camera_scan`, the chair-leg detector), `claude_daemon.py` (warm Claude), `arm_gesture.py` (plays a gesture), `speak.py`, `waypoint_tool.py` (named destinations), `prebake_lines.py` (bake spoken lines to WAV) |
| **Knowledge / config** | `office_knowledge.yaml` (what it knows and says about each place — no build, no restart), `camera_calib.yaml` (measured against the arm's scan pose — see [arm/](../arm/README.md)) |
| **Faces** | `face_lib.py`, `face_greet.py`, `face_check.py`, `face_enroll.py` — see [faces.md](../docs/faces.md) |
| **Diagnostics** | `camera_check.sh` (probes *and repairs* the vision chain), `demo_check.sh` (one-shot pre-demo verdict), `floor_probe.py`, `costmap_view.py`, `scan_vs_map.py` (the glass-wall detector), `pose_check.py`, `nav2_errors.sh` (failures as a **number**, not a feeling) |
| **Map editing** | `map_crop.py`, `map_edit.py` |
| **Exploration** | `explore.py`, `explore_auto.py` |

What each diagnostic answers, and the order to climb them when something is wrong, is in
[developing.md → Diagnostic tools](../docs/developing.md#diagnostic-tools).

## Not in git

- `build/`, `install/`, `log/` — colcon output.
- `models/` — the face recognition `.onnx` models. Fetch them with
  `bash get_face_models.sh` ([why](../docs/faces.md#how-recognition-works)).
- `*.bak` — backups. If you find yourself making them, commit instead.
