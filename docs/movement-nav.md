# Movement & Navigation — how the robot maps and drives

This is the movement half of the robot: build a map (SLAM), save it, then drive
autonomously to named places with Nav2 — plus an autonomous "map the room by
itself" mode. It pairs with the voice layer in [`voice-guide.md`](./voice-guide.md).

> These files live in the live workspace `~/ros2_ws/src/` on the robot. This repo
> keeps a copy under `ros2_ws/src/…` so the work is versioned; to run them, they
> must be in the live `~/ros2_ws` (the launch files hardcode that path).

---

## 1. Modes (they fight over the hardware — don't run two at once)

| Mode | What runs | Gamepad |
|---|---|---|
| Normal app | `start_app_node.service` | ON |
| SLAM (drive to map) | slam + lidar; you drive with the pad | ON (`use_joy=true`) |
| Navigation | Nav2 + AMCL on a saved map | OFF (`use_joy=false`) |
| Auto-mapping | base + SLAM + Nav2 + explorer | OFF |

**Always** `sudo systemctl stop start_app_node.service` before Nav2 / auto-mapping —
the gamepad service grabs the LiDAR and its `joystick_control` node floods
`/controller/cmd_vel` with zeros, overriding Nav2 (this was the "robot won't move"
bug). `use_joy` default is `true` in `slam/launch/include/robot.launch.py`, explicit
`false` in `navigation.launch.py` and `auto_map_full.launch.py`.

---

## 2. Files (all under `ros2_ws/src/`)

| File | What it does |
|---|---|
| `slam/launch/include/robot.launch.py` | Base bring-up include (drivers, `use_joy` default). |
| `slam/launch/auto_map_full.launch.py` | Autonomous mapping: base + SLAM + Nav2 (no AMCL) + explorer. |
| `slam/launch/auto_mapping.launch.py` | Variant of the auto-map launch. |
| `bringup/scripts/auto_map.sh` | One-shot launcher for auto-mapping (stops the gamepad, runs the stack in the foreground). |
| `bringup/scripts/slam_explore.sh` | Helper to drive exploration during SLAM. |
| `navigation/launch/navigation.launch.py` | Normal navigation (Nav2 + AMCL) on `map_01`, `use_joy=false`. |
| `navigation/launch/nav_for_slam.launch.py` | Nav2 wired for the mapping mode (no AMCL). |
| `navigation/launch/include/navigation_base.launch.py`, `bringup.launch.py` | Nav2 base includes. |
| `navigation/config/nav2_params.yaml` | Nav2 tuning for normal navigation. |
| `navigation/config/nav2_controller_teb.yaml` | TEB local planner tuning (15 Hz, homotopy off, etc.). |
| `navigation/config/nav2_params_mapping.yaml` | Separate, slow/safe Nav2 tuning for auto-mapping (0.18 m/s, big obstacle margins). |
| `navigation/config/nav2_controller_teb_mapping.yaml` | TEB tuning for the mapping mode. |
| `robot_ds_behavior/robot_ds_behavior/explore.py` | Frontier explorer for autonomous mapping (sends Nav2 goals into unknown space). |
| `waypoint_tool.py` | Save/list/go/del **named** waypoints (Nav2 has no built-in). See below. |
| `slam/maps/` | The floor-4 map itself: `map_01.pgm` + `map_01.yaml` + `map_01.waypoints.yaml`. |

---

## 3. Build a map (SLAM) and save it

```bash
sudo systemctl start start_app_node.service          # gamepad ON to drive while mapping
ros2 launch slam slam.launch.py                      # start SLAM, then drive around with the pad
ros2 run slam map_save                               # saves to map_01 (or: bash src/bringup/scripts/save_map.sh)
```
SLAM only **creates** maps; **Navigation opens** them. `map_save` always overwrites
`map_01`. The map is `map_01.pgm` (occupancy) + `map_01.yaml` (metadata).

## 4. Navigate to named places (waypoints)

```bash
sudo systemctl stop start_app_node.service           # gamepad OFF
ros2 launch navigation navigation.launch.py          # Nav2 + AMCL on map_01
# set "2D Pose Estimate" in RViz, then:
python3 ~/ros2_ws/waypoint_tool.py save kitchen      # save robot's current spot as "kitchen"
python3 ~/ros2_ws/waypoint_tool.py pick kitchen      # ...or click it in RViz ("Publish Point")
python3 ~/ros2_ws/waypoint_tool.py list
python3 ~/ros2_ws/waypoint_tool.py go kitchen        # publishes /goal_pose -> Nav2 drives there
```
Waypoints are saved to `slam/maps/map_01.waypoints.yaml` (`{x, y, yaw}` in the Nav2
`map` frame). **This same file is what the voice brain uses as its destinations.**

Navigation tuning that mattered: TEB at 15 Hz, homotopy planning off, three velocity
limits raised to actually move (in `nav2_params.yaml` + `nav2_controller_teb.yaml`).

## 5. Map the room autonomously

```bash
bash ~/ros2_ws/src/bringup/scripts/auto_map.sh       # foreground, from a normal terminal
```
Starts base + `slam_toolbox` + Nav2 (no AMCL) + the frontier explorer (`explore.py`).
Nav2 drives; the explorer feeds goals into unknown space. Uses its **own** slow/safe
tuning (`nav2_params_mapping.yaml` + `nav2_controller_teb_mapping.yaml`, 0.18 m/s, big
margins) so it doesn't ram things — the normal navigation tuning is left untouched.

---

## 6. Gotchas

- **`need_compile=False`.** `ros2 launch <pkg> <file>` reads from `install/…/launch/`,
  NOT `src/`. After editing a **top-level** launch file, copy it into `install/`.
  Includes with a hardcoded `src` path (robot.launch.py, navigation_base, nav2_params)
  do pick up `src/` edits directly. Python packages in symlink-install (controller,
  robot_ds_behavior) just need the node restarted.
- **Low obstacles (chair legs) live below the LiDAR — the depth camera catches them.**
  `camera_scan_node` turns the depth cloud into a floor-level `/camera_scan`, and Nav2's
  `camera_layer` (in `nav2_params.yaml`, on both the local *and* global costmaps) folds it
  into the costmap. This is the **GREEN chain**: when it is up the robot avoids chair legs;
  when it is down the robot really is blind to low things, so **no `GREEN`, no go**. Full
  detail in [developing.md — the vision chain](./developing.md#the-vision-chain).
- **LiDAR** = Slamtec A1 (`LIDAR_TYPE=A1`), `/dev/lidar`. Frames: `map` / `odom` /
  `base_footprint`; scan frame `lidar_frame`.
- **DDS** — run from a terminal that sourced the normal robot env (`~/ros2_ws/.robotrc`);
  a bare `/opt/ros/humble`-only shell sometimes can't see the graph.

---

## 7. Re-map a bad area (without losing the rest, or the waypoints)

When part of the floor came out wrong (bent walls, AMCL slips), **continue** the existing
map instead of rebuilding from scratch. SLAM Toolbox deserialises the saved graph
(`map_01.data` + `map_01.posegraph`) and keeps mapping on top, so it keeps the **same
frame/origin** → **the waypoints survive**.

### What survives and what doesn't

| | Survives the re-map? |
|---|---|
| Waypoints (kitchen, dev, chess…) | ✅ Yes (same frame) |
| The rest of the already-mapped floor | ✅ Yes |
| Glass walls hand-painted into the `.pgm` | ❌ No — the `.pgm` is regenerated. See the trick. |

**Key trick:** the `.pgm` painting is NOT in the SLAM data, only in the image. On save,
SLAM writes a fresh `.pgm` and wipes it. To avoid re-painting → **physically cover the
glass while mapping** (cardboard/chairs): the lidar sees it solid and maps it as a factory
wall.

### Steps

**0. Before**
- Clear the area.
- Cover the glass in that area with something opaque.
- Backup: `~/ros2_ws/src/slam/maps/backup_map.sh`

**1. Launch SLAM (continuing the map).** Place the robot on a known spot (e.g. on a
waypoint whose coordinates you know, in `map_01.waypoints.yaml`).

```bash
# Terminal 1
source ~/ros2_ws/.typerc
ros2 launch slam slam.launch.py
# Terminal 2 (watch the map live)
ros2 launch slam rviz_slam.launch.py
# Terminal 3 (load the old map to continue on top of it)
ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph \
"{filename: '/home/ubuntu/ros2_ws/src/slam/maps/map_01', match_type: 2, initial_pose: {x: <X>, y: <Y>, theta: <TH>}}"
```

`match_type: 2` = start at the given pose. If it complains about fields:
`ros2 interface show slam_toolbox/srv/DeserializePoseGraph`.

**2. ⚠️ Verify BEFORE moving.** In RViz, the lidar scan must **overlap** the walls of the
old map. If it looks bent/shifted → STOP and redo the deserialize with a better pose. Do
not drive until it lines up (or you build a ghost map on top of the real one).

**3. Drive only the bad area.**
- **In arcs, NEVER pivot** in place (breaks scan-matching → bent walls).
- Slowly, slow passes to re-observe well.
- Drive back out into a corridor that was already good → this triggers the loop closure
  that straightens things out.

**4. Save (same frame → waypoints intact).**

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
"{filename: '/home/ubuntu/ros2_ws/src/slam/maps/map_01'}"
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'map_01'}}"
```

**5. After.**
- Launch Navigation and check AMCL no longer slips in those rooms.
- If a **pure keepout** (not glass) was lost, re-apply it with `~/ros2_ws/map_edit.py`.

### If it comes out worse

Restore from the `backup_*` folder that `backup_map.sh` made (copy the 5 files back into
`~/ros2_ws/src/slam/maps/`) and try again.
