# Bench Bring-up — JetRover + Jetson Orin Nano Super

From a box of hardware to a robot that drives, sees, hears and speaks — **validating
each part in isolation** before we trust the stack. See [`plan.md`](../plan.md) for
where the project stands now.

> **Bring-up is complete.** The robot was brought up in June 2026 and has run the full
> guide/tour demo since. This doc is the **record of how it was done** — kept because it
> is what you'd retrace if the Orin is ever reflashed or the robot rebuilt. The dated
> notes below are that history, not open tasks.

---

## 0. Before the robot — pre-staging (do now, no hardware)

- [ ] Install **Balena Etcher** (Windows). ✅ done
- [ ] Download the **"Jetson Orin Nano Developer Kit SD Card Image"** (JetPack 6.x)
      from NVIDIA's JetPack downloads. Keep it zipped — Etcher reads the `.zip`.
- [ ] *Plan B* download: **NVIDIA SDK Manager** installer (needed only if the
      QSPI bootloader turns out too old — see step 1).
- [ ] Confirm the USB-A→USB-C cable is a **data** cable, not charge-only.

---

## 1. Flash the Orin

We have **no local Ubuntu host**, so we use the Windows-friendly path: write the
JetPack image to the NVMe **externally** via the USB↔M.2 adapter, then boot it.

1. NVMe into the USB↔M.2 adapter → plug into the Windows PC.
2. **Balena Etcher** → select the JetPack `.zip` → select the NVMe → Flash.
3. Power off, move the NVMe into the Jetson's **M.2 Key-M** slot.
4. Power on (HDMI + keyboard attached) and complete first-boot setup.

> **What actually happened (2026-06-24):** the QSPI gate **was** hit — the
> Etcher-only NVMe wouldn't boot (stuck in UEFI). Resolved with the full 2-stage
> Hiwonder process: **(1)** SDK Manager (JetPack 6.2) to update the QSPI bootloader,
> run from an **Ubuntu 22.04 VM in VMware** (VirtualBox kept dropping the recovery
> USB) via `sdkmanager --cli` (the Electron GUI won't render in the VM); **(2)**
> re-write the **JetRover NVMe image** with Etcher (SDK Manager overwrites the NVMe
> with vanilla JetPack). Full gotchas in the `jetson-orin-flashing` memory.

- [x] Orin boots to the desktop from NVMe — **done 2026-06-24**, JetRover system
      (`/opt/ros/humble` + `~/ros2_ws` present; confirmed not vanilla JetPack).

## 2. ROS 2 Humble + workspace

The Orin runs ROS 2 **Humble**. Our Python is distro-agnostic.

> **Real-robot reality (2026-06-24):** ROS 2 Humble + Nav2 + SLAM **come
> pre-installed** with the JetRover image — the apt block below is NOT needed. It's
> auto-sourced (`echo $ROS_DISTRO` → `humble`; default shell is **zsh**, so the
> manual source, if ever needed, is `setup.zsh`). `~/ros2_ws/src` already holds the
> vendor stack: `bringup driver navigation slam peripherals app calibration ...`.
> The only remaining task here is to add **our** `robot_ds_behavior` package and
> build it.

```bash
# (skip on the real robot — ROS/Nav2/SLAM already present) reference only:
# sudo apt install ros-humble-desktop ros-humble-navigation2 \
#      ros-humble-nav2-bringup ros-humble-slam-toolbox

# add our code: clone the repo and copy robot_ds_behavior into the workspace
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y   # resolves package.xml deps
colcon build --symlink-install
source install/setup.bash
ros2 pkg executables robot_ds_behavior   # expect: guide brain talk explore
```

- [ ] `colcon build` succeeds.
- [ ] `ros2 run robot_ds_behavior guide` is found (entry points wired).

## 3. Validate each part ALONE

Bring the vendor (Hiwonder) stack up first, then poke each subsystem. Don't move
on until each checkbox is green — a flaky LiDAR or mis-framed odom wastes days at
the SLAM stage.

```bash
# vendor bring-up — starts chassis driver, LiDAR, etc. (needs full robot power)
ros2 launch bringup bringup.launch.py
# ^ first check `ros2 node list`; if the JetRover auto-starts on full power-on,
#   nodes may already be up (was EMPTY with chassis off on the 19V bench feed).

# manual driving (teleop_twist_keyboard + teleop_twist_joy are pre-installed):
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # publishes /cmd_vel
```

> **First-drive safety:** lift the robot (tracks off the ground) or use clear
> space, start slow, keep the main switch reachable as a kill. Don't drive while
> the battery is charging.

| Part | Quick test | Pass = |
|---|---|---|
| **Motors / drive** | `ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}}'` (1–2 s, then zero) | robot creeps forward, both tracks |
| **LiDAR (A1)** | `ros2 topic echo /scan --once` | one full ring of ranges, no all-`inf` |
| **Depth cam** | `ros2 topic hz /depth_cam/depth/points` | a steady stream (the robot caps depth to ~5 fps for USB headroom) |
| **IMU** | `ros2 topic echo /imu --once` | sane orientation/accel |
| **Mic (6-array)** | record a few seconds, play back | voice audible |
| **Speaker** | play a test wav / `say()` | audible |
| **7" LCD touch** | tap test | touch registers |
| **micro-ROS / frames** | `ros2 run tf2_tools view_frames` | `base_footprint` present; `/cmd_vel`, `/scan` are the names our code expects |

> Our code expects `/cmd_vel`, `/scan`, and `base_footprint`. If the JetRover
> publishes different names, remap in the launch file — **don't** edit the nodes.

## 4. Map floor 4 (SLAM) — Step 5 starts here

```bash
ros2 launch slam slam.launch.py     # our slam package; drive the floor once, slowly, a full loop
ros2 run slam map_save              # saves to map_01 in ~/ros2_ws/src/slam/maps/ (always overwrites)
```

- [ ] `/map` builds while driving; save it.

The full mapping / re-mapping / driving flow lives in
[movement-nav.md](./movement-nav.md).

## 5. Tag POIs + run our code

Set the **real-robot** config in `robot_ds_behavior/pois.py`:
**`MAP_YAW = 0.0`, `MAP_OFFSET = (0.0, 0.0)`** (identity — POIs are tagged straight
in the floor map, no world→map calibration), then fill the real coordinates of
each destination (kitchen, reception, meeting, desks, exit). Destinations are now saved
as **named waypoints** in the map — record them with the web app (**Map → Add a point**)
or `waypoint_tool.py`; see [developing.md → Add a place](./developing.md#add-a-place-the-robot-can-go-to).

```bash
ros2 run robot_ds_behavior guide kitchen     # lead to a named destination, then return to base (--solo to skip)
ros2 run robot_ds_behavior brain "i'm hungry"  # Claude picks the destination
```

- [ ] Robot navigates to a named destination on the real map.

## 6. Voice + narration → demo

Wire `voice_prototype` in front (mic → STT → `pick_poi` → `guide` → TTS via
`say()`), then a dry-run end-to-end. That's Steps 6–7 of the plan.

---

## Gotchas seen in sim (watch for them on hardware)

- **Aggressive speed breaks SLAM** — a bad loop closure jumps localization several
  metres and the robot reports "Arrived" while off-target. Keep velocities modest
  while mapping; tune up only after the map is solid.
- **Obstacle avoidance = MPPI critic weights**, not just "LiDAR sees it". In sim,
  CostCritic had to outweigh PathAlignCritic for the robot to deviate around
  transient obstacles. Re-tune to the real chassis footprint / carpet.
- **`consider_footprint: true` needs a footprint polygon** — with only
  `robot_radius` set it crashes the controller. Keep it `false` unless a polygon
  is defined.
