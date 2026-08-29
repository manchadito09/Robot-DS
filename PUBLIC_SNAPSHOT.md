# About this public snapshot

This repository is a **public snapshot** of a working robot, published so the code
can be read. It is not the live development repo, and a few things were left out
on purpose. If a document below refers to a file you cannot find, this is why.

## What was left out, and why

| Left out | Why |
|---|---|
| `ros2_ws/src/slam/maps/` — the floor map (`map_01.pgm`, `map_01.yaml`, `map_01.waypoints.yaml`) | It is a LiDAR scan of a real office floor, with the exact position of its stairs, lifts and rooms. That is the building's business, not the internet's. Every tool that reads it is still here; make your own with SLAM — see [docs/movement-nav.md](./docs/movement-nav.md). |
| The vendor's ROS 2 packages — `bringup`, `navigation`, `slam`, `driver`, `peripherals` | They ship with the [Hiwonder JetRover](https://docs.hiwonder.com/projects/JetRover/en/jetson-orin-nano/) and declare no license, so they are not ours to republish. Get them from the vendor image. The **Nav2 YAML we tuned ourselves** is kept, under `ros2_ws/src/navigation/config/` and `ros2_ws/src/slam/config/`. |
| The remote-access details (machine name, address, ports) | Removed from [system/README.md](./system/README.md). Nobody needs our robot's front door. |
| The git history | This is a single commit. The history of the private repo carries slide decks, internal documents and an old tarball that were never audited for a public release, and deleting a file does not remove it from a git history. |

## What was never in the repo to begin with

No API keys. No WiFi password (it lives in the vendor's `wifi_conf.py`, on the robot).
No face data — the recognition store and the models are gitignored and stay on the
robot. See [docs/faces.md](./docs/faces.md#privacy).

## What is all here

Everything we wrote: the behaviour package (`robot_ds_behavior/`), the web app
(`robot_web/`), the voice front-end (`voice_prototype/`), the standalone tools in
`ros2_ws/`, the arm gestures (`arm/`), the systemd units (`system/`), and the docs.
