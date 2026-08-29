# robot_web/ — the phone/laptop web app

The robot's remote control: a page you open on a phone or a laptop on the same network.
Map (tap to go), Tour, Arm gestures, **Talk to me**, **Add me** (greeting), and a big red
**STOP**. It is also where two watchdogs live — see [more than a web page](#more-than-a-web-page).

> ### 🔴 It runs from the robot, not from this repo
> `robot-web.service` starts **`/home/ubuntu/Robot-DS/robot_web/server.py`**. Editing the
> copy in this repo changes nothing on the robot — copy it across. Same trap as
> [the three copies of the source](../docs/developing.md#-read-this-first-three-copies-of-the-source).

## The files

| File | What it is |
|---|---|
| `server.py` | The whole backend: serves the page, the `/api/…` endpoints (go, drive, tour, arm, waypoints, faces, status), and the watchdogs below. |
| `index.html` | The entire front-end — one page, plain HTML/JS. No framework, no build step. |
| `start_web.sh` | Starts/restarts the always-on web service. Wired to the **Start Robot Server** desktop icon. |
| `show_qr.sh` | The **start-of-session button**: restarts the web app, puts the arm in its scan pose, starts the camera, then shows a QR card. Wired to **Show QR (phone app)** / **Robot Web QR**. |
| `show_qr.py` | Builds the "scan me" card (QR + URL). |
| `open_web.sh` | Waits for the service to answer, then opens the page in a browser. |
| `show_qr.desktop`, `icon*.png` | Copies of the desktop launcher and its icons, kept so a reflash doesn't lose them. |

**Why a QR at all:** the Jetson has no working browser, so the robot can't show its own
page. It shows a QR instead and your phone does the browsing.

## Starting it

You don't start it by hand — `robot-web.service` keeps it running and restarts it if it
dies (the unit is backed up in [`system/`](../system/README.md)).

```bash
sudo systemctl restart robot-web     # after editing server.py
```

`index.html` needs no restart — just reload the page in the browser.

## More than a web page

Two things live in `server.py` that you would not expect from a UI:

- **The camera watchdog.** It subscribes to `/camera_scan` — the *end* of the vision
  chain, so one topic proves the whole path. If it goes quiet it repairs the camera, and
  if this happens mid-trip it also stops the robot and says so, rather than letting it
  drive blind to low obstacles. This is the `GREEN` chain — see
  [the vision chain](../docs/developing.md#the-vision-chain).
- **The face-greeter watchdog** (`_face_idle_watch`). If `face_greet.py` dies it brings it
  back in ~2 s, and it hands the camera over whenever the page's preview needs it — see
  [faces.md](../docs/faces.md).

To drive deliberately **without** a camera, the watchdog can be switched off with an
environment override — the exact commands are in
[developing.md](../docs/developing.md#the-vision-chain).

## Changing it

Full recipes (and the traps) are in [developing.md](../docs/developing.md). The short
version:

| You changed… | To see it |
|---|---|
| `index.html` | Reload the page |
| `server.py` | `sudo systemctl restart robot-web` |
| Anything | Copy it into the repo and commit — nothing syncs itself |
