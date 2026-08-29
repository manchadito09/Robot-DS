# Robot system config (outside ROS)

Things that live in `/etc` and `/home/ubuntu/wifi_manager` on the Jetson, kept here so a reflash or a
dead SD card doesn't take them with it. Copies, not the live files — the robot reads the originals.

## What's kept here

Every service the robot needs that is **ours** (the vendor's own units are not our problem):

| File | What it starts | If it's gone |
|---|---|---|
| `robot-web.service` | The phone/laptop web app — `Robot-DS/robot_web/server.py` | No web app: no map, no Talk to me, no **STOP** button |
| `claude-daemon.service` | The warm Claude daemon — `ros2_ws/claude_daemon.py` | Still works, but every answer takes ~10 s instead of ~2 s |
| `stt-daemon.service` | The warm Whisper daemon — `voice_prototype/stt_daemon.py` | The robot stops understanding speech ([details](../voice_prototype/README.md)) |
| `wifi-watchdog.service` + `.timer` + `wifi_watchdog.sh` | Keeps the robot on the office WiFi — the story below | The robot can strand itself off the network with nobody there |

**To restore one after a reflash:**

```bash
sudo cp <name>.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now <name>
```

Keep these in sync by hand — the robot reads `/etc/systemd/system/`, not this folder. If you
change a unit on the robot, copy it back here.

## Why any of this exists: the robot used to sabotage its own WiFi

Hiwonder ships `wifi.service` → `/home/ubuntu/wifi_manager/wifi.py`, and with
`wifi_conf.py: WIFI_MODE = 1` (AP mode) it did this on **every boot**:

```
nmcli connection down/delete <the active WiFi>     # dropped the office WiFi
rm /etc/NetworkManager/system-connections/*        # deleted EVERY saved profile
nmcli con add ... HW-<serial> autoconnect yes      # recreated its own hotspot
nmcli con modify ... 802-11-wireless.mode ap
nmcli con up HW-<serial>                           # and brought it up on wlan0
```

One radio, two jobs. When the hotspot won, the robot had no internet, no Tailscale, and no way in
from outside the office. Fixing it with `nmcli con modify` never stuck, because the profile was not
being *modified* — it was being **deleted and recreated** at the next boot.

## The fix

1. **`wifi_conf.py`: `WIFI_MODE = 2`** (client) with `WIFI_STA_SSID` / `WIFI_STA_PASSWORD` set to the
   office WiFi. Same vendor mechanism, opposite outcome: it now connects to the office on purpose and
   never raises a hotspot. The real file holds the WiFi password, so it is **not** in this repo.
   The `busybox devmem` poke and the GPIO/LED code in `wifi.py` were left alone — nobody knows what
   they do, and guessing could kill the WiFi hardware.

2. **`wifi_watchdog.sh` + `wifi-watchdog.{service,timer}`** — every 5 minutes: if `wlan0` is not on
   the office WiFi, put it back; if the profile is gone, recreate it from `wifi_conf.py`; if the
   hotspot ever comes up, take it down; if `tailscaled` died, restart it. Silent when all is well.

   It exists because `wifi.py` wipes the profiles and *then* connects, retrying only 4 times. If those
   4 fail (slow WiFi at boot), the robot ends with no profile and no WiFi — fatal when nobody is there.

   Tested for real, not in theory: WiFi dropped → back in 14 s. Profile **deleted from disk** (exactly
   what `wifi.py` does) → recreated and online in 11 s.

## Also set on the robot (no file to keep here)

- **Autologin**: `/etc/gdm3/custom.conf` → `AutomaticLoginEnable=true`, `AutomaticLogin=ubuntu`.
  Without it, a reboot leaves the robot on the login screen: no desktop, no icons, no RViz.
- **No screen lock / no blanking / no suspend** (`gsettings`: `screensaver lock-enabled false`,
  `idle-activation-enabled false`, `session idle-delay 0`, `power sleep-inactive-ac-type nothing`).
  The desktop was locking itself after a while and asking for a password.
- **Remote access**: the robot joins a private [Tailscale](https://tailscale.com) network at boot,
  which is how we reach it from outside the office — SSH, the web app, and a full remote desktop.
  The machine name, address and ports are deliberately left out of this public snapshot.

Together these mean a power cut no longer needs hands: the robot boots, joins the office WiFi, comes
back on Tailscale, and lands on the desktop by itself.
