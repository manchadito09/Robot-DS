#!/bin/bash
# wifi_watchdog.sh - keep the robot on the office WiFi, no hands.
#
# Why this exists: Hiwonder's wifi.service wipes every saved NetworkManager profile at boot
# (rm /etc/NetworkManager/system-connections/*) and only THEN tries to connect, retrying just
# 4 times. If those 4 fail (slow WiFi at boot, AP not up yet), the robot ends with no profile
# and no WiFi -- no internet, no Tailscale, unreachable. Which is fatal when nobody is there.
#
# So every few minutes: is wlan0 on the office WiFi? If not, put it back. Also kill the robot's
# own hotspot if it ever comes up: it steals wlan0 and leaves the robot with no internet.
#
# Reads the SSID/password from wifi_conf.py so there is ONE source of truth.
# Installed as wifi-watchdog.timer (see /etc/systemd/system/). Runs as root.
set -uo pipefail

CONF=/home/ubuntu/wifi_manager/wifi_conf.py
LOG=/home/ubuntu/wifi_manager/watchdog.log
IFACE=wlan0

say() { echo "$(date '+%F %T') $*" >> "$LOG"; }

# keep the log small
[ -f "$LOG" ] && [ "$(stat -c%s "$LOG")" -gt 200000 ] && tail -c 50000 "$LOG" > "$LOG.tmp" && mv "$LOG.tmp" "$LOG"

SSID=$(grep -oP "^WIFI_STA_SSID\s*=\s*'\K[^']+" "$CONF" 2>/dev/null)
PSK=$(grep -oP "^WIFI_STA_PASSWORD\s*=\s*'\K[^']+" "$CONF" 2>/dev/null)
if [ -z "${SSID:-}" ]; then
    say "ERROR: no WIFI_STA_SSID in $CONF -- doing nothing (refusing to guess)"
    exit 1
fi

active=$(nmcli -t -f NAME,DEVICE con show --active 2>/dev/null | awk -F: -v i="$IFACE" '$2==i{print $1}')

# The robot's own hotspot (HW-*) hijacks wlan0 and leaves it with no internet. Never let it live.
if [[ "$active" == HW-* ]]; then
    say "hotspot $active is up on $IFACE -- taking it down"
    nmcli con down "$active" >/dev/null 2>&1
    nmcli con modify "$active" connection.autoconnect no >/dev/null 2>&1
    active=""
fi

if [ "$active" = "$SSID" ]; then
    exit 0                       # already where we want to be, say nothing
fi

say "$IFACE is on '${active:-nothing}', want '$SSID' -- reconnecting"

if nmcli -t -f NAME con show 2>/dev/null | grep -qx "$SSID"; then
    nmcli con up "$SSID" >/dev/null 2>&1              # profile exists: just bring it up
else
    say "profile '$SSID' is gone from NetworkManager -- recreating it"
    nmcli device wifi rescan >/dev/null 2>&1
    sleep 5
    nmcli device wifi connect "$SSID" password "$PSK" >/dev/null 2>&1
fi

sleep 5
active=$(nmcli -t -f NAME,DEVICE con show --active 2>/dev/null | awk -F: -v i="$IFACE" '$2==i{print $1}')
if [ "$active" = "$SSID" ]; then
    say "back on '$SSID'"
    systemctl is-active --quiet tailscaled || { say "tailscaled was down -- restarting"; systemctl restart tailscaled; }
else
    say "STILL NOT CONNECTED (on '${active:-nothing}') -- will try again next tick"
fi
