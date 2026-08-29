#!/usr/bin/env bash
# Start (or restart) the always-on web server service.
# Wired to the "Start Robot Server" desktop icon.
sudo -n systemctl restart robot-web
if which notify-send >/dev/null 2>&1; then
    notify-send "Robot Web" "Server (re)started — http://localhost:8000"
fi
exit 0
