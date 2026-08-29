#!/usr/bin/env bash
# Ensure the always-on web service is up, then open the control page in a browser.
# Wired to the "Robot Web" desktop icon.
systemctl is-active --quiet robot-web || sudo -n systemctl start robot-web
# wait until it answers (up to ~10s) so the browser doesn't hit a dead page
for _ in $(seq 1 20); do
    curl -s -o /dev/null --max-time 1 http://localhost:8000/ && break
    sleep 0.5
done
exec chromium-browser --app=http://localhost:8000
