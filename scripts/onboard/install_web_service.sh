#!/usr/bin/env bash
# Install the web dashboard as a systemd *user* service (starts robot bringup + safety_stop + web).
#   bash scripts/onboard/install_web_service.sh          # install + enable at boot + start now
#   systemctl --user stop vln-web                         # stop (frees camera/LiDAR for other launches)
#   systemctl --user disable vln-web                      # no autostart
#   journalctl --user -u vln-web -f                       # logs
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p ~/.config/systemd/user
cp "$HERE/vln-web.service" ~/.config/systemd/user/vln-web.service
systemctl --user daemon-reload
systemctl --user enable --now vln-web
# start at boot even before anyone logs in (needs polkit/sudo on some systems)
loginctl enable-linger "$USER" 2>/dev/null || echo "note: 'sudo loginctl enable-linger $USER' to start before login"
sleep 2
systemctl --user --no-pager status vln-web | head -5
echo "dashboard: http://$(hostname -I | awk '{print $1}'):8080/"
