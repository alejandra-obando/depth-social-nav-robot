#!/usr/bin/env bash
# From the PC: runs the tests (nothing is copied if they fail), copies software/ to
# ~/social_nav on the Pi (only what changed; what is deleted here is deleted there),
# builds the four packages and, if social-nav is running, restarts it.
# Usage: software/scripts/deploy.sh [--no-tests]
#   WALLENS_IP=<ip> as in the second term; otherwise the wallens repo's scripts/pi-ip.sh
#   (WALLENS_REPO=<path>, default ../wallens-fruit-picker-robot) or wallens.local.
set -euo pipefail
HERE=$(cd "$(dirname "$0")/.." && pwd)
REPO=$(cd "$HERE/.." && pwd)
WALLENS_REPO=${WALLENS_REPO:-$REPO/../wallens-fruit-picker-robot}
if [ -n "${WALLENS_IP:-}" ]; then IP=$WALLENS_IP
elif [ -x "$WALLENS_REPO/scripts/pi-ip.sh" ]; then IP=$("$WALLENS_REPO/scripts/pi-ip.sh")
else IP=wallens.local; fi
PI=wallens@$IP

if [ "${1:-}" != "--no-tests" ]; then
  echo "== tests"
  PY=python3; [ -x "$REPO/.venv/bin/python" ] && PY="$REPO/.venv/bin/python"
  (cd "$HERE/src" && "$PY" -m pytest -q 2>&1 | tail -1)
fi

echo "== rsync to $PI:social_nav"
rsync -rlpc --delete --itemize-changes --exclude __pycache__ --exclude '*.pyc' --exclude .pytest_cache \
  "$HERE/" "$PI:social_nav/" | sed 's/^/  /'

echo "== colcon build"
ssh -n "$PI" "~/social_nav/scripts/install-pi.sh > /tmp/social_nav_build.log 2>&1; c=\$?; tail -3 /tmp/social_nav_build.log; exit \$c" \
  || { echo "build failed: nothing restarted (ssh $PI cat /tmp/social_nav_build.log)"; exit 1; }

if ssh -n "$PI" "systemctl is-active -q social-nav"; then
  echo "== restarting social-nav"
  ssh -n "$PI" "sudo -n systemctl restart social-nav" || echo "  could not: run it by hand on the Pi"
fi
grep -q . <<< "$(cd "$HERE" && git status --porcelain systemd config 2>/dev/null)" \
  && echo "== systemd/ or config/ changed: on the Pi run sudo ~/social_nav/scripts/install-pi.sh deps"
exit 0
