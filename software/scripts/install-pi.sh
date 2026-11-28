#!/usr/bin/env bash
# Installs the social navigation packages on the Pi of the second term, on top of the
# wallens base robot (scripts 00-05 of the wallens repository already run).  On the Pi,
# from ~/social_nav (copied there by scripts/deploy.sh):
#   sudo scripts/install-pi.sh deps     apt + pip dependencies, udev rule, systemd unit (once)
#   scripts/install-pi.sh               links the packages into ~/ros_ws/src and builds them
set -euo pipefail
HERE=$(cd "$(dirname "$0")/.." && pwd)
PACKAGES="perception corners navigation bringup"

if [ "${1:-}" = deps ]; then
  [ "$(id -u)" = 0 ] || { echo "Run with sudo" >&2; exit 1; }
  apt-get update
  apt-get install -y ros-jazzy-realsense2-camera ros-jazzy-v4l2-camera alsa-utils espeak-ng python3-pip python3-numpy
  # AprilTag 3 (has aarch64 wheels).  Ubuntu 24.04 protects the system Python: --break-system-packages
  # only adds this package, nothing is replaced.
  sudo -u "${SUDO_USER:-wallens}" pip install --user --break-system-packages pupil-apriltags
  install -m 644 "$HERE/config/99-social-nav.rules" /etc/udev/rules.d/99-social-nav.rules
  udevadm control --reload-rules && udevadm trigger
  install -m 644 "$HERE/systemd/social-nav.service" /etc/systemd/system/social-nav.service
  visudo -cf "$HERE/systemd/sudoers-social-nav"      # a broken rule would break sudo: checked first
  install -m 440 "$HERE/systemd/sudoers-social-nav" /etc/sudoers.d/social-nav
  systemctl daemon-reload
  usermod -aG audio "${SUDO_USER:-wallens}"
  echo "OK: dependencies installed.  Check: ls -l /dev/cam_rear; aplay -L"
  exit 0
fi

[ "$(id -u)" != 0 ] || { echo "The build runs without sudo (as the robot user)" >&2; exit 1; }
WS=${1:-$HOME/ros_ws}
[ -d "$WS/src" ] || { echo "$WS/src not found: install the wallens workspace first (scripts/05-ros-ws.sh)" >&2; exit 1; }
for p in $PACKAGES; do
  ln -sfn "$HERE/src/$p" "$WS/src/$p"
done
set +u; source /opt/ros/jazzy/setup.bash; set -u
cd "$WS"
nice -n 10 colcon build --packages-select $PACKAGES
echo "OK: $PACKAGES built in $WS"
# Spoken phrases (espeak-ng), only if there are none yet: recordings of your own are not overwritten.
VOICE=$HOME/.config/social_nav/voice
if [ ! -d "$VOICE" ]; then
  set +u; source "$WS/install/setup.bash"; set -u
  ros2 run bringup make_voice "$VOICE"
fi
