#!/usr/bin/env bash
# From the PC: starts a run.  Place the robot at the start corner facing the opposite
# corner first: the start pose is the robot's pose when the controller starts.
#   software/scripts/start.sh            WALLENS_IP=<ip> if wallens.local does not resolve
# Then: the robot waits until it sees the back tag, says "Sígueme" and follows.
set -euo pipefail
PI=wallens@${WALLENS_IP:-wallens.local}
ssh "$PI" "sudo -n systemctl restart social-nav && echo 'social-nav started: the start pose is stored now'"
echo "Log: ssh $PI journalctl -u social-nav -f      Stop: software/scripts/stop.sh"
