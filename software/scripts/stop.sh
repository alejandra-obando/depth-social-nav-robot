#!/usr/bin/env bash
# From the PC: stops the run (the node publishes a zero cmd_vel and exits; the base robot keeps running).
set -euo pipefail
PI=wallens@${WALLENS_IP:-wallens.local}
ssh "$PI" "sudo -n systemctl stop social-nav && echo 'social-nav stopped'"
