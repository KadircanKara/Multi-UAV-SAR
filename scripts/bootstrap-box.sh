#!/usr/bin/env bash
# Prepare a fresh Ubuntu box to run the production stack.
#
#   ssh sar-prod
#   curl -fsSL https://raw.githubusercontent.com/KadircanKara/Multi-UAV-SAR/main/scripts/bootstrap-box.sh | sudo bash
#
# or, from a checkout:  sudo ./scripts/bootstrap-box.sh
#
# Installs Docker, creates the data directory with the right ownership, and adds
# swap. Safe to re-run — every step checks before acting.
set -euo pipefail

APP_DIR="${SAR_APP_DIR:-/opt/sar}"
DATA_DIR="${SAR_DATA_DIR:-/opt/sar/Results}"
APP_UID="${SAR_APP_UID:-1000}"
SWAP_GB="${SAR_SWAP_GB:-2}"

if [[ $EUID -ne 0 ]]; then
    echo "error: run with sudo" >&2
    exit 1
fi

echo "==> updating packages"
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y -qq ca-certificates curl gnupg unattended-upgrades

echo "==> installing Docker"
if command -v docker >/dev/null 2>&1; then
    echo "    already installed: $(docker --version)"
else
    install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
        | gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    chmod a+r /etc/apt/keyrings/docker.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
        > /etc/apt/sources.list.d/docker.list
    apt-get update -qq
    apt-get install -y -qq docker-ce docker-ce-cli containerd.io \
        docker-buildx-plugin docker-compose-plugin
    echo "    installed: $(docker --version)"
fi

systemctl enable --now docker

# So the login user can run docker without sudo (takes effect next login).
for u in ubuntu admin; do
    if id "$u" >/dev/null 2>&1; then
        usermod -aG docker "$u"
        echo "    added $u to the docker group (log out and back in to use it)"
    fi
done

echo "==> creating $DATA_DIR"
mkdir -p "$DATA_DIR/.runs"
# The containers run as uid 1000 and write temp run dirs under Results/.
chown -R "$APP_UID:$APP_UID" "$APP_DIR"
echo "    owned by uid $APP_UID"

echo "==> configuring ${SWAP_GB}G swap"
if swapon --show | grep -q '/swapfile'; then
    echo "    already active"
else
    fallocate -l "${SWAP_GB}G" /swapfile
    chmod 600 /swapfile
    mkswap /swapfile >/dev/null
    swapon /swapfile
    grep -q '^/swapfile' /etc/fstab || echo '/swapfile none swap sw 0 0' >> /etc/fstab
    echo "    enabled and persisted"
fi
# Prefer reclaiming cache over swapping; swap here is a safety net for a
# memory spike, not something to run in day to day.
sysctl -qw vm.swappiness=10
grep -q '^vm.swappiness' /etc/sysctl.conf || echo 'vm.swappiness=10' >> /etc/sysctl.conf

echo "==> enabling automatic security updates"
dpkg-reconfigure -f noninteractive unattended-upgrades >/dev/null 2>&1 || true

echo
echo "ready:"
free -h | sed 's/^/    /'
echo
df -h / | sed 's/^/    /'
echo
echo "next:"
echo "  1. copy docker-compose.prod.yml, Caddyfile and .env into $APP_DIR"
echo "  2. load the data:   sudo RESULTS_URL='<presigned url>' ./scripts/fetch-data.sh"
echo "  3. start:           cd $APP_DIR && docker compose -f docker-compose.prod.yml up -d"
