#!/bin/bash
# EC2 bootstrap — paste this as User Data when launching the spot instance.
# Runs the batch optimizer, syncs to S3, then self-terminates.
#
# BEFORE LAUNCHING: replace the two variables below.

S3_BUCKET="multi-uav-sar-results"   # ← your bucket name
S3_REGION="us-east-1"               # ← your region

REPO_URL="https://github.com/KadircanKara/Multi-UAV-SAR.git"
REPO_DIR="/home/ec2-user/Multi-UAV-SAR"
LOG="/var/log/sar-batch.log"
SEEDS=5
NGEN=1000

exec > >(tee -a "$LOG") 2>&1
set -euo pipefail
echo "=== SAR batch bootstrap $(date) ==="

# ── system deps ───────────────────────────────────────────────────────────────
dnf install -y git python3.11 python3.11-pip python3.11-devel gcc gcc-c++ &>/dev/null
echo "system packages ready"

# ── clone repo ────────────────────────────────────────────────────────────────
if [ -d "$REPO_DIR/.git" ]; then
    git -C "$REPO_DIR" pull --ff-only
else
    git clone "$REPO_URL" "$REPO_DIR"
fi
cd "$REPO_DIR"

# ── python venv ───────────────────────────────────────────────────────────────
python3.11 -m venv .venv
.venv/bin/pip install --quiet --upgrade pip
.venv/bin/pip install --quiet -r requirements.txt
echo "venv ready"

# ── count CPUs for worker sizing ──────────────────────────────────────────────
NCPU=$(nproc)
WORKERS=$(( NCPU > 2 ? NCPU - 2 : 1 ))
echo "CPUs=$NCPU  workers=$WORKERS"

# ── dry-run first to confirm cell list ────────────────────────────────────────
.venv/bin/python backend/scripts/ec2_batch_run.py \
    --s3-bucket "$S3_BUCKET" \
    --s3-region "$S3_REGION" \
    --seeds "$SEEDS" \
    --ngen "$NGEN" \
    --workers "$WORKERS" \
    --dry-run

# ── full batch run ────────────────────────────────────────────────────────────
echo "=== starting batch run $(date) ==="
.venv/bin/python backend/scripts/ec2_batch_run.py \
    --s3-bucket "$S3_BUCKET" \
    --s3-region "$S3_REGION" \
    --seeds "$SEEDS" \
    --ngen "$NGEN" \
    --workers "$WORKERS"

# terminate is called from within ec2_batch_run.py; this line is fallback
echo "=== batch complete $(date) ==="
