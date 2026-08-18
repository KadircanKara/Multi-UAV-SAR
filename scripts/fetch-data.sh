#!/usr/bin/env bash
# Download and unpack the mission data on the server.
#
#   # on your laptop — the URL is valid for one hour
#   URL=$(aws s3 presign s3://YOUR_BUCKET/Results.tar.gz --expires-in 3600)
#
#   # on the box
#   sudo RESULTS_URL="$URL" ./scripts/fetch-data.sh
#
# A presigned URL carries its own signature, so the server needs no AWS
# credentials at all — nothing long-lived to steal, and the link expires.
set -euo pipefail

DEST="${SAR_DATA_DIR:-/opt/sar/Results}"
APP_UID="${SAR_APP_UID:-1000}"

if [[ -z "${RESULTS_URL:-}" ]]; then
    echo "error: RESULTS_URL is not set" >&2
    echo "       generate one with:" >&2
    echo "         aws s3 presign s3://YOUR_BUCKET/Results.tar.gz --expires-in 3600" >&2
    exit 1
fi

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

echo "downloading..."
# --fail so an expired URL surfaces as an error instead of saving S3's XML
# error document and failing confusingly at the untar step.
curl --fail --show-error --silent --location -o "$tmp/data.tar.gz" "$RESULTS_URL"

size=$(du -h "$tmp/data.tar.gz" | cut -f1)
echo "got $size, verifying..."
tar -tzf "$tmp/data.tar.gz" >/dev/null

echo "unpacking to $DEST ..."
mkdir -p "$(dirname "$DEST")"
tar -xzf "$tmp/data.tar.gz" -C "$tmp"

if [[ ! -d "$tmp/Results" ]]; then
    echo "error: archive did not contain a Results/ directory" >&2
    exit 1
fi

# Replace atomically-ish: keep the old tree until the new one is in place.
if [[ -d "$DEST" ]]; then
    echo "  existing data found, moving aside"
    rm -rf "${DEST}.old"
    mv "$DEST" "${DEST}.old"
fi
mv "$tmp/Results" "$DEST"

# The container runs as uid 1000 and writes temp run dirs under Results/.runs.
# Without this the app starts and then fails on the first optimization.
mkdir -p "$DEST/.runs"
chown -R "$APP_UID:$APP_UID" "$DEST"

echo
echo "done:"
# Count scenarios, not files: Objectives/ also holds one -AllObjectives.pkl
# sibling per scenario, so a bare *.pkl count reports double.
echo "  scenarios  : $(find "$DEST/Objectives" -name '*-ObjectiveValues.pkl' | wc -l | tr -d ' ')"
echo "  solutions  : $(find "$DEST/Solutions"  -name '*.pkl' | wc -l | tr -d ' ')"
echo "  total size : $(du -sh "$DEST" | cut -f1)"
echo "  owned by   : uid $APP_UID"
[[ -d "${DEST}.old" ]] && echo "  previous copy kept at ${DEST}.old — remove it once the app is verified"
exit 0
