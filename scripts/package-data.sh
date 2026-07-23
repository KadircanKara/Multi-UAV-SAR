#!/usr/bin/env bash
# Package the seeded mission data for upload.
#
#   ./scripts/package-data.sh [output.tar.gz]
#
# Results/ is ~1.7 GB on disk but compresses to ~70 MB: the solution pickles are
# highly repetitive, so gzip manages roughly 24x. That is small enough to keep
# in object storage and pull on a fresh box in seconds.
#
# Only the three directories the backend actually reads are included. Animations/,
# Paths/, Res/ and Runtimes/ are legacy and never opened at runtime, and .runs/
# is scratch that the app recreates.
set -euo pipefail

cd "$(dirname "$0")/.."

OUT="${1:-Results.tar.gz}"
SRC="${SAR_RESULTS_ROOT:-Results}"

if [[ ! -d "$SRC" ]]; then
    echo "error: no data at '$SRC'" >&2
    echo "       run this from a checkout that has the seeded Results/ tree," >&2
    echo "       or point SAR_RESULTS_ROOT at one." >&2
    exit 1
fi

missing=()
for d in Objectives Solutions Metadata; do
    [[ -d "$SRC/$d" ]] || missing+=("$d")
done
if (( ${#missing[@]} )); then
    echo "error: '$SRC' is missing: ${missing[*]}" >&2
    exit 1
fi

n_obj=$(find "$SRC/Objectives" -name '*.pkl' | wc -l | tr -d ' ')
n_sol=$(find "$SRC/Solutions"  -name '*.pkl' | wc -l | tr -d ' ')
echo "packaging $n_obj objective sets, $n_sol solution sets from '$SRC'..."

if [[ "$n_obj" == "0" || "$n_sol" == "0" ]]; then
    echo "error: refusing to build an empty archive" >&2
    exit 1
fi

extra=()
[[ -f "$SRC/custom_models.json" ]] && extra+=("Results/custom_models.json")

# Stage a symlink named Results so the archive always unpacks as Results/,
# whatever the source directory is called. GNU tar's --transform would do this
# in one flag, but macOS ships bsdtar, which has no such option — and this
# script runs on a laptop. -h makes tar follow the symlink.
stage=$(mktemp -d)
trap 'rm -rf "$stage"' EXIT
ln -s "$(cd "$SRC" && pwd)" "$stage/Results"

tar -czhf "$OUT" -C "$stage" \
    Results/Objectives Results/Solutions Results/Metadata "${extra[@]}"

size=$(du -h "$OUT" | cut -f1)
echo "wrote $OUT ($size)"
echo
echo "next:"
echo "  aws s3 cp $OUT s3://YOUR_BUCKET/$(basename "$OUT")"
