#!/usr/bin/env bash
# Download the MovingAI MAPF benchmark maps and random scenarios used by swarmpath.
# Usage: bash data/fetch.sh   (safe to re-run: existing files are kept)
set -euo pipefail

cd "$(dirname "$0")"
BASE=https://movingai.com/benchmarks/mapf
MAPS=(empty-8-8 random-32-32-10 room-32-32-4 warehouse-10-20-10-2-1 den312d)

mkdir -p maps scen
for zip in mapf-map.zip mapf-scen-random.zip; do
    [ -f "$zip" ] || { echo "downloading $zip"; curl -fsSL --retry 3 -o "$zip" "$BASE/$zip"; }
done

for name in "${MAPS[@]}"; do
    unzip -n -j -q mapf-map.zip "*/$name.map" "$name.map" -d maps 2>/dev/null || true
    unzip -n -j -q mapf-scen-random.zip "*/$name-random-*.scen" -d scen 2>/dev/null || true
    [ -f "maps/$name.map" ] || { echo "missing maps/$name.map" >&2; exit 1; }
    ls scen/"$name"-random-*.scen >/dev/null || { echo "missing scenarios for $name" >&2; exit 1; }
done
echo "maps: $(ls maps | wc -l), scenarios: $(ls scen | wc -l)"
