#!/usr/bin/env bash
# pack.sh - build SensorBoard_CMV300_Capture.zip for students.
# Stages outside the project so the zip never has to replace a file in place.
# Excludes solution/, _not_shipped/, build products and simulation artefacts.
set -eu
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME="$(basename "$PKG")"
STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT

mkdir -p "$STAGE/$NAME"
( cd "$PKG" && tar cf - \
    --exclude='solution' --exclude='_not_shipped' \
    --exclude='build' --exclude='bitfile' --exclude='.venv' --exclude='.git' --exclude='.Xil' \
    --exclude='__pycache__' --exclude='*.pyc' --exclude='.DS_Store' \
    --exclude='sim/xsim.dir' --exclude='sim/*.wdb' --exclude='sim/*.vcd' \
    --exclude='sim/*.pb' --exclude='sim/*.log' --exclude='sim/*.jou' \
    --exclude='*.jou' --exclude='*.log' --exclude='*.str' \
    . ) | ( cd "$STAGE/$NAME" && tar xf - )

( cd "$STAGE" && zip -r -q "$NAME.zip" "$NAME" )
cp "$STAGE/$NAME.zip" "$(dirname "$PKG")/$NAME.zip"
echo "Packed: $(dirname "$PKG")/$NAME.zip ($(du -h "$STAGE/$NAME.zip" | cut -f1))"
echo
echo "--- contents ---"
( cd "$STAGE" && unzip -l "$NAME.zip" | awk 'NR>3 && NF>3 {print "  " $4}' | grep -v '/$' | sort )
echo
echo "--- leak check: none of these may appear ---"
if ( cd "$STAGE" && unzip -l "$NAME.zip" ) | grep -qE 'solution/|_not_shipped/|CMV300_Lvds|Sensor_Sequencer|CMV300_Ctrl\.v|CMV300_Spi\.v'; then
    echo "  *** LEAK ***"; ( cd "$STAGE" && unzip -l "$NAME.zip" ) | grep -E 'solution/|_not_shipped/|CMV300_Lvds|Sensor_Sequencer|CMV300_Ctrl\.v|CMV300_Spi\.v'
    exit 1
else
    echo "  clean"
fi
