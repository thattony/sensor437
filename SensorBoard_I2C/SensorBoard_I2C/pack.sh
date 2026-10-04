#!/usr/bin/env bash
# pack.sh - zip this project for distribution, without build products, the Python
# venv, simulation artefacts and any bitstream (students build their own). Result: ../<folder>.zip
set -eu
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME="$(basename "$PKG")"
OUT="$(dirname "$PKG")/$NAME.zip"
rm -f "$OUT"
cd "$(dirname "$PKG")"
zip -r -q "$OUT" "$NAME" \
    -x "$NAME/bitfile/*.bit" "$NAME/build/*" "$NAME/.venv/*" "$NAME/sim/xsim.dir/*" "$NAME/sim/*.wdb" "$NAME/sim/*.vcd" \
       "$NAME/sim/*.pb" "$NAME/sim/*.log" "$NAME/sim/*.jou" "$NAME/sim/*.out" "$NAME/sim/webtalk*" \
       "$NAME/.Xil/*" "$NAME/*.jou" "$NAME/*.log" "$NAME/*.str" "$NAME/vivado_*.backup.*" \
       "*/__pycache__/*" "*.pyc" "*/.DS_Store" "$NAME/.git/*"
echo "Packed: $OUT ($(du -h "$OUT" | cut -f1))"
unzip -l "$OUT" | tail -1
