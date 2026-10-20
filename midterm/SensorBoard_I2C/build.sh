#!/usr/bin/env bash
# ============================================================================
# build.sh - Build bitfile/SensorBoard_Top.bit with Vivado (or run the simulation).
#
#   ./build.sh            run the build in the foreground (5-20 min)
#   ./build.sh --detach   start the build in the background and return at once
#   ./build.sh --status   report whether a detached build is running / finished
#   ./build.sh --sim      run sim/tb_i2c_transmit.v in xsim (1-2 min), log: sim/sim.log
#
# Vivado is located in this order:
#   1. `vivado` already on PATH (or VIVADO_SETTINGS=/path/to/settings64.sh)
#   2. a native install under /tools/Xilinx, /opt/Xilinx or ~/Xilinx
#   3. the vivado-on-silicon-mac Docker container, if this project lives inside
#      that repository tree (host <repo> is mounted as /home/user in the container)
#
# Log: build/build.log  (ends with "BUILD COMPLETE" and "EXIT_CODE=0" on success)
# ============================================================================
set -u
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIT="SensorBoard_Top"
LOG="$PKG/build/build.log"
mkdir -p "$PKG/build"
MODE="${1:-}"

status() {
    if [ ! -f "$LOG" ]; then echo "STATUS: no build log yet ($LOG)"; return 2; fi
    if grep -q '^EXIT_CODE=' "$LOG"; then
        code=$(grep '^EXIT_CODE=' "$LOG" | tail -1 | cut -d= -f2)
        if [ "$code" = "0" ] && grep -q 'BUILD COMPLETE' "$LOG"; then
            echo "STATUS: FINISHED OK  -> $PKG/bitfile/$BIT.bit"
            grep -h 'IP BOARD =\|Post-route timing\|I2C PAD' "$LOG" | grep -v '^#' | sed 's/^/  /'
            return 0
        fi
        echo "STATUS: FAILED (exit $code)"; grep -n '^ERROR' "$LOG" | head -5; return 1
    fi
    echo "STATUS: RUNNING"; tail -2 "$LOG" | cut -c1-120 | sed 's/^/  /'; return 3
}
[ "$MODE" = "--status" ] && { status; exit $?; }

# What to run inside the Vivado environment
BUILD_CMD="vivado -mode batch -source vivado/build.tcl -log build/vivado.log -journal build/vivado.jou"
SIM_CMD="cd sim && rm -rf xsim.dir && xvlog -work work ../hdl/*.v i2c_slave_model.v tb_i2c_transmit.v && xelab -debug typical -top tb_i2c_transmit -snapshot tb_snap && xsim tb_snap -R"
if [ "$MODE" = "--sim" ]; then CMD="$SIM_CMD"; LOG="$PKG/sim/sim.log"; else CMD="$BUILD_CMD"; fi

# ---------------------------------------------------------------- native Vivado
VIVADO_CMD=""
if [ -n "${VIVADO_SETTINGS:-}" ] && [ -f "$VIVADO_SETTINGS" ]; then
    # shellcheck disable=SC1090
    source "$VIVADO_SETTINGS"
fi
if command -v vivado >/dev/null 2>&1; then
    VIVADO_CMD="vivado"
else
    for s in /tools/Xilinx/Vivado/*/settings64.sh /opt/Xilinx/Vivado/*/settings64.sh "$HOME"/Xilinx/Vivado/*/settings64.sh; do
        if [ -f "$s" ]; then
            # shellcheck disable=SC1090
            source "$s"; VIVADO_CMD="vivado"; break
        fi
    done
fi

if [ -n "$VIVADO_CMD" ]; then
    echo "Using native Vivado: $(command -v vivado)"
    cmd="cd '$PKG' && $CMD"
    if [ "$MODE" = "--detach" ]; then
        nohup bash -c "$cmd > '$LOG' 2>&1; echo EXIT_CODE=\$? >> '$LOG'" >/dev/null 2>&1 &
        echo "Build started in background. Poll with: ./build.sh --status   (log: $LOG)"
        exit 0
    fi
    bash -c "$cmd" 2>&1 | tee "$LOG"; rc=${PIPESTATUS[0]}
    echo "EXIT_CODE=$rc" >> "$LOG"; exit "$rc"
fi

# ------------------------------------------------- vivado-on-silicon-mac Docker
root="$PKG"
while [ "$root" != "/" ]; do
    if [ -f "$root/scripts/linux_start.sh" ] && ls "$root"/Xilinx/Vivado/*/settings64.sh >/dev/null 2>&1; then break; fi
    root="$(dirname "$root")"
done
if [ "$root" = "/" ]; then
    cat <<MSG
ERROR: no Vivado found.
  - Install Vivado (2024.1 verified) and put it on PATH, or set VIVADO_SETTINGS=/path/to/settings64.sh
  - On Apple Silicon, place this project inside the vivado-on-silicon-mac repository
    (e.g. <repo>/Projects/SensorBoard_Starter) so the Docker container can see it.
MSG
    exit 1
fi
if ! command -v docker >/dev/null 2>&1 || ! docker info >/dev/null 2>&1; then
    echo "ERROR: Docker is not running (needed for the vivado-on-silicon-mac container)."; exit 1
fi
ver=$(basename "$(ls -d "$root"/Xilinx/Vivado/* | tail -1)")
cpath="/home/user${PKG#"$root"}"
echo "Using vivado-on-silicon-mac container (repo: $root, Vivado $ver, container path: $cpath)"

if ! docker ps --format '{{.Names}}' | grep -qx vivado_container; then
    echo "Starting vivado_container..."
    docker run -d --init --rm --name vivado_container \
        --mount type=bind,source="$root",target=/home/user \
        -p 127.0.0.1:5901:5901 --platform linux/amd64 x64-linux \
        sudo -H -u user bash /home/user/scripts/linux_start.sh >/dev/null || exit 1
    sleep 5
fi
inner="source /home/user/Xilinx/Vivado/$ver/settings64.sh && cd '$cpath' && $CMD"
clog="${LOG#"$PKG/"}"
if [ "$MODE" = "--detach" ]; then
    docker exec -d vivado_container bash -c "$inner > $clog 2>&1; echo EXIT_CODE=\$? >> $clog"
    echo "Build started in container. Poll with: ./build.sh --status   (log: $LOG)"
    echo "When finished you may stop the container with: docker kill vivado_container"
    exit 0
fi
docker exec vivado_container bash -c "$inner" 2>&1 | tee "$LOG"; rc=${PIPESTATUS[0]}
echo "EXIT_CODE=$rc" >> "$LOG"
exit "$rc"
