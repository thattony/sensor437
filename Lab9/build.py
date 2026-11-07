#!/usr/bin/env python3
"""
build.py - the build/sim entry point of the Lab 9 package. Works the same on
Windows (Command Prompt / PowerShell), macOS and Linux; the only requirement
is that the Vivado bin directory is on PATH.

    python build.py --sim     run the two testbenches in xsim (no synthesis):
                              sim/tb_i2c_transmit.v, then sim/tb_lab9.v -
                              two RESULT: ALL PASS lines when both pass
    python build.py           build bitfile/SensorBoard_Top.bit (~15 minutes)

Tools are located with shutil.which(), which resolves the .bat wrappers on
Windows; every path is absolute via pathlib, and nothing goes through a shell.
"""
import argparse
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).absolute().parent   # not resolve(): keeps U:\ instead of a \\server UNC path cmd.exe can't cd into


def need(tool):
    p = shutil.which(tool)
    if p is None:
        print("ERROR: '%s' not found on PATH. Add the Vivado bin directory to PATH "
              "(e.g. C:\\Xilinx\\Vivado\\2024.1\\bin on Windows, or source "
              "settings64.sh on Linux/macOS) and retry." % tool)
        sys.exit(1)
    return p


def run(cmd, cwd):
    print("+ " + " ".join(str(c) for c in cmd))
    r = subprocess.run([str(c) for c in cmd], cwd=str(cwd))
    if r.returncode != 0:
        print("ERROR: '%s' exited with %d" % (Path(str(cmd[0])).name, r.returncode))
        sys.exit(r.returncode)


def sim(debug=False):
    xvlog, xelab, xsim = need("xvlog"), need("xelab"), need("xsim")
    simdir = ROOT / "sim"
    shutil.rmtree(simdir / "xsim.dir", ignore_errors=True)
    dbg = ["-debug", "typical"] if debug else ["-O3"]   # the debug kernel is 10-30x slower
    hdl = sorted((ROOT / "hdl").glob("*.v"))
    run([xvlog, "-work", "work"] + hdl
        + [simdir / "i2c_slave_model.v", simdir / "tb_i2c_transmit.v", simdir / "cmv300_model.v"], simdir)
    run([xvlog, "-sv", "-work", "work", simdir / "tb_lab9.v"], simdir)
    run([xelab] + dbg + ["-top", "tb_i2c_transmit", "-snapshot", "tb_i2c"], simdir)
    run([xsim, "tb_i2c", "-R"], simdir)
    run([xelab] + dbg + ["-top", "tb_lab9", "-snapshot", "tb_l9"], simdir)
    run([xsim, "tb_l9", "-R"], simdir)
    print("SIM DONE - look for the two RESULT lines above")


def build():
    vivado = need("vivado")
    (ROOT / "build").mkdir(exist_ok=True)
    run([vivado, "-mode", "batch",
         "-source", ROOT / "vivado" / "build.tcl",
         "-log", ROOT / "build" / "vivado.log",
         "-journal", ROOT / "build" / "vivado.jou"], ROOT)
    bit = ROOT / "bitfile" / "SensorBoard_Top.bit"
    if not bit.is_file():
        print("ERROR: no bitstream at %s - see build/vivado.log" % bit)
        sys.exit(1)
    print("BUILD COMPLETE: %s" % bit)


def main():
    ap = argparse.ArgumentParser(description="Lab 9 build / simulation")
    ap.add_argument("--sim", action="store_true", help="run the testbenches in xsim (no synthesis)")
    ap.add_argument("--debug", action="store_true", help="sim with the debug kernel + waveforms (10-30x slower)")
    a = ap.parse_args()
    sim(a.debug) if a.sim else build()


if __name__ == "__main__":
    main()
