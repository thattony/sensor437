#!/usr/bin/env python3
"""ECE 437 Lab 10 - build entry point.

    python build.py --sim     run sim/tb_lab10.v in xsim (seconds). Do this first.
    python build.py           synthesise and generate the bitstream (20-40 min:
                              this design contains a DDR3 controller).

Every acceptance number in Milestones 1 and 2 is reachable in simulation. Only
Milestone 3, which measures the real USB link, needs the board.
"""
import os, subprocess, sys

here = os.path.dirname(os.path.abspath(__file__))
args = ["--sim"] if "--sim" in sys.argv[1:] else []
sh = os.path.join(here, "build.sh")
if not os.access(sh, os.X_OK):
    os.chmod(sh, 0o755)
sys.exit(subprocess.call([sh] + args, cwd=here))
