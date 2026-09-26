# -*- coding: utf-8 -*-

import time
import sys
import os

# ------------------------------------------------------------
# FrontPanel setup
# ------------------------------------------------------------

ok_loc = r'C:\Program Files\Opal Kelly\FrontPanelUSB\API\Python\x64'
ok_dll_loc = r'C:\Program Files\Opal Kelly\FrontPanelUSB\API\lib\x64'

sys.path.append(ok_loc)
os.add_dll_directory(ok_dll_loc)

import ok


# ============================================================
# SETTINGS
# ============================================================

# This bitstream MUST contain:
# if (clkdiv == divide - 1)
BITFILE = r"lab2_example_equal.bit"

SLOW_DIVIDE = 100_000_000     # ~1 counter step/s
FAST_DIVIDE = 10_000_000      # ~10 counter steps/s

NUM_TRIALS = 5

CLK_FREQ = 200_000_000


# ============================================================
# Helper functions
# ============================================================

def configure_board(divide):
    """Configure FPGA and start mode 2 with the requested divider."""

    dev = ok.okCFrontPanel()

    status = dev.OpenBySerial("")
    if status != 0:
        print("Could not open FPGA. Error:", status)
        sys.exit()

    status = dev.ConfigureFPGA(BITFILE)
    if status != 0:
        print("Could not configure FPGA. Error:", status)
        dev.Close()
        sys.exit()

    # Mode 2 = count upward by 2
    dev.SetWireInValue(0x02, 2)

    # programmable divider
    dev.SetWireInValue(0x03, divide)

    # host reset disabled
    dev.SetWireInValue(0x04, 0)

    dev.UpdateWireIns()

    return dev


def read_counter(dev):
    dev.UpdateWireOuts()
    return dev.GetWireOutValue(0x22)


def wait_for_counter_change(dev):
    """Wait until the counter advances once."""

    old = read_counter(dev)

    while True:
        new = read_counter(dev)

        if new != old:
            return new


# ============================================================
# Q3(a)
# == version: slow -> fast
# ============================================================

print()
print("====================================================")
print("Q3(a): == VERSION, SLOW -> FAST")
print("====================================================")
print("Start: divide =", SLOW_DIVIDE, "(~1 step/s)")
print("Change to:", FAST_DIVIDE, "(~10 steps/s)")
print()

q3a_results = []


for trial in range(1, NUM_TRIALS + 1):

    dev = configure_board(SLOW_DIVIDE)

    # Wait for a known rising edge of slow_clk.
    counter_before = wait_for_counter_change(dev)

    # After a slow_clk rising edge, clkdiv was just reset to 0.
    #
    # Wait 0.20 s:
    # clkdiv is now approximately:
    #
    # 200 MHz * 0.20 s = 40,000,000
    #
    # This is already GREATER than the new threshold
    # 10,000,000 - 1.
    #
    # Therefore, with ==, the new threshold has been missed.
    time.sleep(0.20)

    before_change = read_counter(dev)

    # Start timing the divider change
    start = time.time()

    dev.SetWireInValue(0x03, FAST_DIVIDE)
    dev.UpdateWireIns()

    # Wait until the counter finally changes.
    # With the == bug, clkdiv must wrap around first.
    while True:

        counter = read_counter(dev)

        if counter != before_change:
            end = time.time()
            break

    elapsed = end - start
    q3a_results.append(elapsed)

    print(
        f"Trial {trial}: {elapsed:.6f} seconds"
    )

    dev.Close()


# ============================================================
# Q3(b)
# Calculate full clkdiv wrap time
# ============================================================

wrap_cycles = 2 ** 32
wrap_time = wrap_cycles / CLK_FREQ

print()
print("====================================================")
print("Q3(b): 32-BIT CLKDIV WRAP")
print("====================================================")

print("clkdiv width = 32 bits")
print("Full wrap cycles =", wrap_cycles)
print("Clock frequency =", CLK_FREQ, "Hz")
print("Full wrap time =", wrap_time, "seconds")


# ============================================================
# Q3(c)
# == version: fast -> slow
# ============================================================

print()
print("====================================================")
print("Q3(c): == VERSION, FAST -> SLOW")
print("====================================================")
print("Start: divide =", FAST_DIVIDE, "(~10 steps/s)")
print("Change to:", SLOW_DIVIDE, "(~1 step/s)")
print()

q3c_results = []


for trial in range(1, NUM_TRIALS + 1):

    dev = configure_board(FAST_DIVIDE)

    # Wait for a known counter rising edge.
    counter_before = wait_for_counter_change(dev)

    # clkdiv just reset to approximately 0.
    #
    # Wait only 0.02 s:
    # clkdiv is approximately 4,000,000,
    # which is still BELOW the new slow threshold
    # of 99,999,999.
    time.sleep(0.02)

    before_change = read_counter(dev)

    start = time.time()

    dev.SetWireInValue(0x03, SLOW_DIVIDE)
    dev.UpdateWireIns()

    # Wait for first counter update after changing to slow rate.
    while True:

        counter = read_counter(dev)

        if counter != before_change:
            end = time.time()
            break

    elapsed = end - start
    q3c_results.append(elapsed)

    print(
        f"Trial {trial}: {elapsed:.6f} seconds"
    )

    dev.Close()


# ============================================================
# Final copy/paste block
# ============================================================

print()
print()
print("####################################################")
print("COPY EVERYTHING BELOW THIS LINE TO CHATGPT")
print("####################################################")

print()
print("Q3(a) == slow -> fast")
for i, value in enumerate(q3a_results, start=1):
    print(f"Run {i}: {value:.6f} s")

print()
print("Q3(b)")
print(f"32-bit clkdiv full wrap = {wrap_time:.8f} s")

print()
print("Q3(c) == fast -> slow")
for i, value in enumerate(q3c_results, start=1):
    print(f"Run {i}: {value:.6f} s")

print()
print("####################################################")