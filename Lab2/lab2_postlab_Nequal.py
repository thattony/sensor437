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

# This bitstream MUST use:
# if (clkdiv >= divide - 1)
BITFILE = r"lab2_m2.bit"

SLOW_DIVIDE = 100_000_000     # about 1 step/s
FAST_DIVIDE = 10_000_000      # about 10 steps/s

NUM_TRIALS = 5


# ============================================================
# Connect to FPGA
# ============================================================

dev = ok.okCFrontPanel()

status = dev.OpenBySerial("")

if status != 0:
    print("Could not open FPGA. Error:", status)
    sys.exit()


# ============================================================
# Helper functions
# ============================================================

def configure_fpga():

    # Retry in case configuration temporarily fails
    for attempt in range(3):

        status = dev.ConfigureFPGA(BITFILE)

        if status == 0:
            time.sleep(0.2)
            return

        print(
            "Configure attempt",
            attempt + 1,
            "failed with error",
            status
        )

        time.sleep(1)

    print("Could not configure FPGA.")
    dev.Close()
    sys.exit()


def set_inputs(divide):

    # mode 2 = count upward by 2
    dev.SetWireInValue(0x02, 2)

    # programmable divider
    dev.SetWireInValue(0x03, divide)

    # reset low
    dev.SetWireInValue(0x04, 0)

    dev.UpdateWireIns()


def read_counter():

    dev.UpdateWireOuts()
    return dev.GetWireOutValue(0x22)


def wait_for_counter_change():

    old = read_counter()

    while True:

        new = read_counter()

        if new != old:
            return new


# ============================================================
# Q3(d)
# >= version: slow -> fast
# ============================================================

print()
print("====================================================")
print("Q3(d): >= VERSION, SLOW -> FAST")
print("====================================================")
print("Start divide =", SLOW_DIVIDE, "(~1 step/s)")
print("New divide   =", FAST_DIVIDE, "(~10 steps/s)")
print()

q3d_results = []


for trial in range(1, NUM_TRIALS + 1):

    # Fresh FPGA state for every trial
    configure_fpga()
    set_inputs(SLOW_DIVIDE)

    # Wait until one counter update occurs.
    # This gives us a known slow_clk rising edge.
    wait_for_counter_change()

    # Wait 0.2 s so clkdiv is already well ABOVE
    # the new FAST_DIVIDE threshold.
    #
    # This is exactly where == would fail.
    time.sleep(0.20)

    before = read_counter()

    # Change to faster divider and start timing
    start = time.perf_counter()

    dev.SetWireInValue(0x03, FAST_DIVIDE)
    dev.UpdateWireIns()

    # Wait for the counter to advance at the new rate
    while True:

        counter = read_counter()

        if counter != before:
            elapsed = time.perf_counter() - start
            break

    q3d_results.append(elapsed)

    print(
        f"Trial {trial}: {elapsed:.6f} seconds"
    )


# ============================================================
# Q3(e)
# >= version with divide = 0
# ============================================================

print()
print("====================================================")
print("Q3(e): >= VERSION, DIVIDE = 0")
print("====================================================")

# Fresh FPGA configuration
configure_fpga()

# Mode 2, divider = 0, reset = 0
set_inputs(0)

initial_counter = read_counter()

print("Initial counter =", initial_counter)
print("Sent divide = 0")
print("Waiting to see what happens...")

start = time.perf_counter()

counter_changed = False
changed_counter = initial_counter
change_time = None

# Wait up to 30 seconds.
#
# With your current expression:
#
#     divide - 1
#
# divide = 0 underflows to 0xFFFFFFFF.
#
# Therefore a very long delay may occur.
timeout = 30.0

while time.perf_counter() - start < timeout:

    counter = read_counter()

    if counter != initial_counter:

        counter_changed = True
        changed_counter = counter
        change_time = time.perf_counter() - start
        break


if counter_changed:

    print()
    print("Counter eventually changed.")
    print("New counter =", changed_counter)
    print(
        "Time until counter changed =",
        change_time,
        "seconds"
    )

else:

    print()
    print(
        "Counter did NOT change within",
        timeout,
        "seconds."
    )


# ============================================================
# FINAL COPY/PASTE RESULTS
# ============================================================

print()
print()
print("####################################################")
print("COPY EVERYTHING BELOW THIS LINE TO CHATGPT")
print("####################################################")

print()
print("Q3(d) >= slow -> fast")

for i, value in enumerate(q3d_results, start=1):
    print(f"Run {i}: {value:.6f} s")


print()
print("Q3(e) divide = 0")

print("Initial counter:", initial_counter)

if counter_changed:

    print("Counter changed to:", changed_counter)
    print(
        f"Time until change: {change_time:.6f} s"
    )

else:

    print(
        f"No counter change within {timeout:.1f} s"
    )


print()
print("####################################################")


# ------------------------------------------------------------
# Close FPGA
# ------------------------------------------------------------

dev.Close()