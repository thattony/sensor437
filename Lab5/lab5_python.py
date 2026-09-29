# -*- coding: utf-8 -*-
"""
ECE 437 - Lab 5
Trigger the I2C state machine and read the result back.

MILESTONE 1  the FSM is already in the FPGA, loaded over JTAG by Vivado so the
             ILA is connected. This script only pulses the GO bit so the ILA
             has something to trigger on. Leave LOAD_BITFILE = False.

MILESTONE 2  you are no longer using the ILA, so load the bitstream from here.
             Set LOAD_BITFILE = True and fill in the read-back section.
"""

import sys
import os
import time
import statistics
import csv
import matplotlib.pyplot as plt

ok_loc = r'C:\Program Files\Opal Kelly\FrontPanelUSB\API\Python\x64'
ok_dll_loc = r'C:\Program Files\Opal Kelly\FrontPanelUSB\API\lib\x64'

sys.path.append(ok_loc)
os.add_dll_directory(ok_dll_loc)

import ok

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
# Milestone 1: MUST stay False. Vivado already programmed the FPGA over JTAG,
# and calling ConfigureFPGA here would overwrite it, taking the ILA with it.
LOAD_BITFILE = True
# BITFILE = r".\Lab5.runs\impl_1\JTAG_Test_File.bit"    # relative path, do not copy bitfiles around
BITFILE = r".\JTAG_Test_File_m2.bit"    # relative path, do not copy bitfiles around

WIRE_GO = 0x00          # WireIn: bit 0 starts the FSM


# ---------------------------------------------------------------------------
# Connect
# ---------------------------------------------------------------------------
dev = ok.okCFrontPanel()

if dev.OpenBySerial("") != 0:
    sys.exit("No Opal Kelly board found. Is FrontPanel still open? Close it.")
print("FrontPanel host interface initialized.")

if LOAD_BITFILE:
    if dev.ConfigureFPGA(BITFILE) != 0:
        sys.exit("Bitfile did not load: " + BITFILE)
    print("Bitfile loaded:", BITFILE)
else:
    print("Using the bitstream already in the FPGA (loaded over JTAG).")
print("-" * 52)


# ---------------------------------------------------------------------------
# Milestone 1: pulse the GO bit
# ---------------------------------------------------------------------------
def start_fsm():
    """Pulse GO high then low.

    The FSM leaves STATE_INIT when it sees the bit high. The full version then
    waits in STATE_GO until it sees the bit go low again, so the machine runs
    once per pulse rather than continuously while the bit is held.
    """
    dev.SetWireInValue(WIRE_GO, 1)
    dev.UpdateWireIns()
    time.sleep(0.001)
    dev.SetWireInValue(WIRE_GO, 0)
    dev.UpdateWireIns()

# Uncomment for m1
# print("Arming the ILA: press Play in Vivado's waveform window BEFORE continuing.")
# input("Press Enter here once the ILA says 'Waiting for Trigger'...")

# start_fsm()
# print("GO pulse sent. The ILA should have triggered.")
# print("Look for SDA going low on the ninth SCL clock - that low is the sensor.")
# print("-" * 52)


# ---------------------------------------------------------------------------
# Milestone 2: read the result back
# ---------------------------------------------------------------------------
# Your extended FSM has to publish its received bytes on WireOuts. Add them in
# Verilog, then put the addresses you chose here. WireOuts live at 0x20 - 0x3F.
#
#   okWireOut wire20 ( .okHE(okHE), .okEH(okEHx[0*65 +: 65]),
#                      .ep_addr(8'h20), .ep_datain(your_data_register) );
#
# Remember to raise endPt_count to match the number of okWireOut modules.

WIRE_RESULT_MSB = 0x20      # <-- set to the address you used
WIRE_RESULT_LSB = 0x21      # <-- set to the address you used


def read_raw():
    """Fetch a snapshot of the WireOuts and return the two bytes."""
    dev.UpdateWireOuts()                 # without this you read stale data
    msb = dev.GetWireOutValue(WIRE_RESULT_MSB) & 0xFF
    lsb = dev.GetWireOutValue(WIRE_RESULT_LSB) & 0xFF
    return msb, lsb


def to_celsius(msb, lsb, thirteen_bit=True):
    """Convert the ADT7420's two temperature bytes to degrees Celsius.

    YOU WRITE THIS. It is about eight lines. Everything you need is in the
    temperature-register section of the datasheet.

    Two things there will catch you out:

    1. RESOLUTION MODE. The sensor powers up in 13-bit mode, where the
       temperature occupies the UPPER 13 bits of the 16-bit word. The lowest
       three bits are alarm flags, not data. In 16-bit mode all sixteen bits
       are temperature. The two modes do not share a scale factor.

    2. SIGN. The value is two's complement. Get this wrong and every reading
       above freezing looks perfect while everything below it comes back as
       roughly +500 C. Do not assume a warm bench means your code is right.

    Check yourself against the table below before you touch the board.
    """
    # raise NotImplementedError("to_celsius is yours to write")
    temperature = 0;
    if thirteen_bit:
        raw = ((msb<<8) | lsb)>>3
        if msb & 0x80:
            raw -= (1<<13)
        temperature = raw * 0.0625
    else:
        raw = (msb<<8) | lsb
        if msb & 0x80:
            raw -= (1<<16)
        temperature = raw * 0.0078125
    return temperature


# ---------------------------------------------------------------------------
# Known-answer test for your conversion. No board required.
# ---------------------------------------------------------------------------
# These are register words the ADT7420 would report in 13-bit mode. If your
# function reproduces all four, the conversion is right. If a temperature
# reading is then still wrong, the fault is in your FSM and not in here.
#
# This is the same idea as reading the ID register before the temperature:
# test one thing at a time against an answer you already know.

TEST_VECTORS_13BIT = [
    # (msb,  lsb,   expected degrees C)
    (0x0C, 0x80,  25.0000),
    (0x0B, 0x38,  22.4375),
    (0x00, 0x00,   0.0000),
    (0xFF, 0xF8,  -0.0625),     # one count below zero
    (0xF3, 0x80, -25.0000),     # the ones that matter
]


def self_test():
    """Run the vectors above. Do this before you blame the FPGA."""
    ok = True
    print(" msb  lsb    expected      yours")
    for msb, lsb, expected in TEST_VECTORS_13BIT:
        try:
            got = to_celsius(msb, lsb, thirteen_bit=True)
            good = abs(got - expected) < 1e-9
        except NotImplementedError:
            print("  to_celsius is not written yet.")
            return False
        ok &= good
        print("  0x%02X 0x%02X  %9.4f  %9.4f   %s"
              % (msb, lsb, expected, got, "ok" if good else "WRONG"))
    print(" PASS" if ok else " FAIL - fix the conversion before reading the sensor")
    return ok


# Uncomment to check your conversion without the board attached:
# self_test()


# --- Step 1: the identification register --------------------------------
# Point your FSM at the ADT7420's ID register and check the value against the
# datasheet. Do this BEFORE reading temperature: the ID is a fixed constant, so
# it is either exactly right or wrong. A temperature reading only ever looks
# plausible, which tells you nothing about whether your FSM works.

# start_fsm()
# msb, lsb = read_raw()
# print("ID register reads: 0x%02X   (datasheet value: 0xCB)" % msb)

# --- Step 2: ten temperature readings -----------------------------------
# Acceptance: every reading within +/- 2 C of the lab thermometer, AND the
# range across the ten under 0.5 C. The spread is the real test.

# print("\n reading    raw        degrees C")
# temps = []
# for i in range(10):
#     start_fsm()
#     time.sleep(0.05)                  # let the FSM finish before reading
#     msb, lsb = read_raw()
#     t = to_celsius(msb, lsb, thirteen_bit=True)
#     temps.append(t)
#     print("   %2d     0x%02X%02X     %7.4f" % (i + 1, msb, lsb, t))

# print("\n mean  %.4f C" % (sum(temps) / len(temps)))
# print(" range %.4f C   (must be under 0.5)" % (max(temps) - min(temps)))


# dev.Close()

# --- Postlab: 100 temperature readings -----------------------------------

print("\n reading    raw        degrees C")

temps = []
raw_values = []

for i in range(100):
    start_fsm()
    time.sleep(0.05)

    msb, lsb = read_raw()
    t = to_celsius(msb, lsb, thirteen_bit=True)

    temps.append(t)
    raw_values.append((msb, lsb))

    print("   %3d     0x%02X%02X     %7.4f"
          % (i + 1, msb, lsb, t))


# ---------------------------------------------------------------------------
# Statistics for Post-Lab Question 3
# ---------------------------------------------------------------------------

mean_temp = statistics.mean(temps)
std_temp = statistics.pstdev(temps)
min_temp = min(temps)
max_temp = max(temps)
temp_range = max_temp - min_temp

resolution = 0.0625    # C/count in 13-bit mode

print("\nStatistics")
print("Mean              = %.4f C" % mean_temp)
print("Standard deviation= %.4f C" % std_temp)
print("Minimum           = %.4f C" % min_temp)
print("Maximum           = %.4f C" % max_temp)
print("Full range        = %.4f C" % temp_range)
print("13-bit resolution = %.4f C/count" % resolution)

if std_temp < resolution:
    print("Std. dev. is smaller than one 13-bit count.")
else:
    print("Std. dev. is greater than or equal to one 13-bit count.")


# ---------------------------------------------------------------------------
# Save all 100 readings
# ---------------------------------------------------------------------------

with open("lab5_temperature_100.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["Sample", "MSB", "LSB", "Temperature_C"])

    for i in range(100):
        msb, lsb = raw_values[i]
        writer.writerow([
            i + 1,
            "0x%02X" % msb,
            "0x%02X" % lsb,
            "%.4f" % temps[i]
        ])


# ---------------------------------------------------------------------------
# Plot for Post-Lab Question 3
# ---------------------------------------------------------------------------

samples = range(1, 101)

plt.figure()
plt.plot(samples, temps, marker="o", markersize=3)
plt.xlabel("Sample Number")
plt.ylabel("Temperature (C)")
plt.title("100 Consecutive ADT7420 Temperature Readings")
plt.grid(True)
plt.tight_layout()
plt.savefig("lab5_temperature_plot.png", dpi=300)
plt.show()


dev.Close()
