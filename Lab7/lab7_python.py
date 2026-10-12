# -*- coding: utf-8 -*-
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
# True:  load BITFILE here. This reconfigures (resets) the FPGA, so the I2C FSM
#        writes 16-bit mode to the ADT7420 again right after loading.
# False: use the bitstream Vivado already loaded over JTAG (keeps the ILA).
LOAD_BITFILE = True
BITFILE = r".\Lab7.runs\impl_1\JTAG_Test_File.bit"    # relative path, do not copy bitfiles around

WIRE_GO = 0x00          # WireIn: rising edge on bit 0 = one temperature read

WIRE_TEMP_MSB = 0x20    # WireOut: last temperature word, MSB
WIRE_TEMP_LSB = 0x21    # WireOut: last temperature word, LSB
WIRE_CONFIG   = 0x22    # WireOut: config register read back after power-up write
WIRE_ID       = 0x23    # WireOut: ID register read during power-up
WIRE_STATUS   = 0x24    # WireOut: bit 0 init_done, bit 1 init_ack_error, bit 2 temp_ack_error
WIRE_COUNT    = 0x25    # WireOut: number of completed temperature reads (16 bit)

CONFIG_16BIT = 0x80     # config register 0x03, bit 7 = 1 -> 16-bit resolution
ADT7420_ID   = 0xCB

ROOM_MIN_C = 10.0       # broad sanity window for a bench at room temperature
ROOM_MAX_C = 40.0
N_READINGS = 10
READ_INTERVAL = 0.25    # s; one 16-bit conversion takes about 240 ms


def fail(msg):
    """Print what failed and stop cleanly."""
    print("FAIL:", msg)
    dev.Close()
    sys.exit(1)


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

if not dev.IsFrontPanelEnabled():
    fail("FrontPanel is not enabled in the FPGA design.")
print("FPGA communication: PASS")
print("-" * 52)

dev.SetWireInValue(WIRE_GO, 0)      # make sure the next 0 -> 1 is a real edge
dev.UpdateWireIns()


# ---------------------------------------------------------------------------
# Lab7 FPGA interface
# ---------------------------------------------------------------------------
def read_wires():
    """Take one snapshot of all WireOuts and return them as a dict."""
    dev.UpdateWireOuts()                 # without this you read stale data
    status = dev.GetWireOutValue(WIRE_STATUS)
    return {
        "msb":            dev.GetWireOutValue(WIRE_TEMP_MSB) & 0xFF,
        "lsb":            dev.GetWireOutValue(WIRE_TEMP_LSB) & 0xFF,
        "config":         dev.GetWireOutValue(WIRE_CONFIG) & 0xFF,
        "id":             dev.GetWireOutValue(WIRE_ID) & 0xFF,
        "init_done":      bool(status & 0x1),
        "init_ack_error": bool(status & 0x2),
        "temp_ack_error": bool(status & 0x4),
        "count":          dev.GetWireOutValue(WIRE_COUNT) & 0xFFFF,
    }


def start_fsm():
    """Pulse GO high then low.

    The FSM acts on the rising edge only, so this gives exactly one temperature
    read no matter how long the bit stays high.
    """
    dev.SetWireInValue(WIRE_GO, 1)
    dev.UpdateWireIns()
    time.sleep(0.001)
    dev.SetWireInValue(WIRE_GO, 0)
    dev.UpdateWireIns()


def read_temperature(timeout=0.5):
    """Trigger one read and return the raw 16-bit word from THAT transaction.

    The FPGA bumps the read counter only after the new word is latched, so a
    counter step of exactly one means one new transaction finished and the data
    is not left over from the previous read.
    """
    before = read_wires()["count"]
    start_fsm()
    t0 = time.time()
    while read_wires()["count"] == before:
        if time.time() - t0 > timeout:
            fail("no I2C transaction completed within %.1f s of the trigger" % timeout)
    w = read_wires()                     # second snapshot: data has been stable since the count stepped
    step = (w["count"] - before) & 0xFFFF
    if step != 1:
        fail("one trigger produced %d transactions" % step)
    if w["temp_ack_error"]:
        fail("sensor did not ACK during the temperature read")
    return (w["msb"] << 8) | w["lsb"]


def to_celsius(raw):
    """Convert the ADT7420's 16-bit-mode temperature word to degrees Celsius.

    In 16-bit mode all sixteen bits are temperature, two's complement,
    1 LSB = 1/128 = 0.0078125 deg C.
    """
    if raw & 0x8000:
        raw -= (1 << 16)
    return raw * 0.0078125


# ---------------------------------------------------------------------------
# Known-answer test for your conversion. No board required.
# ---------------------------------------------------------------------------
# 16-bit-mode register words from the ADT7420 datasheet temperature table.
TEST_VECTORS_16BIT = [
    # (raw,   expected degrees C)
    (0x4B00,  150.0000),
    (0x0C80,   25.0000),
    (0x0001,    0.0078125),     # one count above zero
    (0x0000,    0.0000),
    (0xFFFF,   -0.0078125),     # one count below zero
    (0xF380,  -25.0000),
    (0xEC00,  -40.0000),
]


def self_test():
    """Run the vectors above. Do this before you blame the FPGA."""
    ok = True
    print("  raw      expected       yours")
    for raw, expected in TEST_VECTORS_16BIT:
        got = to_celsius(raw)
        good = abs(got - expected) < 1e-9
        ok &= good
        print("  0x%04X  %11.7f  %11.7f   %s"
              % (raw, expected, got, "ok" if good else "WRONG"))
    return ok


print("Conversion known-answer test:")
if not self_test():
    fail("to_celsius does not match the datasheet table")
print("Conversion: PASS")
print("-" * 52)


# --- Step 1: wait for the FPGA's power-up sequence -------------------------
# After reset the FSM writes 0x80 to the config register, reads it back, and
# reads the ID register, all before it accepts a trigger.
t0 = time.time()
while True:
    w = read_wires()
    if w["init_done"]:
        break
    if time.time() - t0 > 1.0:
        if LOAD_BITFILE and os.path.getmtime(BITFILE) < os.path.getmtime(
                r".\Lab7.srcs\sources_1\imports\Downloads\I2C_Transmit.v"):
            print("NOTE: %s is older than I2C_Transmit.v - regenerate the bitstream." % BITFILE)
        fail("ADT7420 initialization did not complete (init_done never set, status = 0x%X)"
             % dev.GetWireOutValue(WIRE_STATUS))

if w["init_ack_error"]:
    fail("sensor did not ACK during initialization")
print("ADT7420 initialization: PASS")

if w["config"] != CONFIG_16BIT:
    fail("config register reads 0x%02X, expected 0x%02X (resolution bit 7 %s)"
         % (w["config"], CONFIG_16BIT, "set" if w["config"] & 0x80 else "NOT set"))
print("Resolution mode: 16-bit   (config register 0x03 = 0x%02X)" % w["config"])

if w["id"] != ADT7420_ID:
    fail("ID register reads 0x%02X, expected 0x%02X" % (w["id"], ADT7420_ID))
print("Sensor ID: PASS           (ID register 0x0B = 0x%02X)" % w["id"])
print("-" * 52)

time.sleep(READ_INTERVAL)               # let the first 16-bit conversion finish


# --- Step 2: one temperature reading ---------------------------------------
count0 = read_wires()["count"]
raw = read_temperature()
t = to_celsius(raw)
print("Raw temperature: 0x%04X" % raw)
print("Temperature: %.4f C" % t)
if not (ROOM_MIN_C <= t <= ROOM_MAX_C):
    fail("%.4f C is outside the room-temperature window %.0f to %.0f C"
         % (t, ROOM_MIN_C, ROOM_MAX_C))
print("Room-temperature check: PASS")
print("-" * 52)


# --- Step 3: more readings, one transaction each ---------------------------
print(" reading    raw        degrees C")
temps = []
for i in range(N_READINGS):
    time.sleep(READ_INTERVAL)
    raw = read_temperature()
    t = to_celsius(raw)
    if not (ROOM_MIN_C <= t <= ROOM_MAX_C):
        fail("reading %d: %.4f C is outside %.0f to %.0f C" % (i + 1, t, ROOM_MIN_C, ROOM_MAX_C))
    temps.append(t)
    print("   %2d     0x%04X     %8.4f" % (i + 1, raw, t))

done = (read_wires()["count"] - count0) & 0xFFFF
if done != N_READINGS + 1:
    fail("%d reads requested but the FPGA completed %d transactions" % (N_READINGS + 1, done))
print("\n %d reads requested, %d I2C transactions completed: PASS" % (N_READINGS + 1, done))
print(" mean   %.4f C" % statistics.mean(temps))
print(" stdev  %.4f C" % statistics.stdev(temps))
print(" range  %.4f C" % (max(temps) - min(temps)))
print("-" * 52)
print("ALL SELF-TESTS PASSED")

dev.Close()
