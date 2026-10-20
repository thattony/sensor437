#!/usr/bin/env python3
"""
smoke_test.py - Verify the toolchain end to end with the unmodified I2C framework.

Usage:  python smoke_test.py [path/to/file.bit]     (default: ../bitfile/SensorBoard_Top.bit)

Needs the sensor board plugged into the XEM7310 (it talks to the ADT7420 and LSM303).
Checks: board opens, bit file loads, FrontPanel enabled, design ID, one I2C frame to the
ADT7420 (must ACK), one to an empty address (must NACK -> error), one on bus 1 (LSM303
accel must ACK), busy/done handshake, dbg_state, transaction time, TriggerOut, PipeOut,
buttons, sensor-board LEDs. If it passes, the build flow, USB link, endpoint map and both
I2C buses are fine and any later problem is in the state machine you are extending.
"""
import sys
import time
from sensor_board import SensorBoard, DESIGN_ID, parse_bitfile_arg

fails = 0


def check(name, ok, detail=""):
    global fails
    print("  [%s] %s %s" % ("PASS" if ok else "FAIL", name, detail))
    if not ok:
        fails += 1


def frame(sb, bus, byte):
    r0, r1, r2 = sb.run(param1=(bus << 8) | byte, timeout=0.5)
    return r0, r2, sb.status()


sb = SensorBoard(parse_bitfile_arg())
print("Board: %s  serial: %s" % (sb.model, sb.serial))
print("Bit file: %s" % sb.bitfile)
print("FrontPanel enabled: True")

did = sb.design_id()
check("design ID", did == DESIGN_ID, "0x%08X (expected 0x%08X)" % (did, DESIGN_ID))

sb.reset()
st = sb.status()
check("idle after reset", not st["busy"] and not st["done"] and st["state"] == 0, str(st))

# frame 1: ADT7420 write address on bus 0 -> ACK
r0, r2, st = frame(sb, 0, 0x90)
check("ADT7420 0x48 ACKs on bus 0 (result0[0] == 0)", (r0 & 1) == 0, "result0=0x%08X" % r0)
check("error = 0 after ACK", not st["error"], str(st))
check("byte/bus echoed in result0", ((r0 >> 8) & 0xFF) == 0x90 and ((r0 >> 16) & 1) == 0, "0x%08X" % r0)
check("transaction time ~103 us (41 ticks x 2.5 us)", 10000 <= r2 <= 11100, "%d cycles = %.1f us" % (r2, r2 / 100.8))
check("done set, busy clear, state back to 0", st["done"] and not st["busy"] and st["state"] == 0, str(st))
check("TriggerOut 0x60 fired", sb.done_triggered())

# frame 2: nobody at 0x49 -> NACK, error
r0, r2, st = frame(sb, 0, 0x92)
check("empty address 0x49 NACKs (result0[0] == 1)", (r0 & 1) == 1, "result0=0x%08X" % r0)
check("error = 1 after NACK", st["error"], str(st))

# frame 3: LSM303 accelerometer write address on bus 1 -> ACK
r0, r2, st = frame(sb, 1, 0x32)
check("LSM303 accel 0x19 ACKs on bus 1", (r0 & 1) == 0 and ((r0 >> 16) & 1) == 1, "result0=0x%08X" % r0)

sb.reset()
buf = sb.read_pipe(64)
words = [int.from_bytes(buf[i:i + 4], "little") for i in range(0, len(buf), 4)]
check("PipeOut 0xA0 counter", len(words) == 16 and all(words[i + 1] - words[i] == 1 for i in range(15)),
      "first words %s" % [hex(w) for w in words[:4]])

st = sb.status()
print("  buttons (1 = pressed): SW3..SW0 = %s   (press one and rerun to see it change)" % format(st["user"] & 0xF, "04b"))

print("  sensor-board LEDs D3..D0: walking pattern for 2 s ...")
for i in range(8):
    sb.set_params(ctrl_user=1 << (i % 4))
    time.sleep(0.25)
sb.set_params(ctrl_user=0)

print()
print("RESULT: %s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
sb.close()
sys.exit(1 if fails else 0)
