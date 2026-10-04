#!/usr/bin/env python3
"""
example_i2c_read.py - Template for a script that talks to YOUR sensor FSM,
once it can do complete register reads. The framework as shipped only sends
one address frame (see i2c_first_frame.py and the header of hdl/SensorBoard_Top.v),
so this script will report NACK/garbage until you have extended I2C_Transmit.v.

It assumes the classic ECE 437 "transaction word" convention in param1
(change it to whatever you define in I2C_Transmit.v / SensorBoard_Top.v):
    param1[31:24] = 7-bit slave address << 1 | 0 (write)
    param1[23:16] = register address
    param1[15:8]  = 7-bit slave address << 1 | 1 (read)
    param1[7]     = 1 for a register write, 0 for a read
    param1[6:0]   = number of bytes
    result0       = first 4 bytes read (first byte in [31:24])
    status.error  = 1 if the slave NACKed

Usage:  python example_i2c_read.py [file.bit]
"""
import time
from sensor_board import SensorBoard, parse_bitfile_arg


def txn(sb, addr7, reg, nbytes=1, write=False):
    word = ((addr7 << 1) << 24) | (reg << 16) | (((addr7 << 1) | 1) << 8) | ((1 if write else 0) << 7) | (nbytes & 0x7F)
    r0, r1, r2 = sb.run(param1=word, timeout=0.5)
    st = sb.status()
    return r0, r1, r2, st


with SensorBoard(parse_bitfile_arg()) as sb:
    print("Board %s, design ID 0x%08X" % (sb.model, sb.design_id()))

    # ADT7420 ID register (0x0B) must read 0xCB when the bus and sensor are healthy
    r0, r1, r2, st = txn(sb, 0x48, 0x0B, 1)
    print("ADT7420 ID  : 0x%02X  (expect 0xCB)  error=%s  %d cycles" % (r0 >> 24, st["error"], r2))

    # Temperature: registers 0x00/0x01, 13-bit two's complement in [15:3], 0.0625 C/LSB
    for _ in range(5):
        r0, r1, r2, st = txn(sb, 0x48, 0x00, 2)
        raw16 = r0 >> 16
        raw13 = raw16 >> 3
        if raw13 & 0x1000:
            raw13 -= 0x2000
        print("Temperature : %.4f C  (raw 0x%04X, %d cycles = %.1f us)" % (raw13 * 0.0625, raw16, r2, r2 / 100.8))
        time.sleep(0.5)
