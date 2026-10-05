#!/usr/bin/env python3
"""
lab6_python.py - ECE 437 Lab 6: drive the generated I2C register-read FSM
(hdl/I2C_Transmit.v, instantiated by hdl/SensorBoard_Top.v) and read one
ADT7420 register.

Transaction (ADT7420.pdf Figure 16, p.19), when param3[0] = 1:
    S, addr+W, ACK, register, ACK, Sr, addr+R, ACK, data byte, master NACK, P

Parameter / result map (the interface the HDL actually implements):
    param1[7:0]   write address byte = addr7 << 1          (0x48 -> 0x90)
                  (the FSM derives the read address {param1[7:1], 1} itself)
    param1[8]     bus: 0 = I2C bus 0 (ADT7420), 1 = I2C bus 1
    param2[7:0]   register pointer                          (0x0B = ADT7420 ID)
    param3[0]     1 = complete register read; 0 = the shipped single address
                  frame (what smoke_test.py / i2c_first_frame.py use)
    result0[0]    ACK_bit: 0 = every slave ACK received, 1 = a NACK somewhere
    result1[7:0]  data byte read from the register
    result2       clock cycles from start to done (/ 100.8 = microseconds)
    status        error (NACK -> 1), state (I2C_Transmit.State, 0 = idle), busy, done

Usage:
    python lab6_python.py [file.bit]                     # both checks: 0x48 reg 0x0B -> 0xCB, then 0x49 -> error
    python lab6_python.py [file.bit] --addr 0x49         # one read from any 7-bit address
    python lab6_python.py [file.bit] --addr 0x48 --reg 0x00 [--bus 0]
"""
import argparse
import os
import sys

# Windows: make the Opal Kelly FrontPanel SDK findable without a wrapper script
if sys.platform == "win32":
    _OK = r"C:\Program Files\Opal Kelly\FrontPanelUSB\API"
    os.environ.setdefault("OK_API_PATH", os.path.join(_OK, "Python", "x64"))
    if os.path.isdir(os.path.join(_OK, "lib", "x64")):
        os.add_dll_directory(os.path.join(_OK, "lib", "x64"))

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sensor_board import SensorBoard, parse_bitfile_arg  # noqa: E402

CLK_MHZ = 100.8
ADT7420_ADDR = 0x48
ADT7420_ID_REG = 0x0B
ADT7420_ID = 0xCB
ABSENT_ADDR = 0x49


def read_register(sb, addr7, reg, bus=0):
    """One complete register read. Returns (data, status dict, result0, cycles)."""
    r0, r1, r2 = sb.run(param1=(bus << 8) | ((addr7 << 1) & 0xFE),
                        param2=reg & 0xFF,
                        param3=1,
                        timeout=0.5)
    st = sb.status()
    return r1 & 0xFF, st, r0, r2


def show(addr7, reg, bus, data, st, r0, cycles):
    print("  address 0x%02X  register 0x%02X  bus %d" % (addr7, reg, bus))
    print("    result1[7:0] = 0x%02X" % data)
    print("    error        = %d" % st["error"])
    print("    state        = %d" % st["state"])
    print("    busy / done  = %d / %d   ACK_bit (result0[0]) = %d" % (st["busy"], st["done"], r0 & 1))
    print("    time         = %d cycles = %.1f us" % (cycles, cycles / CLK_MHZ))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bitfile", nargs="?", help="optional .bit file (default bitfile/SensorBoard_Top.bit)")
    ap.add_argument("--addr", type=lambda s: int(s, 0), help="7-bit slave address for a single read")
    ap.add_argument("--reg", type=lambda s: int(s, 0), default=ADT7420_ID_REG, help="register (default 0x0B)")
    ap.add_argument("--bus", type=int, default=0, choices=(0, 1))
    args = ap.parse_args()
    bitfile = args.bitfile if args.bitfile and args.bitfile.endswith(".bit") else parse_bitfile_arg([])

    with SensorBoard(bitfile) as sb:
        print("Board %s, serial %s, design ID 0x%08X" % (sb.model, sb.serial, sb.design_id()))
        print("Bit file: %s" % sb.bitfile)

        if args.addr is not None:                       # single, user-chosen read
            data, st, r0, cyc = read_register(sb, args.addr, args.reg, args.bus)
            show(args.addr, args.reg, args.bus, data, st, r0, cyc)
            return 0

        fails = 0
        print("\nTest 1: valid device - ADT7420 0x48, ID register 0x0B (expect 0xCB, error 0, state 0)")
        data, st, r0, cyc = read_register(sb, ADT7420_ADDR, ADT7420_ID_REG, 0)
        show(ADT7420_ADDR, ADT7420_ID_REG, 0, data, st, r0, cyc)
        ok1 = data == ADT7420_ID and not st["error"] and st["state"] == 0
        print("    -> %s" % ("PASS" if ok1 else "FAIL"))
        fails += not ok1

        print("\nTest 2: absent device - address 0x49 (expect error 1, state 0)")
        data, st, r0, cyc = read_register(sb, ABSENT_ADDR, ADT7420_ID_REG, 0)
        show(ABSENT_ADDR, ADT7420_ID_REG, 0, data, st, r0, cyc)
        ok2 = st["error"] and st["state"] == 0 and not st["busy"]
        print("    -> %s" % ("PASS" if ok2 else "FAIL"))
        fails += not ok2

        print("\nTest 3: valid device again after the NACK (bus recovered?)")
        data, st, r0, cyc = read_register(sb, ADT7420_ADDR, ADT7420_ID_REG, 0)
        show(ADT7420_ADDR, ADT7420_ID_REG, 0, data, st, r0, cyc)
        ok3 = data == ADT7420_ID and not st["error"] and st["state"] == 0
        print("    -> %s" % ("PASS" if ok3 else "FAIL"))
        fails += not ok3

        print("\nRESULT: %s" % ("ALL PASS" if fails == 0 else "%d FAIL" % fails))
        return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
