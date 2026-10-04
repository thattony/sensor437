#!/usr/bin/env python3
"""
i2c_first_frame.py - Drive the I2C framework (hdl/I2C_Transmit.v, instantiated by hdl/SensorBoard_Top.v).

The FPGA sends ONE I2C frame per start: START, the byte {7-bit address, R/W},
read the ACK in the 9th clock, STOP. That is the first frame of every I2C
transaction and enough to find out which sensors answer on which bus.

Parameter / result map (defined in the header of hdl/SensorBoard_Top.v - keep this file in step):
    param1[7:0]   byte to send = (addr7 << 1) | rw     e.g. 0x90 = ADT7420 write
    param1[8]     bus: 0 = I2C bus 0 (ADT7420 0x48, HTS221 0x5F, LPS35HW 0x5C)
                       1 = I2C bus 1 (LSM303 accel 0x19 / mag 0x1E, AD7156 0x48)
    result0[0]    ACK bit: 0 = the slave pulled SDA low (ACK), 1 = nobody there (NACK)
    result0[15:8] byte that was sent, result0[16] bus used
    result2       clock cycles from start to done (/100.8 = microseconds)
    status.error  1 = NACK (or the FSM hit its default arm)
    status.state  I2C_Transmit.State (0 idle, 1-2 START, 3-34 bits, 35-38 ACK, 39-41 STOP)

Usage:
    python i2c_first_frame.py [file.bit]                 # ADT7420 ACK/NACK check, then scan both buses
    python i2c_first_frame.py [file.bit] 0x48            # one frame: address 0x48 (write) on bus 0
    python i2c_first_frame.py [file.bit] 0x19 --bus 1    # one frame on bus 1
    python i2c_first_frame.py [file.bit] 0x48 --read     # send the READ address (see note below)

Send the WRITE address as a stand-alone frame. After ACKing a READ address the
slave immediately drives data bit 7 on SDA; if that bit is 0 it holds SDA low and
the STOP cannot happen - the bus stays stuck until the byte is clocked out and
NACKed (ADT7420.pdf Figure 17, p.19). Reading data is the next thing to build.
"""
import sys
from sensor_board import SensorBoard, parse_bitfile_arg

CLK_MHZ = 100.8

# Every I2C part on the ECE 437 sensor board (docs/SENSOR_BOARD.md)
KNOWN = {
    (0, 0x48): "ADT7420 temperature",
    (0, 0x5F): "HTS221 humidity",
    (0, 0x5C): "LPS35HW pressure",
    (1, 0x19): "LSM303DLHC accelerometer",
    (1, 0x1E): "LSM303DLHC magnetometer",
    (1, 0x48): "AD7156 capacitance",
}


def send_frame(sb, addr7, bus=0, read=False, timeout=0.5):
    """One frame. Returns (acked, cycles, status)."""
    byte = ((addr7 & 0x7F) << 1) | (1 if read else 0)
    word = ((bus & 1) << 8) | byte
    r0, r1, r2 = sb.run(param1=word, timeout=timeout)
    st = sb.status()
    acked = (r0 & 1) == 0
    if (r0 >> 8) & 0xFF != byte or ((r0 >> 16) & 1) != (bus & 1):
        print("  ! result0 echo mismatch: sent bus %d byte 0x%02X, FPGA reports bus %d byte 0x%02X"
              % (bus, byte, (r0 >> 16) & 1, (r0 >> 8) & 0xFF))
    return acked, r2, st


def scan(sb, bus):
    """Send the write address to every non-reserved 7-bit address; list who ACKs."""
    found = []
    for addr in range(0x08, 0x78):                 # 0x00-0x07 and 0x78-0x7F are reserved
        acked, cycles, st = send_frame(sb, addr, bus)
        if acked:
            found.append(addr)
            print("  bus %d  0x%02X  ACK   %s" % (bus, addr, KNOWN.get((bus, addr), "(unknown device)")))
    return found


def main():
    bitfile = parse_bitfile_arg()
    args = [a for a in sys.argv[1:] if not a.endswith(".bit")]
    bus = 0
    read = "--read" in args
    if "--bus" in args:
        bus = int(args[args.index("--bus") + 1], 0)
    addr_args = [a for a in args if not a.startswith("--") and a not in ("0", "1")]

    with SensorBoard(bitfile) as sb:
        print("Board %s, design ID 0x%08X" % (sb.model, sb.design_id()))
        sb.reset()

        if addr_args:
            addr7 = int(addr_args[0], 0)
            acked, cycles, st = send_frame(sb, addr7, bus, read)
            print("bus %d  0x%02X %s -> %s   %d cycles = %.1f us   error=%s state=%d"
                  % (bus, addr7, "READ" if read else "WRITE", "ACK" if acked else "NACK",
                     cycles, cycles / CLK_MHZ, st["error"], st["state"]))
            return

        # 1. the temperature sensor must answer, an empty address must not
        acked, cycles, st = send_frame(sb, 0x48, 0)
        print("ADT7420 0x48 write on bus 0 : %s  (%d cycles = %.1f us, error=%s)  expect ACK"
              % ("ACK" if acked else "NACK", cycles, cycles / CLK_MHZ, st["error"]))
        acked, cycles, st = send_frame(sb, 0x49, 0)
        print("address 0x49 on bus 0       : %s  (error=%s)  expect NACK - nobody there"
              % ("ACK" if acked else "NACK", st["error"]))

        # 2. who is on the buses?
        print("Scanning bus 0 ...")
        f0 = scan(sb, 0)
        print("Scanning bus 1 ...")
        f1 = scan(sb, 1)
        print("Found %d device(s) on bus 0, %d on bus 1 (board has 3 + 3)." % (len(f0), len(f1)))
        missing = [k for k in KNOWN if k[1] not in (f0 if k[0] == 0 else f1)]
        for bus_, addr in missing:
            print("  missing: bus %d 0x%02X %s" % (bus_, addr, KNOWN[(bus_, addr)]))


if __name__ == "__main__":
    main()
