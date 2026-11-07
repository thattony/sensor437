#!/usr/bin/env python3
"""
temperature.py - Talk to the sensors over I2C: read the ADT7420 temperature, read/write any
register of any sensor, or scan both buses for every device that answers
(hdl/I2C_Transmit.v: START, SAD+W, register, [data | repeated START, SAD+R, bytes], STOP).

Usage:
    python temperature.py [file.bit]                 # ID check, then print the temperature every 0.5 s (Ctrl-C to stop)
    python temperature.py [file.bit] read  0x0B      # read one register (add a byte count: read 0x00 2)
    python temperature.py [file.bit] write 0x03 0x80 # write one register (0x80 = 16-bit mode, see datasheet Table 11 p.14)
    python temperature.py [file.bit] scan            # list every I2C device that ACKs on bus 0 and bus 1 (all six parts)
    python temperature.py [file.bit] --addr 0x5F read 0x0F   # any other sensor on bus 0 (--bus 1 for bus 1)

Registers (ADT7420.pdf Table 6 p.13): 0x00/0x01 temperature MSB/LSB, 0x02 status,
0x03 configuration (bit 7: 0 = 13-bit, 1 = 16-bit), 0x0B ID = 0xCB.
"""
import sys
import time
from sensor_board import SensorBoard, parse_bitfile_arg

ADT7420 = 0x48

# Every I2C part on the ECE 437 sensor board (docs/SENSOR_BOARD.md)
KNOWN = {
    (0, 0x48): "ADT7420 temperature",
    (0, 0x5F): "HTS221 humidity",
    (0, 0x5C): "LPS35HW pressure",
    (1, 0x19): "LSM303DLHC accelerometer",
    (1, 0x1E): "LSM303DLHC magnetometer",
    (1, 0x48): "AD7156 capacitance",
}


def fmt(info):
    return "acks=%s error=%s %.1f us" % (format(info["acks"], "04b"), info["error"], info["us"])


def convert(msb, lsb, bits16=False):
    raw16 = (msb << 8) | lsb
    if bits16:                               # 16-bit mode: 1 LSB = 1/128 C
        return (raw16 - 0x10000 if raw16 & 0x8000 else raw16) / 128.0, raw16
    raw13 = raw16 >> 3                       # 13-bit mode (default): bits [15:3], 1 LSB = 0.0625 C
    if raw13 & 0x1000:
        raw13 -= 0x2000
    return raw13 * 0.0625, raw16


def scan(sb):
    """One 1-byte read of register 0 at every address on both buses; present = frame 1 (SAD+W) ACKed."""
    found = 0
    for bus in (0, 1):
        print("bus %d:" % bus)
        for addr in range(0x08, 0x78):                 # 0x00-0x07 and 0x78-0x7F are reserved
            data, info = sb.i2c_read(addr, 0x00, 1, bus)
            if (info["acks"] & 1) == 0:
                found += 1
                print("  0x%02X  ACK   %-28s (reg 0x00 = 0x%02X, %.1f us)" % (addr, KNOWN.get((bus, addr), "(unknown device)"), data[0], info["us"]))
    print("%d device(s) found (the board has %d I2C parts)" % (found, len(KNOWN)))


def main():
    bitfile = parse_bitfile_arg()
    args = [a for a in sys.argv[1:] if not a.endswith(".bit")]
    addr, bus = ADT7420, 0
    if "--addr" in args:
        i = args.index("--addr"); addr = int(args[i + 1], 0); del args[i:i + 2]
    if "--bus" in args:
        i = args.index("--bus"); bus = int(args[i + 1], 0); del args[i:i + 2]

    with SensorBoard(bitfile) as sb:
        print("Board %s, design ID 0x%08X" % (sb.model, sb.design_id()))
        sb.reset()

        if args and args[0] == "scan":
            scan(sb)
            return
        if args and args[0] == "read":
            reg = int(args[1], 0); n = int(args[2], 0) if len(args) > 2 else 1
            data, info = sb.i2c_read(addr, reg, n, bus)
            print("read  0x%02X reg 0x%02X -> %s   (%s)" % (addr, reg, " ".join("0x%02X" % b for b in data), fmt(info)))
            return
        if args and args[0] == "write":
            reg = int(args[1], 0); val = int(args[2], 0)
            info = sb.i2c_write(addr, reg, val, bus)
            print("write 0x%02X reg 0x%02X <= 0x%02X   (%s)" % (addr, reg, val, fmt(info)))
            data, info = sb.i2c_read(addr, reg, 1, bus)
            print("read back           -> 0x%02X   (%s)" % (data[0], fmt(info)))
            return

        # default: ID check, then a temperature loop
        data, info = sb.i2c_read(addr, 0x0B, 1, bus)
        print("ADT7420 ID register 0x0B = 0x%02X  (expect 0xCB)   %s" % (data[0], fmt(info)))
        if info["error"]:
            print("The sensor did not answer - check the bus / address ('temperature.py scan' lists every device on both buses).")
            return
        (cfg,), _ = sb.i2c_read(addr, 0x03, 1, bus)
        bits16 = bool(cfg & 0x80)
        print("configuration 0x03 = 0x%02X -> %s-bit mode" % (cfg, "16" if bits16 else "13"))
        print("Temperature (Ctrl-C to stop):")
        try:
            while True:
                data, info = sb.i2c_read(addr, 0x00, 2, bus)
                t, raw = convert(data[0], data[1], bits16)
                print("  %.4f C   raw 0x%04X   %s" % (t, raw, fmt(info)))
                time.sleep(0.5)
        except KeyboardInterrupt:
            print()


if __name__ == "__main__":
    main()
