# Lab 9 - CMV300 control plane

Bring the CMOSIS CMV300 image sensor up and measure everything it does - with
counters, not pictures. This lab has **no capture path**: nothing here stores a
pixel. You generate the sensor's clock, sequence its power-up, talk to it over
SPI, trigger frames, and read what the measurement block counted.

## The three files you write

| File | What it must do |
|---|---|
| `hdl/CMV300_Ctrl.v` | power-up sequencer (CLK_IN running -> >= 1 us -> release SYS_RES_N -> >= 1 us -> ready), reset sequence, and a FRAME_REQ pulse generator (one trigger -> one pulse of >= 4 CLK_IN periods; never pass a level through) |
| `hdl/CMV300_Spi.v` | 16-bit SPI master, MSB first, <= 10 MHz: control bit, 7 address bits, 8 data bits; the sensor samples SPI_IN on the rising edge and launches SPI_OUT on the falling edge; reads, writes and bursts |
| `hdl/CMV300_Counters.v` | everything that watches the pixel bus, sampled on the FALLING edge of CLK_OUT: LVAL edge counter (rows), DVAL-high cycles per row (pixels), and the OR / AND of every pixel of the last frame |

All three ship as stubs with **frozen port lists** (see `INTERFACE.md` - the
wrapper, the testbench and the Python depend on them). A fourth file is also
a stub, but not new work: `hdl/I2C_Transmit.v` has the same frozen port list
as the I2C lab - **paste in the machine you completed there**, and
`sim/tb_i2c_transmit.v` (and `temperature.py` on hardware) must pass again.
Everything else is given and complete: `CMV300_Meters.v` (CLK_OUT meter + the
three frame-timing intervals), `CMV300_Subsystem.v` (wrapper, MMCM, ODDR,
endpoints), the top level, the behavioural sensor model and the testbench.
The shipped reference bit file contains a working I2C machine, so
`temperature.py` runs against it out of the box.

## Simulate (do this first, and after every change)

    python build.py --sim     # no synthesis; log output ends in RESULT: ...

Runs the I2C testbench, then `sim/tb_lab9.v`: 13 scenarios against the sensor
model, ending in one `RESULT: ALL PASS` or `RESULT: N FAILED` line. Every check
prints its scenario, number, time, expected and actual value on one line - that
line is what you paste when asking for help. With the stubs as shipped the
testbench FAILS with named checks; that is the starting state. (On macOS/Linux,
`./build.sh` is a convenience wrapper around the same `build.py`.)

## Build a bit file

    python build.py           # ~15 min; bitfile/SensorBoard_Top.bit

## Check your setup in the first twenty minutes

A reference bit file is shipped in `bitfile/`. With the board attached:

    python3 -m venv .venv && .venv/bin/pip install -r python/requirements.txt
    .venv/bin/python python/smoke_test.py     # -> RESULT: ALL PASS

A pass proves the toolchain, USB, endpoints and sensor - so any later failure
is in your modules, not your setup. `python/cmv300.py` is the day-to-day CLI
(`test`, `regs`, `read/write`, `clk`, `geom`, `period`, `expsweep`, `masks`,
`temp`); `python/temperature.py` still talks to the I2C sensors.

Later labs add the capture path (pixels into memory and up to the PC); nothing
in this package prepares for that, on purpose.
