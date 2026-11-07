# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# Lab 9 package - orientation for an assistant

This is the ECE 437 **Lab 9 student package**: CMV300 image-sensor control
plane ONLY. The student writes exactly three modules - `hdl/CMV300_Ctrl.v`
(power-up sequencer + FRAME_REQ pulse generator), `hdl/CMV300_Spi.v` (16-bit
SPI master) and `hdl/CMV300_Counters.v` (row/pixel counters + OR/AND pixel
masks, sampled on the falling edge of CLK_OUT - the mid-row DVAL-low INVALID
slot is the trap the lab wants found). Their port lists are FROZEN; the
contract is `INTERFACE.md` (the three port lists, the complete endpoint map,
and every named testbench check - written to be pasted into a prompt).

**There is no capture path in this lab, by design.** No pixel is stored
anywhere: no BRAM frame buffer, no external memory, no pipe endpoints. The
measurement block (`hdl/CMV300_Meters.v`, given and complete) counts LVAL
edges, DVAL cycles, frames, the frame period and the OR/AND of the pixels -
that is the whole instrument. If a task seems to need storing image data,
that is a later lab; suggesting a capture path here is wrong.

## Commands

- `python build.py --sim` - both testbenches in xsim, no synthesis, no
  hardware: `sim/tb_i2c_transmit.v` first, then `sim/tb_lab9.v` (13 scenarios
  against the behavioural sensor model). Each run ends in one
  `RESULT: ALL PASS` / `RESULT: N FAILED` line; every check prints scenario,
  number, time, expected and actual on one line. There is no single-test
  flag - both testbenches always run. Simulate first, and after every change.
- `python build.py --sim --debug` - same, with the debug kernel + waveforms
  (10-30x slower).
- `python build.py` - synthesize `bitfile/SensorBoard_Top.bit` (~15 min;
  driven by `vivado/build.tcl`, log at `build/vivado.log`).
- All of the above need the Vivado bin dir on PATH (`xvlog`/`xelab`/`xsim`/
  `vivado`); `./build.sh` is a two-line wrapper for macOS/Linux convenience.
- Hardware, board attached:
  `python3 -m venv .venv && .venv/bin/pip install -r python/requirements.txt`,
  then `python/smoke_test.py` against the shipped reference bit file FIRST -
  it checks the design ID 0xEC437901 before anything else, and a pass proves
  toolchain/USB/endpoints/sensor, so any later failure is in the student
  modules. Day-to-day CLI:
  `python/cmv300.py [file.bit] test|regs|read N|write N V|clk|geom|period|expsweep|masks|temp`.
- The Python core is stdlib-only (requirements.txt lists optional extras).
  The OpalKelly `ok` module is not on PyPI: `python/fp_api.py` finds it,
  preferring the bundled macOS build in `opalkelly/FrontPanelAPI`
  (override with `OK_API_PATH`).

## Architecture

- `hdl/SensorBoard_Top.v` = FrontPanel endpoints + the I2C machine
  (`hdl/I2C_Transmit.v`, WireIn 0x00-0x03 / WireOut 0x20-0x23) +
  `hdl/CMV300_Subsystem.v`. The I2C endpoints are exactly the earlier lab's,
  but `I2C_Transmit.v` SHIPS AS A STUB with that lab's frozen port list: the
  student pastes in the machine they completed there. Do not write it for
  them - ask for their earlier-lab file; `sim/tb_i2c_transmit.v` and
  `python/temperature.py` define "working". The shipped reference bit file
  has a working I2C inside, so `temperature.py` passes against it as-is.
- `CMV300_Subsystem.v` (given, do not edit) holds the MMCM that makes the
  25 MHz sensor clock from the 200 MHz board oscillator, the ODDR that
  forwards it to CVM300_CLK_IN gated by the student sequencer's clk_en, the
  three student modules and `CMV300_Meters.v`, and wires the camera
  endpoints: WireIn 0x04/0x05, TriggerIn 0x41, WireOut 0x24-0x2C,
  TriggerOut 0x61 (full map in INTERFACE.md).
- Clock domains: okClk = 100.8 MHz (FrontPanel IP - all student control
  logic), the 25 MHz CLK_IN, and the sensor's returned CLK_OUT.
  `CMV300_Counters.v` samples D/LVAL/DVAL on the FALLING edge of CLK_OUT and
  must hand results to okClk at a frame end with a toggle synchroniser so the
  host never reads a torn value.
- `python/sensor_board.py` wraps the FrontPanel endpoints;
  `python/cmv300.py` subclasses it with the camera endpoints.
- `sim/cmv300_model.v` is the behavioural sensor: it enforces the power-up
  sequence and reports violations (the testbench provokes one on purpose);
  scenario 10 is the invalid-slot pixel-count trap. `sim/xsim.dir` is
  scratch, deleted and recreated on every sim run.

## Package layout notes

- `datasheets/` holds the documents the module headers cite (CMV300.pdf,
  ADT7420.pdf, the I2C sensor datasheets, the sensor-board schematic).
  Answer register/timing questions from these, not from memory.
- If an `instructor/` directory is present, this is the instructor's master
  copy: it holds solution files and MUST be removed before the package is
  distributed to students. Never copy anything from `instructor/` into
  `hdl/` unless the instructor asks.

## Sensor facts and traps

- The sensor returns CLK_OUT only when reg 69 = 9, and at ~36.7 MHz unless
  reg 83 = 251 (the 25 MHz PLL band). CLK_OUT meter (WireOut 0x27):
  25,000 +/- 16 per ms.
- One frame: 488 rows of 648 valid pixels (324 + one DVAL-low slot + 324);
  frame period 12.88 ms +/- 1% at default exposure; period =
  max(readout, exposure) + FOT.
- Ctrl sequencing: clock BEFORE reset release, clock still running WHILE
  resetting, each step >= 1 us; SPI traffic only after sensor_ready.
  FRAME_REQ is ONE pulse of >= 4 CLK_IN periods (>= 160 ns) per trigger -
  `frame_req_in` may arrive as a held level, so edge-detect it or the sensor
  free-runs (scenario 12 checks exactly this). Same for t_exp1.
- SPI: MSB first - control bit, 7 address bits, 8 data bits; the sensor
  samples SPI_IN on rising SPI_CLK edges and launches SPI_OUT on falling
  edges. Keep the shipped parameters: T_WAIT/T_REQ (Ctrl), HALF = 5 (Spi,
  10.08 MHz), GAP = 64 (Counters).
