# SensorBoard_Starter — write an FPGA sensor interface from its datasheet, with an LLM

A minimal, working FPGA project for the **Opal Kelly XEM7310-A75** on the **UIUC ECE 437
Sensor Board**. It contains everything needed to build a bitstream, load it over USB and
exchange data with a PC — and exactly **one frame of I2C**: `hdl/I2C_Transmit.v` sends
START, one address byte, reads the ACK and sends STOP. That is the exercise: pick a sensor on
the board, find the timing diagram / protocol description in its datasheet, and grow that
framework into a state machine that implements the full transaction on the real hardware,
using Claude Code (or any coding assistant) as your collaborator.

The board carries six sensors and two I2C buses (see [`docs/SENSOR_BOARD.md`](docs/SENSOR_BOARD.md)):

| Sensor | Measures | Interface |
|---|---|---|
| ADT7420 | temperature | I2C bus 0, `0x48` |
| HTS221 | humidity + temperature | I2C bus 0, `0x5F` |
| LPS35HW | pressure + temperature | I2C bus 0, `0x5C` |
| LSM303DLHC | acceleration + magnetic field | I2C bus 1, `0x19` / `0x1E` |
| AD7156 | capacitance (touch pads) | I2C bus 1, `0x48` |
| CMV300 (labelled `CVM300` on the board) | 648×488 image | SPI config + 10-bit parallel video |

## 0. Which file do I edit?

```
hdl/
├── SensorBoard_Top.v   TOP LEVEL - FrontPanel IP, every sensor pin + its safe default, I2C pad
│                       drivers, bus select, result/status words.   You do not edit this.
└── I2C_Transmit.v      THE I2C STATE MACHINE - START, address byte, ACK, STOP.   THIS IS YOURS.
```

That is the whole FPGA design: two files. Everything else is tooling — `constraints/` (pinout),
`vivado/build.tcl` + `build.sh` (build), `sim/` (testbench + behavioural I2C slave),
`python/` (talk to the board), `docs/` + `datasheet/` (the sensors), `opalkelly/` (vendor IP).

## 1. Quick start (before touching any HDL)

```bash
python3 -m venv .venv && .venv/bin/pip install -r python/requirements.txt   # once; no PyPI deps
./build.sh --detach          # ~5 min native Vivado, 15–20 min in the Docker container
./build.sh --status          # repeat until STATUS: FINISHED OK
.venv/bin/python python/smoke_test.py      # board on USB -> "RESULT: ALL PASS"
```
```PowerShell
# Windows PowerShell version for this setup:
# Run from: U:\ece437\SensorBoard_I2C\SensorBoard_I2C

# Once: create Python 3.10 environment and install requirements
py -3.10 -m venv .venv310
.\.venv310\Scripts\pip.exe install -r .\python\requirements.txt

# Make Vivado and Opal Kelly SDK available in this PowerShell session
$env:Path = "C:\Xilinx\Vivado\2022.2\bin;$env:Path"
$env:OK_API_PATH = "C:\Program Files\Opal Kelly\FrontPanelUSB\API\Python\x64"

# Build FPGA bitstream in background
& "C:\Program Files\Git\bin\bash.exe" ./build.sh --detach

# Check build status; repeat until STATUS: FINISHED OK
& "C:\Program Files\Git\bin\bash.exe" ./build.sh --status

# Run smoke test with board connected by USB
.\.venv310\Scripts\python.exe -c "import os,sys,runpy; os.add_dll_directory(r'C:\Program Files\Opal Kelly\FrontPanelUSB\API\lib\x64'); sys.path.insert(0,r'.\python'); runpy.run_path(r'.\python\smoke_test.py', run_name='__main__')"
```

`smoke_test.py` exercises the framework as shipped: it sends the ADT7420's address on bus 0
(must ACK), an empty address (must NACK → `error`), the LSM303's address on bus 1 (must ACK),
and checks the handshake/trigger/pipe endpoints and the LEDs. When it passes you know the
toolchain, the USB link, the endpoint map and both I2C buses are fine — from now on any
problem is in *your* state machine. `python/i2c_first_frame.py` goes one step further and
scans both buses: all six I2C parts must show up. (No bitstream is shipped — your first build
produces `bitfile/SensorBoard_Top.bit`, which both scripts load by default.)

Windows/Linux: install the Opal Kelly FrontPanel SDK and `export OK_API_PATH=<SDK>/API/Python/<ver>/x64`.
Apple Silicon: keep this folder inside the `vivado-on-silicon-mac` repository so `build.sh`
can use the Docker container.

## 2. How the pieces fit

```
PC (python/sensor_board.py)  <-- USB / FrontPanel -->  hdl/SensorBoard_Top.v            <-->  hdl/I2C_Transmit.v  <-->  sensors
   run(param1..3)                WireIn 0x00-0x03            TOP: FrontPanel IP, every sensor     THE FILE YOU EDIT:
   results(), status()           WireOut 0x20-0x23, 0x3F     pin + safe default, I2C pads,        START, byte, ACK, STOP
   read_pipe()                   Trigger 0x40/0x60, Pipe 0xA0  bus select, results  (not edited)
```

**Two Verilog files, that is all.** `SensorBoard_Top.v` is the top level: it owns the
FrontPanel IP, every pin of every sensor with the level it must idle at (documented in a
table in its header), the open-drain I2C pad drivers, the bus selector and the result/status
words. It instantiates `I2C_Transmit` once (`u_i2c`) and hands it a 100.8 MHz clock, a `start`
pulse, the byte to send, the spare parameter words and the selected bus's SCL/SDA. The I2C
lines are open-drain (`*_low = 1` pulls the line down, `0` releases it) — you never drive a
line high. `sim/` and `python/` are helpers, not part of the FPGA design.

`I2C_Transmit` is the classic ECE 437 state machine: one state per quarter of an SCL period,
advanced by a 2.5 µs tick (so SCL = 100 kHz), states 1–2 START, 3–34 the eight bits, 35–38
the ACK clock, 39–41 STOP. As shipped, `param1[7:0]` is the byte to send (`0x90` = ADT7420
write address), `param1[8]` selects the bus, `result0[0]` is the ACK bit (0 = a slave
answered) and `result2` the number of clock cycles the frame took. It stops there on purpose:
no register pointer, no data bytes, no repeated START. Every figure you need to add them is
cited in its header.

## 3. Working with the assistant — the point of this project

The assistant can read the datasheets in `datasheet/` (PDF, by page range), the schematic,
and `docs/SENSOR_BOARD.md`; it can run the simulation and the build, and (with the board on
USB) run Python against the hardware. `CLAUDE.md` tells it the board facts and the workflow,
so **your prompt only needs to say what to implement and where the spec is**. Precision in
the prompt is what you are practising. Compare:

> *weak:* "write an I2C driver for the humidity sensor"

> *good:* "Extend `hdl/I2C_Transmit.v` (instantiated by `hdl/SensorBoard_Top.v`, bus 0) so that it
> reads the HTS221 WHO_AM_I register (0x0F) using the single-byte read sequence of
> **Table 13, page 16 of `datasheet/HTS221.pdf`** (address patterns in Table 10, page 15),
> keeping SCL = 100 kHz and the timing limits of **Table 6, page 10**. `param1[7:0]` =
> register address, `param1[15:8]` = 7-bit slave address; put the byte in `result0[7:0]`,
> set `error` on a NACK, update `python/i2c_first_frame.py` to match. Simulate with
> `./build.sh --sim` first, then build and read the register from Python — it must return
> 0xBC."

Then iterate in the same style:

- "Extend it to N-byte reads with auto-increment (sub-address bit 7, **section 5.1.1, page 15**;
  sequence in **Table 14, page 16**) and read HUMIDITY_OUT_L/H (0x28/0x29); convert using the
  calibration registers of **Table 19 and the procedure in section 8 (pages 26–27)**."
- "Add a register write path (**Table 11, page 16**) so I can set CTRL_REG1 (0x20) to 0x81 —
  PD = 1, ODR = 1 Hz, per **Table 17, page 22**."
- "Read the LSM303 magnetometer: registers 0x03–0x08 come out in the order X, Z, Y
  (**Table 17, page 23**). Return the three axes in `result0..result2` as sign-extended 16-bit values."
- "Configure the CMV300 over SPI following the write timing of **Figure 7, page 15** (control
  bit, 7 address bits, 8 data bits, sampled on the rising edge of SPI_CLK), read a register back
  per **Figure 9, page 16**, then request one frame (**section 3.10**) and stream the parallel
  output (**section 4.2, page 22**) into the PipeOut."
- "Run the whole verification list in `CLAUDE.md` and tell me what was verified on hardware."

Things a good prompt pins down: which file, which bus and address, which figure/table/page,
the clock rate and timing limits, how parameters and results map onto `param*`/`result*`,
what "done" looks like on the board (the ID register value), and that simulation comes before
hardware.

## 4. Suggested tasks (increasing difficulty)

0. Scan both buses with the framework as shipped (`python/i2c_first_frame.py`) and explain,
   from the simulation log, why a stand-alone frame must carry the *write* address.
1. Read a device ID register over I2C (ADT7420 `0x0B` → `0xCB` is the easiest — verified on this hardware):
   add the register-pointer byte (Figure 14), then the repeated START, read address, data byte
   and master NACK (Figure 16).
2. Multi-byte read + data conversion (ADT7420 temperature; HTS221 with calibration; LPS35HW 24-bit pressure).
3. Register write + read-back (configure a sample rate; power up the LSM303 accelerometer).
4. Two devices on the same bus in one transaction sequence; a device on bus 1.
5. Continuous acquisition: data-ready pin (`hts221_drdy`, `lsm303_drdy`) → FIFO → PipeOut streaming to the PC.
6. SPI: configure the CVM300 and capture an image (parallel video into a frame buffer → PipeOut).
7. Optimisation: measure transaction time (`result2` as a cycle counter), LUT/FF count
   (`build/post_synth_utilization_hier.rpt`, row `u_i2c`), SCL up to the datasheet's 400 kHz,
   robustness against bad parameters and NACKs.

## 5. Tips that save an afternoon

- **Change SDA only while SCL is low, at least one tick after it fell; sample while SCL is high.** Edges
  on both lines in the same clock look like START/STOP conditions to the slave.
- **NACK the last byte you read**, otherwise the slave keeps driving SDA into your STOP.
- ST parts (HTS221, LPS35HW, LSM303) need **bit 7 of the sub-address set** to auto-increment.
- Keep the FSM on the 100.8 MHz `clk` and use the one-clock `tick` enable for bit timing — no ripple clocks.
- Put the state register on `dbg_state`: `sb.status()["state"]` tells you where it is stuck.
- Scope the bus on TP7–TP10 when simulation and hardware disagree.
- Leave every sensor you don't use in its safe default (already in `SensorBoard_Top.v`) — a
  mis-configured HTS221/LPS35HW in SPI mode will fight you on bus 0.
- **Never end a frame right after a read address.** The slave starts driving data bit 7 as
  soon as it has ACKed; if that bit is 0 your STOP cannot happen and the bus is stuck until
  you clock the byte out. The shipped testbench shows this (frame 4).

## 6. Files

See the table in [`CLAUDE.md`](CLAUDE.md). Short version: extend `hdl/I2C_Transmit.v` (the
param/result map is in the header of `hdl/SensorBoard_Top.v`), write your Python on top of
`python/sensor_board.py` (start from `python/i2c_first_frame.py`), simulate with `sim/`,
build with `build.sh`, read `docs/SENSOR_BOARD.md` and `datasheet/`.
