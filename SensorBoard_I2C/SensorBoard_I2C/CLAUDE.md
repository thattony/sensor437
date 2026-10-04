# SensorBoard_Starter — instructions for Claude Code

Bare-bones, self-contained FPGA project for the **OpalKelly XEM7310-A75** (Artix-7
`xc7a75tfgg484-1`) plugged into the **UIUC ECE 437 Sensor Board**. It builds, loads and
talks to the board out of the box, and contains exactly **one I2C frame** of sensor logic:
`hdl/I2C_Transmit.v` sends START + one address byte, reads the ACK and sends STOP
(`hdl/SensorBoard_Top.v` instantiates it and maps it onto the PC endpoints). The student's job — with
your help — is to grow that framework into the complete read/write transactions of a
protocol/timing diagram from one of the sensor datasheets in `datasheet/`, build it, and
verify it on the real board.

Nothing outside this folder is required except Vivado (natively, or the
`vivado-on-silicon-mac` Docker container on Apple Silicon — this folder must then live inside
that repository tree) and, for hardware runs, the board on this computer's USB.

## Layout

| Path | What |
|---|---|
| `hdl/I2C_Transmit.v` | **The framework to extend.** Module `I2C_Transmit`: one state per quarter-bit (states 0–41), advanced by a `tick`; sends START, one byte (`tx_byte`, MSB first), samples the ACK in the 9th clock, sends STOP; `busy/done/ACK_bit/error_bit/State` handshake; open-drain `scl_low/sda_low` + `sda_in`. No register pointer, no data bytes, no repeated START yet — the header lists the datasheet figures to implement next. |
| `hdl/SensorBoard_Top.v` | **Top level (the only other Verilog file).** FrontPanel IP, 100.8 MHz `okClk`, start/reset conditioning, a header table of **every sensor** (bus, address, ID register, side-band pins), the safe default of every sensor pin, open-drain I2C pad drivers, bus selector (`param1[8]`), result/status words, cycle counter into `result2`, LEDs, pipe counter. Instantiates `I2C_Transmit #(.TICK_DIVIDE(252)) u_i2c`. Touch it only to change a safe default (e.g. take the CMV300 out of reset), add ports to `u_i2c`, or change the endpoint map. |
| `constraints/xem7310_v1.xdc` | Full XEM7310 pinout (unmodified vendor/UIUC file). Ports not in the design → harmless `[Common 17-55] set_property expects at least one object` warnings. |
| `constraints/sensor_board.xdc` | Sensor-board side-band pins the vendor file leaves commented out + button pull-ups. |
| `vivado/build.tcl` | Batch flow: project → **every** `hdl/*.v` + `constraints/*.xdc` → FrontPanel IP (endpoints configured here) → synth → impl → `bitfile/SensorBoard_Top.bit`. Adding a module = adding a file to `hdl/`. |
| `build.sh` | Finds Vivado (native or Docker), runs the build (`--detach` / `--status`) or the simulation (`--sim`). |
| `sim/tb_i2c_transmit.v`, `sim/i2c_slave_model.v` | Testbench of `I2C_Transmit` alone (the top level is not simulated) + behavioural I2C register-file slave (ADT7420 at 0x48 and LSM303 accel at 0x19 on one simulated bus, registers preloaded). `sim/tb_i2c_slave_selftest.v` drives the slave with a bit-banged master — a passing reference for the exact I2C event order (run it as documented in its header). |
| `python/sensor_board.py` | `SensorBoard` class: load bitfile, `run(param1..3)`, `status()`, `results()`, triggers, `read_pipe()`. All scripts accept an optional `.bit` path as first argument. |
| `python/smoke_test.py` | End-to-end check of the unmodified starter (design ID, ADT7420 ACK on bus 0, NACK on an empty address, LSM303 ACK on bus 1, handshake, transaction time, trigger, pipe, buttons, LEDs). Needs the sensor board attached. |
| `python/i2c_first_frame.py` | Drives the framework as shipped: one frame to any address on either bus, or a scan of both buses listing every device that ACKs (all six I2C parts should appear). |
| `python/example_i2c_read.py` | Template for the script of a *finished* FSM (ADT7420 ID + temperature with the ECE 437 transaction-word convention). The shipped framework does not understand that word yet — adapt it to the FSM you build. |
| `python/fp_api.py` | Locates/loads the FrontPanel `ok` module (bundled macOS build in `opalkelly/FrontPanelAPI`; Windows/Linux: install the SDK, set `OK_API_PATH`). |
| `opalkelly/ip_repos/…` | FrontPanel Subsystem Vivado IP v1.0.6 (required for synthesis, unmodified). |
| `docs/SENSOR_BOARD.md` | **Hardware reference**: every sensor, bus, I2C address, side-band pin, safe default, register cheat-sheet, test points, endpoint map. Read it before writing any FSM. |
| `datasheet/` | Schematic + layout (source of truth for wiring) and the datasheets of all six sensors + the FrontPanel manual (`datasheet/README.md` lists them). |
| `bitfile/SensorBoard_Top.bit` | Produced by the build (not shipped — students build it). The Python scripts load it by default. |

## Board facts you need constantly (details in `docs/SENSOR_BOARD.md`)

- FSM clock = `okClk` = **100.8 MHz** (from the FrontPanel IP). One clock domain for FSM and
  PC endpoints → no synchronisers needed. `clk200` (board oscillator) is also available but
  crossing between it and `clk` needs synchronisation.
- **I2C bus 0** (`I2C_SCL_0` H3 / `I2C_SDA_0` G3): ADT7420 `0x48`, HTS221 `0x5F`, LPS35HW `0x5C`.
  **I2C bus 1** (`I2C_SCL_1` D2 / `I2C_SDA_1` E2): LSM303DLHC accel `0x19` + mag `0x1E`, AD7156 `0x48`.
  10 kΩ pull-ups on the board; all four parts allow SCL ≤ 400 kHz.
- I2C in `I2C_Transmit` is **open-drain by construction**: `sda_low = 1` pulls SDA to 0,
  `= 0` releases it; read the level on `i2c0_sda_in`. Same for SCL (so clock stretching can be
  detected) and for bus 1. Never "drive high".
- Side-band pins with a **required safe default** (already assigned in the placeholder — keep
  them unless the design uses them): `hts221_spi_en = 1`, `lps35_cs = 1` (both chips stay in I2C
  mode and quiet), `lps35_sdo = 0` (address 0x5C), `adt7420_a0 = adt7420_a1 = 0` (address 0x48),
  CVM300 held in reset with no clock.
- Imager: CMOSIS **CMV300** (spelled `CVM300` in the schematic/XDC/HDL). 4-wire SPI
  (`cvm_spi_en/clk/mosi/miso`; 16-bit frames: control bit 1=write/0=read, 7 address bits,
  8 data bits, MSB first, sampled on the rising edge of SPI_CLK — `CMV300.pdf` §3.9, p.15–16),
  FPGA supplies `cvm_clk_in` (10–40 MHz per §3.6; the pin table says 25 MHz max), sensor returns
  10-bit pixels on `cvm_d` with `cvm_clk_out`/`cvm_line_valid`/`cvm_data_valid` (§4.2, p.22).
- Buttons: pressed = 0. Sensor-board LEDs `s_LED`: 1 = on. XEM LEDs: the `led` port of the top
  is 1 = on (the top level inverts for the active-low pins).
- ID checks that prove a bus works (all confirmed against the datasheets in `datasheet/`):
  ADT7420 reg `0x0B` = `0xCB`, HTS221 reg `0x0F` = `0xBC`, LPS35HW reg `0x0F` = `0xB1`,
  LSM303 mag regs `0x0A..0x0C` = `0x48 0x34 0x33`, AD7156 configuration reg `0x0F` power-on
  default = `0x19` (its chip-ID register 0x17 is a factory value the datasheet does not publish).
  A missing device reads `0xFF` on a released bus.

## PC ↔ FPGA contract (keep unless you change top level, `build.tcl` and Python together)

| Endpoint | Meaning |
|---|---|
| WireIn `0x00` | bit 0 start (rising edge → one `start` pulse), bit 1 reset (level), `[31:8]` → `ctrl` for free use |
| WireIn `0x01..0x03` | `param1..param3` — **the FSM defines their meaning** (document it in the FSM header and in the Python script). As shipped: `param1[7:0]` = address byte `{addr7, R/W}`, `param1[8]` = bus (0/1); `result0[0]` = ACK bit (0 = ACK), `result0[15:8]` = byte sent, `result0[16]` = bus, `result2` = clock cycles busy. |
| WireOut `0x20..0x22` | `result0..result2` |
| WireOut `0x23` | status: bit 0 `busy`, bit 1 `done`, bit 2 `error`, `[15:8]` `dbg_state`, `[31:16]` `status_user` |
| WireOut `0x3F` | design ID `0xEC437001` (change it in the top level when you change the endpoint map) |
| TriggerIn `0x40` | bit 0 start, bit 1 reset (pulses) |
| TriggerOut `0x60` | bit 0 fires on the rising edge of `done` |
| PipeOut `0xA0` | `po_data` is sampled whenever `po_read` is high — feed it from a FIFO or counter |

FSM handshake the Python relies on: `busy` = 1 while working, `done` = 1 when results are
valid (stays 1 until the next start/reset), `error` = 1 for a failed transaction (e.g. NACK).
`dbg_state` should follow the state register so a stuck FSM can be diagnosed from Python.

## Task: "implement the FSM for <sensor> / <timing diagram>"

This is the main use of the package. The starting point is `hdl/I2C_Transmit.v`: it already
does frame 1 of every I2C transaction (START, address byte, ACK, STOP) in the one-state-per-
quarter-bit style (4 states = 1 SCL period, one `tick` = 2.5 µs per state), so a student's
first prompts are typically "add the register-pointer byte (frame 2 of Figure 14)", "add the
repeated START + read address + data byte with the master NACK (Figure 16)", "make it a 2-byte
read (Figure 17)". Keep the `busy/done/ACK_bit/State` handshake and the open-drain convention
while extending it; you may restructure freely (bit counters + shift registers instead of
one state per bit, parameters for byte count, …) as long as the top level's param/result map
and the Python are updated together. A stand-alone frame must use the **write** address
(`0x90`): after ACKing a read address the slave drives data bit 7 and, if it is 0, holds SDA
low so the STOP never happens (the simulation shows this).

Expected workflow:

1. **Read the hardware reference** (`docs/SENSOR_BOARD.md`) and the datasheet pages the user
   points at (`Read` the PDF with a page range). Extract: bus/pins, slave address, the exact
   sequence (START, address+R/W, ACK slots, repeated START, NACK on the last byte, STOP; or
   SPI CS/clock polarity/phase, bit order, word format), and the timing limits
   (`f_SCL` max, `t_LOW`/`t_HIGH`/setup/hold, register conversion times, power-up delays).
2. **Derive the timing** from 100.8 MHz: e.g. 100 kHz SCL with 4 phases per bit → a tick every
   252 clocks (`TICK_DIVIDE = 252`, the tick generator at the top of `I2C_Transmit.v`). Change SDA only while SCL is low and at least one
   tick after it fell; sample SDA while SCL is high. State the numbers in comments.
3. **Extend `hdl/I2C_Transmit.v`** (and, if it needs new ports, the `u_i2c` instance and the
   result words in `hdl/SensorBoard_Top.v`): keep the safe defaults for unused sensors; use `param*` for
   addresses/registers/byte counts, `result*` for data, `error` for NACK, `dbg_state` = state.
   Prefer bit/byte counters + shift registers over one state per bit once the frame count
   grows. Keep the reset and the `default:` arm that returns to idle.
4. **Simulate first**: extend `sim/tb_i2c_transmit.v` (preload the slave's registers, set the
   parameter words, add a slave at the right address if needed), then `./build.sh --sim` and
   read `sim/sim.log` — the slave prints START/STOP/address/byte events, so a protocol error is
   visible without a scope; the testbench prints `TB: PASS/FAIL` lines and ends with
   `RESULT: ALL PASS`. `sim/tb.vcd` has the waveforms. The shipped testbench sends four frames
   (ADT7420 ACK, empty address NACK, LSM303 ACK on bus 1, and the read-address demonstration
   that ends without a STOP).
5. **Build**: `./build.sh --detach`, poll `./build.sh --status` every ~60 s until
   `STATUS: FINISHED OK` (5 min natively, 15–20 min in Docker). Success criteria in
   `build/build.log`: `IP BOARD = XEM7310-A75`, `write_bitstream Complete`, `BUILD COMPLETE`,
   `EXIT_CODE=0`, `Post-route timing: WNS = +x, WHS = +x`. Resource use of the FSM alone:
   `build/post_synth_utilization_hier.rpt` (row `u_i2c`). After a Docker build:
   `docker kill vivado_container`.
6. **Verify on hardware**: `python3 -m venv .venv && .venv/bin/pip install -r python/requirements.txt`
   (once), then write/adapt a script on top of `python/sensor_board.py` (start from
   `python/example_i2c_read.py`): read the ID register first, then real data, convert with the
   datasheet formula, print. Check `error`, check a wrong address returns `0xFF`/error rather
   than hanging, check the FSM recovers from bad parameters (e.g. byte count 0). Report what the
   board actually returned.
7. Compare against the requirement: transaction time (use a cycle counter in `result2`),
   LUT/FF count, robustness. Say explicitly what was verified on hardware versus only simulated.

## Task: "build" / "smoke test" / "run"

- Build: see step 5. Only `./build.sh --detach` + `--status`; do not run Vivado in a foreground
  tool call (too long).
- Smoke test of an **unmodified** starter (sensor board attached): `.venv/bin/python python/smoke_test.py`
  → `RESULT: ALL PASS`. If it passes, the toolchain, USB link, endpoint map and both I2C buses
  are fine and any later problem is in the FSM. `.venv/bin/python python/i2c_first_frame.py`
  scans both buses and must list all six I2C parts.
- Board must be connected **directly to this computer's USB** (FrontPanel never goes through Docker).
  Only one program can hold the board: `OpenBySerial failed: -1` means another script (or the
  FrontPanel app) has it open.

## Troubleshooting

| Symptom | Meaning |
|---|---|
| `FrontPanel not enabled` / `IsFrontPanelEnabled() False` | IP built for the wrong BOARD (must be `XEM7310-A75`, not `XEM7310MT-A75`). `build.tcl` enforces it; rebuild. |
| design ID ≠ `0xEC437001` | wrong/old bit file loaded, or endpoint map changed without updating the ID. |
| `busy` stays 1, `done` never | FSM stuck — read `dbg_state` from `status()`, find the state, check for a missing transition/`default:`; use TriggerIn reset to recover. |
| all bytes `0x00` | FSM never drove/sampled the bus, or a line is held low (sensor board not powered/seated). Scope TP7–TP10. |
| all bytes `0xFF` | bus idle, nobody answered: wrong address, SDA never released for the slave, wrong bus. Confirm with the ID register of a known-good part (ADT7420 `0xCB`). |
| ID correct but data wrong | byte order / auto-increment (ST parts need sub-address bit 7 for multi-byte reads), or the master ACKed the last byte instead of NACKing so the slave keeps driving SDA. |
| ADT7420 ACKs once, then every frame NACKs / SDA stuck low | a frame ended after a **read** address: the slave is still driving data. Clock out 9 SCL pulses with SDA released (or power-cycle the sensor board), and finish read transactions with data + master NACK + STOP. |
| timing violation in `post_route_timing.rpt` | logic on `clk200`, or combinational paths through the pads; keep the FSM on `clk` (okClk). |

## Rules that keep the build working (tool/board facts, not design choices)

- `CONFIG.BOARD {XEM7310-A75}` on `frontpanel_0` is mandatory. Without it the bitstream loads
  but FrontPanel never enumerates.
- Use the FrontPanel **IP core** (bundled). The legacy `okLibrary.v` files will not synthesise
  in current Vivado.
- Part `xc7a75tfgg484-1`; do not change it.
- If you add/rename FrontPanel endpoints: change `build.tcl` (`CONFIG.WI/WO/TI/TO/PO.*`), the
  IP instance port names in `SensorBoard_Top.v` (`wiXX_ep_dataout`, `woXX_ep_datain`,
  `tiXX_ep_trigger`, `toXX_ep_trigger`, `poXX_ep_read/datain`, all lowercase hex), and
  `python/sensor_board.py` together — and bump the design ID.
- Vivado 2024.1 verified; 2023.1+ should work. No board files or licence beyond a standard install.
