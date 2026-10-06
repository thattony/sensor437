# Lab 6 Milestone 2 – Step log

Filled in during the session, one entry per step; every simulation run after a code change is one attempt.
Previous attempt records are never overwritten — new attempts are appended under their step.

| Step (from plan) | Attempts to pass | What was missing from attempt 1 |
|---|---|---|
| 1. Finish frame 2, the ADT7420 register pointer (0x90, ACK, 0x0B, ACK) | 1 | Nothing — passed on the first attempt. |
| 2. Repeated START + read address (… 0x0B, ACK, Sr, 0x91, ACK) | 1 | Nothing — passed on the first attempt. |
| 3. Receive the data byte + master NACK + STOP (… 0x91, ACK, data, NACK, P) | 1 | Nothing — passed on the first attempt. |
| 4. Validate absent-device handling and complete final hardware validation | 1 | Nothing — passed on the first attempt. |

---

## Step 1 – Finish frame 2, the register-pointer frame

**Goal:** S → 0x90 → ACK → 0x0B → ACK (then the existing STOP), same quarter-bit state style as frame 1.
No repeated START, no read address, no data byte, no master NACK yet.

**Changes**
- `hdl/I2C_Transmit.v`: new states 42–73 (pointer byte, MSB first, 4 states/bit) and 74–77 (release SDA,
  sample the pointer ACK in state 76), then back to the existing STOP states 39–41. State 38 branches:
  `RegMode ? 42 : 39`. `param3[0]` (latched as `RegMode`) enables frame 2, `param2[7:0]` (latched as
  `RegPointer`) is the pointer byte. With `param3 = 0` the shipped single frame is unchanged
  (smoke_test.py writes param2 = param3 = 0). New `ACK_ptr` = ACK of frame 2; `ACK_bit` = OR of both ACKs.
- `sim/tb_i2c_transmit.v`: bus monitor that decodes each 9-bit slot straight off SDA/SCL, START counter,
  frame 3b (pointer 0x0B, full checks) and frame 3c (pointer 0x00, restores the slave pointer so frame 4's
  demonstration is unchanged). Frames 1–4 untouched.
- `hdl/SensorBoard_Top.v`: not changed.

### Attempt 1 — 2026-10-04 18:13 — PASS
`./build.sh --sim` → `RESULT: ALL PASS` (0 failures). Evidence from `sim/sim.log`:
```
407528000  BUS: byte 0x90, 9th clock ACK
492523000  ADT7420: register pointer <= 0x0b
497523000  BUS: byte 0x0b, 9th clock ACK
510022000  ADT7420: STOP
510042000  TB: done. byte 0x90 -> ACK_bit=0 error_bit=0  (19652 cycles = 194 us)
TB: PASS 3b: bus byte 2 = 0x0B, ACKed
TB: PASS 3b: FSM sampled pointer ACK (ACK_ptr = 0), ACK_bit = 0
TB: PASS 3b: exactly 1 START and 1 STOP
```
Frames 1–4 (Milestone 1 behaviour) give the same results and cycle counts as before (10578–10580 cycles).
What was missing from attempt 1: nothing — passed on the first attempt.

---

## Step 2 – Repeated START + read address 0x91

**Goal:** after the pointer ACK: Sr → 0x91 → ACK, with no STOP between 0x0B and 0x91.
No data byte, no master NACK, no final STOP sequence, no 0x49 error-path fix yet.

**Changes**
- `hdl/I2C_Transmit.v`: state 77 now goes to 78 instead of the STOP. New states 78–81 = repeated START
  (78: SCL low, release SDA; 79: SCL high, SDA high; 80: SDA falls while SCL high = Sr; 81: SCL low),
  82–113 = read address `ReadAddr = {tx_byte[7:1], 1}` (0x90 → 0x91), MSB first, 4 states/bit,
  114–117 = release SDA, sample the ACK in state 116 (new `ACK_rd`, also OR-ed into `ACK_bit`).
  State 117 jumps to the existing STOP states 39–41 as a **temporary** end of the transaction
  (data byte + master NACK are step 3). The step-1 "pointer write, then STOP" ending is replaced
  (param3[0] = 1 now always continues to the repeated START). param3[0] = 0 path untouched.
- `sim/tb_i2c_transmit.v`: bus monitor now also prints START / repeated START / STOP with the number of
  bytes already on the bus; frame 3b checks 3 bytes, Sr after byte 2, the only STOP after byte 3, slave
  rw = 1, ACK_rd = 0. Old frame 3c (pointer 0x00 frame) removed — with the read path it would read
  mem[0x00] = 0x0C and hang the bus; the slave pointer is reset to 0x00 directly in the testbench
  (test-only) so frame 4 is unchanged.
- `hdl/SensorBoard_Top.v`: not changed.

### Attempt 1 — 2026-10-04 — PASS
`./build.sh --sim` → `RESULT: ALL PASS` (0 failures, 22 PASS lines). Evidence from `sim/sim.log`:
```
320034000  BUS: START (after 0 bytes)
402529000  ADT7420: address 0x48 matched, WRITE
407528000  BUS: byte 0x90, 9th clock ACK
492523000  ADT7420: register pointer <= 0x0b
497523000  BUS: byte 0x0b, 9th clock ACK
510022000  ADT7420: START
510022000  BUS: repeated START (after 2 bytes)
592517000  ADT7420: address 0x48 matched,  READ
597516000  BUS: byte 0x91, 9th clock ACK
602516000  ADT7420: sending mem[0x0b] = 0xcb
610016000  ADT7420: STOP
610016000  BUS: STOP (after 3 bytes)
610035000  TB: done. byte 0x90 -> ACK_bit=0 error_bit=0  (29732 cycles = 294 us)
TB: PASS 3b: 2 STARTs: START before byte 1, repeated START after byte 2
TB: PASS 3b: no STOP between 0x0B and 0x91 (only STOP after byte 3)
TB: PASS 3b: bus byte 3 = 0x91, ACKed
TB: PASS 3b: slave took it as a READ (rw = 1)
TB: PASS 3b: FSM sampled read-address ACK (ACK_rd = 0), ACK_bit = 0
```
Milestone 1 frames (param3 = 0) unchanged: frames 1–3 each START → 1 byte → STOP, 10578–10580 cycles;
frame 4 still shows "no STOP – slave holds SDA low" with mem[0x00] = 0x0c.
Note: the temporary STOP only works here because data bit 7 of 0xCB is 1 (slave leaves SDA released);
with a data byte whose MSB is 0 it would hang like frame 4 — step 3 (read byte + master NACK) fixes that.
What was missing from attempt 1: nothing — passed on the first attempt.

---

## Step 3 – Receive the data byte, master NACK, STOP

**Goal:** after the read-address ACK: receive 8 bits MSB first with SDA released, master NACK in the 9th
clock, STOP, back to state 0; byte in `result1[7:0]`. Expected for register 0x0B: 0xCB.
0x49 error path not touched.

**Changes**
- `hdl/I2C_Transmit.v`: state 117 now goes to 118 (the temporary jump to STOP from step 2 is gone).
  New states 118–149 = data byte, 4 states per bit: (a) SCL low, SDA released (slave changes its bit),
  (b) SCL high, (c) SCL high + `RxByte[n] <= sda_in` (same sample point as the ACK states 37/76/116),
  (d) SCL low. New states 150–153 = master NACK (SDA released during the 9th clock), then state 153 →
  existing STOP states 39–41 → state 0. New `reg [7:0] RxByte`, cleared on start;
  `rx_data = {24'd0, RxByte}` → top level `result1` (top level unchanged).
- `sim/tb_i2c_transmit.v`: frame 3b now checks the full read (4 bytes, data 0xCB, master NACK, slave
  released SDA, rx_data = 0xCB, single STOP after byte 4, State 0). Frame 3c re-added as a real read of
  register 0x00 (0x0C, MSB = 0 – the case that hangs in frame 4) – this replaces the step-2 test-only
  pointer reset. Frame 4 got one extra check: rx_data = 0 on the param3[0] = 0 path.
- `hdl/SensorBoard_Top.v`: not changed.

### Attempt 1 — 2026-10-04 18:42 — PASS
`./build.sh --sim` → `RESULT: ALL PASS` (0 failures, 27 PASS lines). Evidence from `sim/sim.log`:
```
320034000  BUS: START (after 0 bytes)
407528000  BUS: byte 0x90, 9th clock ACK
492523000  ADT7420: register pointer <= 0x0b
497523000  BUS: byte 0x0b, 9th clock ACK
510022000  BUS: repeated START (after 2 bytes)
592517000  ADT7420: address 0x48 matched,  READ
597516000  BUS: byte 0x91, 9th clock ACK
602516000  ADT7420: sending mem[0x0b] = 0xcb
687511000  BUS: byte 0xcb, 9th clock NACK
692510000  ADT7420: master NACK, releasing bus
700010000  ADT7420: STOP
700010000  BUS: STOP (after 4 bytes)
700030000  TB: done. byte 0x90 -> ACK_bit=0 error_bit=0  (38804 cycles = 384 us)
TB: PASS 3b: data byte on the bus = 0xCB
TB: PASS 3b: master NACKed the data byte (9th clock high)
TB: PASS 3b: rx_data[7:0] (-> result1[7:0]) = 0xCB
TB: PASS  released SDA after the NACK (not transmitting, not pulling SDA)
TB: PASS 3b: exactly 1 STOP, after byte 4 (none between 0x0B and 0x91)
TB: PASS it = 0, busy = 0, State back to 0, lines released, SDA wire high
TB: PASS 3b: time 380..395 us (153 ticks x 2.5 us)
--- frame 3c (register 0x00, MSB = 0) ---
1072486000  BUS: byte 0x0c, 9th clock NACK
1077486000  ADT7420: master NACK, releasing bus
1084985000  BUS: STOP (after 4 bytes)
TB: PASS 3c: rx_data = 0x0C, data byte NACKed, 1 STOP after byte 4
TB: PASS : State back to 0, SDA wire high (no hang), slave pointer = 0x00
```
Milestone 1 frames (param3 = 0) unchanged: frames 1–3 10578/10580/10580 cycles, same ACK/NACK results;
frame 4 still "no STOP – slave holds SDA low"; new check rx_data = 0 on that path passes.
Observation (testbench only, not an FSM problem): `check()` holds a 64-character name, so names longer
than 64 characters lose their first characters in the log (e.g. "TB: PASS : State back to 0 …" is the
3c check, "TB: PASS it = 0, …" is the 3b error_bit check). The checks themselves are evaluated correctly.
Simulation wall time grew to ~10 min (elapsed 00:10:14) with the two 384 µs read frames.
What was missing from attempt 1: nothing — passed on the first attempt.

---

## Step 4 – Validate absent-device handling and complete final hardware validation

**Goal:** Gate 4 (0x49 → error = 1, state 0) in simulation, then final simulation, bitstream build
(Gate 2), smoke test, example script (diagnostic only), `python/lab6_python.py`, and the real-board
0x48 / reg 0x0B → 0xCB (Gate 3) and 0x49 → error = 1, state 0 (Gate 4) checks.
Attempt count increases only if a failure needs a code change.

### Attempt 1

**4.1 Absent device 0x49 in simulation — PASS, HDL unchanged.**
Testbench only: new frame 3d = complete-read mode (param3[0] = 1, param2 = 0x0B) with tx_byte 0x92
(address 0x49). "error" in the testbench = the top level's status bit, `error_bit | (done & ACK_bit)`.
No HDL change: the FSM does not abort after the first NACK – it runs the whole sequence (every byte
NACKed, data reads 0xFF from the released bus), master NACK, STOP, state 0. That satisfies the gate
cleanly, so it was left as is.
```
1177479000  BUS: byte 0x92, 9th clock NACK
1267473000  BUS: byte 0x0b, 9th clock NACK
1279973000  BUS: repeated START (after 2 bytes)
1367467000  BUS: byte 0x93, 9th clock NACK
1457461000  BUS: byte 0xff, 9th clock NACK
1469960000  BUS: STOP (after 4 bytes)
1469980000  TB: done. byte 0x92 -> ACK_bit=1 error_bit=0  (38804 cycles = 384 us)
TB: PASS 3d: first byte 0x92 NACKed on the bus
TB: PASS 3d: top-level error = 1 (ACK_bit = 1), error_bit = 0
TB: PASS 3d: done = 1, busy = 0, State = 0
TB: PASS 3d: SCL/SDA released, both wires high, STOP seen
```

**4.2 Final complete simulation — PASS.** `./build.sh --sim` → `TB: finished, 0 failure(s) -> RESULT: ALL PASS`
(31 PASS lines). Normal read still correct (frame 3b): `BUS: byte 0xcb, 9th clock NACK`,
`TB: PASS 3b: rx_data[7:0] (-> result1[7:0]) = 0xCB`, `TB: PASS 3b: master NACKed the data byte`,
`TB: PASS 3b: exactly 1 STOP, after byte 4`, State back to 0. Milestone 1 frames unchanged
(10578 / 10580 / 10580 / 10580 cycles).

**4.3 Bitstream build (Gate 2) — PASS.** `./build.sh --detach`, polled `./build.sh --status` →
`STATUS: FINISHED OK -> bitfile/SensorBoard_Top.bit` (Vivado 2022.2, finished 2026-10-04 20:19:32).
From `build/build.log`:
```
  IP BOARD = XEM7310-A75
  Post-route timing: WNS = 0.208229 ns, WHS = 0.069612 ns
write_bitstream completed successfully
impl_1: write_bitstream Complete! (100%)
BUILD COMPLETE
IP board  : XEM7310-A75
EXIT_CODE=0
```
Only critical warnings: the documented harmless `[Common 17-55] set_property expects at least one object`
from the vendor XDC. (`build/post_synth_utilization_hier.rpt` row `u_i2c` now reads 112 LUT / 48 FF – recorded
here only as a build fact; the Lab 5 vs Lab 6 comparison is Milestone 3.)
The Milestone 1 bitstream was copied to `bitfile/SensorBoard_Top_M1_starter.bit` before the rebuild;
every hardware test below used the NEW `bitfile/SensorBoard_Top.bit` (20:19).

**4.4 smoke_test.py on the real board, new bitstream — PASS (12/12).**
```
Board: XEM7310-A75  serial: 24090019Y6
Bit file: U:\ece437\SensorBoard_I2C\SensorBoard_I2C\bitfile\SensorBoard_Top.bit
  [PASS] design ID 0xEC437001 (expected 0xEC437001)
  [PASS] idle after reset {... 'state': 0 ...}
  [PASS] ADT7420 0x48 ACKs on bus 0 (result0[0] == 0) result0=0x00009000
  [PASS] error = 0 after ACK
  [PASS] byte/bus echoed in result0 0x00009000
  [PASS] transaction time ~103 us (41 ticks x 2.5 us) 10447 cycles = 103.6 us
  [PASS] done set, busy clear, state back to 0
  [PASS] TriggerOut 0x60 fired
  [PASS] empty address 0x49 NACKs (result0[0] == 1) result0=0x00009201
  [PASS] error = 1 after NACK {... 'error': True, 'state': 0 ...}
  [PASS] LSM303 accel 0x19 ACKs on bus 1 result0=0x00013200
  [PASS] PipeOut 0xA0 counter first words ['0x1', '0x2', '0x3', '0x4']
RESULT: ALL PASS
```
The param3[0] = 0 (Milestone 1) path is unchanged on hardware with the Milestone 2 bitstream.

**4.5 example_i2c_read.py, run unchanged (diagnostic only, not a gate) — runs, but its convention does
not match this design.**
```
Board XEM7310-A75, design ID 0xEC437001
ADT7420 ID  : 0x00  (expect 0xCB)  error=True  10569 cycles
Temperature : 0.0000 C  (raw 0x0001, 10362 cycles = 102.8 us)   (x5, 10362..10557 cycles)
```
Why: the script packs everything into param1 (`[31:24]` addr+W, `[23:16]` register, `[15:8]` addr+R,
`[7]` write flag, `[6:0]` byte count) and reads the data from `result0[31:24]`. This design uses
`param1[7:0]` = addr+W byte, `param1[8]` = bus, `param2[7:0]` = register, `param3[0]` = 1 for a read,
and returns the data in `result1[7:0]`. For the ID read the script's word is 0x900B9101, so the FSM sees
`param1[7:0]` = 0x01 as the address byte, `param1[8]` = 1 (bit 0 of 0x91) → bus 1, and param3 = 0 →
the single-frame mode: one ~104 µs frame to address byte 0x01 on bus 1, NACK → error = True.
`result0[31:24]` is always 0 in this top level → "0x00". In the temperature word (0x90009102)
`r0 >> 16` = `result0[16]` = the bus bit = 1 → "raw 0x0001". The FSM was not changed for this script.

**4.6 python/lab6_python.py created** against the implemented interface: param1 = (bus << 8) | (addr7 << 1),
param2 = register, param3 = 1; prints result1[7:0], error, state, busy/done, ACK_bit, cycles/µs.
Default run = 0x48/0x0B (expect 0xCB, error 0, state 0), 0x49 (expect error 1, state 0), then
0x48/0x0B again (bus recovered after the NACK). `--addr`, `--reg`, `--bus` select a single read.
On Windows it adds the Opal Kelly DLL directory itself (no wrapper needed).

**4.7 lab6_python.py on the real board, new bitstream — Gate 3 and Gate 4 PASS.**
```
Test 1: valid device - ADT7420 0x48, ID register 0x0B (expect 0xCB, error 0, state 0)
    result1[7:0] = 0xCB
    error        = 0
    state        = 0
    busy / done  = 0 / 1   ACK_bit (result0[0]) = 0
    time         = 38609 cycles = 383.0 us
    -> PASS
Test 2: absent device - address 0x49 (expect error 1, state 0)
    result1[7:0] = 0xFF
    error        = 1
    state        = 0
    busy / done  = 0 / 1   ACK_bit (result0[0]) = 1
    time         = 38594 cycles = 382.9 us
    -> PASS
Test 3: valid device again after the NACK (bus recovered?)
    result1[7:0] = 0xCB   error = 0   state = 0   38651 cycles = 383.4 us
    -> PASS
RESULT: ALL PASS
```
Single-address mode (as the TA would rerun it):
`lab6_python.py --addr 0x49` → result1 0xFF, error 1, state 0, busy 0, 38740 cycles;
`lab6_python.py --addr 0x48 --reg 0x0B` → result1 0xCB, error 0, state 0, 38636 cycles.
Hardware read time 38594–38740 cycles (≈383–384 µs) vs 38804 in simulation; the spread is under one tick
(252 cycles) – the tick counter free-runs and is only reset by rst, so the wait for the first tick varies.

**Milestone 2 gates – all verified:**
1. `./build.sh --sim` → RESULT: ALL PASS (0 failures) – simulation.
2. `build/build.log` → XEM7310-A75, WNS = +0.208229 ns, WHS = +0.069612 ns, EXIT_CODE=0.
3. Real hardware: 0x48 / reg 0x0B → result1[7:0] = 0xCB.
4. Real hardware: 0x49 → error = 1, state = 0.
5. Real hardware: `smoke_test.py` → RESULT: ALL PASS (12/12).

What was missing from attempt 1: nothing — passed on the first attempt (no code change was needed;
the only edits in this step were testbench frame 3d and the new `python/lab6_python.py`).

---

# Milestone 3 Comparison

| Metric | Lab 5 | Lab 6 | Source / Notes |
|---|---|---|---|
| LUT | 45 | 112 | Lab 5: lab5_util_hier.rpt, (I2C_Test1) own-logic row; Lab 6: build/post_synth_utilization_hier.rpt, u_i2c |
| FF | 21 | 48 | Same reports |
| States | 154 | 154 | Lab 5: states 0–153 from source; Lab 6: FSM extraction in build/build.log |
| Encoding | 8-bit binary | sequential, 8-bit | Lab 5 source; Lab 6 synthesis log |
| SCL | 100.8 kHz calculated | 100.0 kHz calculated | Lab 5 normalized with ClkDivThreshold = 30 |
| t_HIGH | 4.96 us calculated | 5.00 us calculated | Physical scope measurement still pending |
| t_LOW | 4.96 us calculated | 5.00 us calculated | Physical scope measurement still pending |
| Duty cycle | 50% calculated | 50% calculated | Physical scope measurement still pending |
| Complete read time | 379.4 us calculated | 382.9–384.3 us measured on hardware | Lab 5: 153 states x 2.48 us; Lab 6: result2 |
| NACK behavior | Continues transaction and returns to state 0; NACK not reported to PC | error = 1, state = 0; next valid read succeeds | Lab 6 verified on hardware |

- Lab 6 uses 112/45 = 2.49x the LUTs of Lab 5.
- Lab 6 uses 48/21 = 2.29x the FFs of Lab 5.
- Lab 5 and Lab 6 are sufficiently normalized for timing comparison: 100.8 kHz vs 100.0 kHz, both 50% duty cycle.
- Lab 5 is about 3–5 us faster after normalization, mainly because its state period is 2.48 us instead of 2.50 us.
- Both calculated timing sets satisfy the ADT7420 timing requirements.
- Remaining Milestone 3 physical measurements: SCL, t_HIGH, t_LOW, and duty cycle on the board for both designs.
