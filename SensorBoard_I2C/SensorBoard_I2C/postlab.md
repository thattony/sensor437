# Lab 6: Post Lab Questions

# Question 1: The two machines, measured
10 points

You have two implementations of the same protocol:

1. The implementation you wrote state by state in Lab 5.
2. The implementation generated in Lab 6.

<!-- ANSWER START -->
**Designs compared.**
- **Lab 5** – `U:\ece437\Lab5`, normalized build: `ClkDivThreshold = 30` (the only source change vs the original
  `50`, per `git diff`; it feeds only `ClockGenerator1`). Synthesis 2026-10-04 20:59:31, bitstream 21:02:12,
  `Lab5.runs/impl_1/JTAG_Test_File_timing_summary_routed.rpt`: WNS 0.208 ns, WHS 0.035 ns.
- **Lab 6** – `U:\ece437\SensorBoard_I2C\SensorBoard_I2C`, `TICK_DIVIDE = 252`, built 2026-10-04 20:19,
  `build/build.log`: WNS 0.208229 ns, WHS 0.069612 ns.

Labels used below: **CALC** = calculated from the HDL clock chain; **MEAS-HW** = measured on the board by the design
itself (`result2`); **SYN** = synthesis report. No oscilloscope measurement exists for either design.
<!-- ANSWER END -->

## 1a. Comparison table

Include your completed comparison table for both versions.

Compare:

- LUT count
- flip-flop count
- number of states
- state encoding
- measured SCL
- t_LOW
- t_HIGH
- time for a complete read
- behavior on a NACK

<!-- ANSWER START -->
**Answer 1a.**

| Metric | Lab 5 (normalized) | Lab 6 | Source / type |
|---|---|---|---|
| LUT | **45** | **112** | SYN – Lab 5: `U:\ece437\Lab5\lab5_util_hier.rpt`, `(I2C_Test1)` own-logic row; Lab 6: `build/post_synth_utilization_hier.rpt`, `u_i2c` row |
| FF | **21** | **48** | SYN – same rows |
| States | **154** (0–153) | **154** | Lab 5: the implemented state register / case list (the Lab 5 synthesis log has no FSM extraction table); Lab 6: SYN – FSM extraction in `build/build.log` (`Synth 8-802`, 154 rows) |
| Encoding | 8-bit binary, as written (not re-encoded) | **sequential**, 8-bit | Lab 5: source; Lab 6: SYN – `Synth 8-3354 ... encoding 'sequential'` |
| SCL | **100.8 kHz** (CALC) | **100.0 kHz** (CALC) | measured: TODO (scope) |
| t_HIGH | **4.96 µs** (CALC) | **5.00 µs** (CALC) | measured: TODO (scope) |
| t_LOW | **4.96 µs** (CALC) | **5.00 µs** (CALC) | measured: TODO (scope) |
| Duty cycle | **50 %** (CALC) | **50 %** (CALC) | measured: TODO (scope) |
| Complete ID read | **379.4 µs** (CALC: 153 states × 2.48 µs; Lab 5 has no cycle counter) | **382.9–384.3 µs** (MEAS-HW: 38594–38740 cycles ÷ 100.8 MHz from `result2`, `lab6_python.py` runs in `m2_log.md` §4.7) | |
| Behaviour on NACK | Samples `ACK_bit` in states 37, 73, 112 (each overwrites the last); never branches on it, so it runs the whole sequence, NACK + STOP, and returns to state 0; the ACK is visible only on `led[7]`/ILA, nothing reaches the PC. From the HDL – no hardware absent-device test exists (the address is hard-coded) | 0x49 → error = 1, state = 0, busy = 0, result1 = 0xFF; the next 0x48 read returns 0xCB (MEAS-HW, `m2_log.md` §4.7). It also runs the whole sequence after a NACK; `ACK_bit` (OR of all ACKs) drives `error` | |

Timing derivation (CALC):
- Lab 5: 200 MHz → `ILA_Clk` toggles every 4 clocks = 25 MHz (40 ns) → `FSM_Clk` toggles every 31 `ILA_Clk` cycles
  (`ClkDiv == 30`) → period 62 × 40 ns = **2.48 µs per state**. SCL = 4 states (low, high, high, low) = 9.92 µs →
  100.8 kHz; t_HIGH = t_LOW = 2 states = 4.96 µs.
- Lab 6: 100.8 MHz, one `tick` every 252 clocks = **2.50 µs per state**; SCL = 4 ticks = 10.0 µs → 100.0 kHz;
  t_HIGH = t_LOW = 5.00 µs.
- Original Lab 5 (`ClkDivThreshold = 50`): 4.08 µs per state → 61.27 kHz, t_HIGH = t_LOW = 8.16 µs.
  (`Lab5/lab5_scl.txt` lists 30.94 and 61.27 kHz but does not say they were scope measurements; they equal the
  calculation and are not used as measured values.)
- **Normalization:** the state periods differ by 0.8 % (2.48 vs 2.50 µs), duty cycle is 50 % in both. Integer dividers
  cannot make them identical, so this is sufficiently normalized for the transaction-time comparison.

**TODO – physical confirmation:** SCL, t_HIGH, t_LOW and duty cycle on the SCL_0 / SDA_0 through-holes for both
designs have not been measured with the oscilloscope.
<!-- ANSWER END -->

## 1b. Resource use and transaction time

Answer:

- Which implementation used fewer LUTs?
- By what factor?

Compare the transaction times at the same SCL and state what you found.

If the two implementations ran at different:

- frequencies
- duty cycles

give the measured numbers.

State whether each implementation still meets the ADT7420 timing table.

<!-- ANSWER START -->
**Answer 1b.**
- **Fewer LUTs: Lab 5.** Lab 6 uses **112 / 45 = 2.49×** the LUTs and **48 / 21 = 2.29×** the FFs of Lab 5
  (both post-synthesis). Caveat: Lab 6 `u_i2c` includes its internal tick counter and additional
  parameter/status/handshake logic (latched address, register and mode, busy/done/start, per-frame ACK flags, reset,
  recovering default arm), while Lab 5's clock-generation logic is outside the FSM block in `ClockGenerator1`.
  This is the closest available like-for-like comparison, but the two modules do not contain identical supporting
  functionality. (Lab 5's whole-design 831 LUT / 785 FF includes the FrontPanel host and is not used.)
- **Complete read at ~100 kHz:** Lab 5 379.4 µs (CALC) vs Lab 6 382.5 µs (CALC) / 382.9–384.3 µs (MEAS-HW).
  Lab 5 is shorter by ~3–5 µs (~1 %). This is **not a meaningful state-machine difference**: both use exactly
  **153 states** per ID read – Lab 6 spends one more state on the repeated START (4 vs 3) and one fewer on the STOP
  (3 vs 4). The gap is the 0.8 % faster Lab 5 state clock (153 × 0.02 µs = 3.1 µs) plus Lab 6's start latency
  (`result2` counts from the start pulse, and the FSM waits up to one tick for its first tick).
- **ADT7420 Table 2 (p.5)** – both calculated timing sets meet every limit:

| Limit | Lab 5 (CALC) | Lab 6 (CALC) |
|---|---|---|
| f_SCL ≤ 400 kHz | 100.8 kHz | 100.0 kHz |
| t_HIGH ≥ 0.6 µs / t_LOW ≥ 1.3 µs | 4.96 / 4.96 µs | 5.00 / 5.00 µs |
| t_SU;DAT ≥ 0.02 µs, t_HD;DAT ≥ 0.03 µs | 2.48 µs | 2.50 µs |
| t_HD;STA, t_SU;STA, t_SU;STO ≥ 0.6 µs | 2.48 µs | 2.50 µs |
| t_BUF ≥ 1.3 µs | ≥ 4.96 µs | ≥ 5.0 µs |

  Rise/fall time (t_R, t_F ≤ 0.3 µs) depends on the pull-ups and bus capacitance and can only be checked on the scope.
<!-- ANSWER END -->

## 1c. Structural differences

List every place where the two implementations differ.

For each difference, classify it as either:

Protocol disagreement:
One implementation is wrong.

or

Design choice:
Both implementations are valid.

<!-- ANSWER START -->
**Answer 1c.**

| Difference | Lab 5 | Lab 6 | Classification |
|---|---|---|---|
| Line drivers | SCL is a push-pull output (`assign I2C_SCL_0 = SCL`); SDA is driven to 1 for '1' bits and START/STOP/Sr high phases, `1'bz` only in ACK, receive and NACK slots | Open-drain by construction: only pull low or release (`scl_low`/`sda_low` → tristate pads) | **Protocol disagreement (Lab 5).** ADT7420 Table 4 (p.7): SCL and SDA are open-drain with pull-ups. It works on this single-master bus because the slave only drives SDA where Lab 5 releases it, but a driven-high line fights any device holding it low |
| Clocking | FSM on a flop-generated 2.48 µs clock (`FSM_Clk`); `PC_control` from okClk read without a synchronizer | One 100.8 MHz clock + one-clock `tick` enable; endpoints and FSM in one domain | Design choice (Lab 6 avoids the unsynchronized crossing) |
| Start | Level-sensitive: leaves `STATE_INIT` while `PC_control[0] = 1` and restarts while it stays high | Edge-detected start pulse, ignored while busy | Design choice |
| Parameters | Address 0x90/0x91 and register 0x0B hard-coded | Address, bus, register, mode from the PC; read address derived as `{param1[7:1],1}` | Design choice |
| Repeated START / STOP | Sr in 3 states (SDA←1 together with SCL↑ – no edge, the line is already released high; then SDA↓), STOP in 4 | Sr in 4 states (release SDA while SCL low, SCL↑, SDA↓, SCL↓), STOP in 3 | Design choice – both meet t_SU;STA / t_SU;STO; same 153-state total |
| NACK / error reporting | One `ACK_bit`, last ACK wins, LED/ILA only | OR of all ACKs → `error` in the status word, `done`, TriggerOut | Design choice (neither aborts after a NACK; both still finish with NACK + STOP) |
| Recovery | No reset; `default` sets an LED but leaves `State` unchanged | Synchronous reset; `default` releases the bus and returns to idle | Design choice |
| Results / handshake | Data byte on WireOut 0x20 only (0x21 undriven, `Synth 8-3848`); Python waits a fixed 50 ms | busy/done/error/state, `result1` data, `result2` cycle count | Design choice |
| Debug access | ILA core probing `State`, SDA, SCL, `ACK_bit` | `dbg_state` and status over USB; self-checking testbench | Design choice |

Same in both: 4 states per SCL period, SDA changed one state after SCL falls, data/ACK sampled in the second SCL-high
state, MSB first, master NACK on the single data byte (ADT7420 Figure 16, p.19).
<!-- ANSWER END -->

## 1d. Debuggability

Which implementation would you rather be handed at two in the morning when the I2C bus will not acknowledge?

Explain why.

Cost/resource usage is not the only consideration.

<!-- ANSWER START -->
**Answer 1d.** **Lab 6.** When the bus will not acknowledge, the first questions are "which byte was NACKed, is the
FSM stuck, and does any device answer at all?". Lab 6 answers them from Python without reprogramming: every
transaction returns `error`, the ACK bit, `busy`/`done` and `status()['state']`; address, bus and register are
parameters, so a different address or the other bus is one command away (`i2c_first_frame.py` scans both buses in
seconds); a reset trigger and the recovering `default` arm bring the FSM back to idle; and the I2C engine is a clean
module separate from FrontPanel, with a self-checking testbench that decodes every byte and START/STOP before
hardware. Lab 5's ILA is a real strength – it shows the actual SDA level in the 9th clock and the internal state on a
waveform, which can reveal electrical problems no status word shows – but each experiment needs a source edit,
rebuild and a JTAG/ILA session, the PC sees no ACK, error or state, and there is no reset.
<!-- ANSWER END -->

# Question 2: How you broke the job up
10 points

This is the question the lab exists for.

Include:

- your pre-lab plan
- your completed step log

Place them side by side.

<!-- ANSWER START -->
**Pre-lab plan: TODO – requires my original pre-lab plan (pre-lab question 3); it is not stored anywhere under
`U:\ece437`.** Place it beside the step log below.

**Completed step log** (`m2_log.md`):

| Step | Increment | Implementation prompts | Attempts to pass | What was missing from attempt 1 |
|---|---|---|---|---|
| 1 | Finish frame 2: register pointer 0x0B + ACK | 1 | 1 | Nothing – passed on the first attempt |
| 2 | Repeated START + read address 0x91 + ACK | 1 | 1 | Nothing – passed on the first attempt |
| 3 | Receive data byte + master NACK + STOP | 1 | 1 | Nothing – passed on the first attempt |
| 4 | Absent-device handling + final hardware validation | 1 | 1 | Nothing – passed on the first attempt |
<!-- ANSWER END -->

## 2a. Prediction vs actual work

Compare your original prediction with what actually happened.

Answer:

- How many prompts did the plan actually take?
- Was the step you predicted would be hardest actually the hardest?

<!-- ANSWER START -->
**Answer 2a.** The plan took **4 implementation prompts and 4 attempts**; every step passed on its first attempt and
no implementation increment failed. By attempts, no step was harder than another; by novelty, step 3 (receiving
data and generating the master NACK) was the only one with no template in the shipped code.
**TODO – comparison with the predicted prompt count and predicted hardest step requires my original pre-lab plan.**
<!-- ANSWER END -->

## 2b. Step that was too large

Identify which step turned out to be too big.

Explain:

- How did you know it was too big?
- What did the failure look like?
- Could you determine which part of the step had failed?

<!-- ANSWER START -->
**Answer 2b.** The recorded data provides **no evidence that any actual step was too large**: all four steps passed
on the first attempt, so there was no failure to observe and nothing to localize. The broadest step by scope was
step 4 (absent-device simulation, final simulation, build, smoke test, example script, `lab6_python.py` and two
hardware gates in one row), but it passed without a code change.
<!-- ANSWER END -->

## 2c. Split the step

Rewrite the step that was too large as two or more smaller steps.

Each smaller step must have a pass condition that can be stated in one sentence.

<!-- ANSWER START -->
**Answer 2c (hypothetical improvement – no step actually failed).** Step 4 split into steps with one-sentence pass
conditions:
1. Absent device in simulation – frame 3d ends with error = 1, State = 0 and both wires high.
2. Build – `build/build.log` shows XEM7310-A75, WNS > 0, WHS > 0 and EXIT_CODE=0.
3. Milestone 1 compatibility – `smoke_test.py` prints RESULT: ALL PASS on the new bitstream.
4. ID read on hardware – `lab6_python.py --addr 0x48 --reg 0x0B` prints result1[7:0] = 0xCB.
5. Absent device on hardware – `lab6_python.py --addr 0x49` prints error = 1 and state = 0.
<!-- ANSWER END -->

## 2d. What affected attempt count?

Consider whether attempt count tracks how specific the prompt was, or whether it instead tracks something else, such as:

- how unusual the implementation step was
- how much of the datasheet the step covered
- how far the step was from the worked example

Use your own m2_log.md evidence to support your argument.

A step that failed and then worked after being divided into smaller steps is useful evidence.

The prompts that did not work are data.

<!-- ANSWER START -->
**Answer 2d.** Every attempt count is 1, so the dependent variable has **no variation**: the data cannot establish
whether attempt count tracked prompt specificity, task difficulty, novelty, distance from the worked example or the
amount of datasheet behaviour a step covered. What the log does show is that every prompt was highly specific (file
named, exact byte sequence, an explicit "do not implement yet" list, numbered checks) and that steps 1–2 were short
distances from the worked example (copies of the shipped transmit and START states), while step 3 was further away
and still passed – consistent with, but not proof of, "a specific prompt plus an existing pattern to copy is enough".
The problems that did occur fell outside the attempt metric (step 2's temporary STOP ending that only worked because
the MSB of 0xCB is 1; `example_i2c_read.py` printing 0x00 / error=True because its parameter convention differs from
the design), and no step failed and then passed after being split.
<!-- ANSWER END -->

# Question 3: What the checks could not tell you
5 points

The four Milestone 2 gates were:

1. Simulation passes.
2. Build meets timing.
3. Identification register reads correctly.
4. An absent device raises an error.

Consider the following three incorrect implementations.

## Case A

The master ACKs the last byte of a read instead of sending a NACK.

## Case B

The master changes SDA on the same tick that SCL rises.

## Case C

The identification register reads correctly every time, but the I2C timing is:

t_HIGH = 2.5 us
t_LOW  = 7.5 us

## For each case

State which of the four gates would catch the problem.

State which gates would allow the problem to pass.

<!-- ANSWER START -->
**Answer.** Labels: **OBSERVED** (this lab's simulation or hardware output), **DATASHEET** (ADT7420.pdf),
**INFERENCE** (reasoning, not run). What the gates exercise: Gate 1 – the testbench checks bytes, every 9th-clock
ACK/NACK, START/STOP positions, rx_data, State and whole-transaction cycle windows, while the slave model states
"Timing is not checked" (zero-delay wires, samples SDA on SCL rising edges) (OBSERVED); Gate 2 – WNS/WHS cover
FPGA-internal paths at 100.8 MHz, not I2C bus timing (OBSERVED values, INFERENCE on scope); Gate 3 – one read of 0x0B
returns 0xCB (OBSERVED); Gate 4 – 0x49 gives error = 1, state 0, and no slave ever drives SDA (OBSERVED).

| Case | 1 Simulation | 2 Timing closure | 3 ID = 0xCB | 4 Absent device |
|---|---|---|---|---|
| A – master ACKs the last byte | **Catches** it with this testbench: check `3b: master NACKed the data byte (9th clock high)` fails; the slave model would then send mem[0x0C] = 0x00 and hold SDA low, so the single-STOP / SDA-high checks fail as in frame 4 (INFERENCE). A testbench checking only rx_data would miss it | Passes it | Passes it – the byte is complete before the 9th clock (OBSERVED: `BUS: byte 0xcb, 9th clock NACK`, rx_data = 0xCB) | Passes it – no slave acts on the master's ACK |
| B – SDA changes on the same edge SCL rises | **Unreliable** – both lines change in the same zero-delay time step, so what the model samples is a simulator ordering race; no setup check exists (INFERENCE) | Passes it – bus setup/hold is not analysed | May pass – depends on board skew and pull-up rise time (INFERENCE; not tested) | Passes it – no slave to misread |
| C – t_HIGH 2.5 µs, t_LOW 7.5 µs | Passes it – only whole-transaction windows are checked and the period is unchanged; the waveform is in `sim/tb.vcd` but unchecked (OBSERVED testbench content) | Passes it | Passes it (given) | Passes it |

- Case A – DATASHEET: Figure 16 (p.19): "the master generates the no acknowledge at the end of the readback";
  OBSERVED: frame 4 shows the consequence of a slave that keeps driving – "no STOP – the slave holds SDA low"; with the
  NACK, `master NACK, releasing bus` → STOP, and frame 3c (data 0x0C, MSB 0) ends cleanly.
- Case B – DATASHEET: Table 2 requires t_SU;DAT ≥ 0.02 µs, and an SDA edge while SCL is high is a START or STOP.
  OBSERVED (HDL): Lab 6 changes SDA one tick (2.5 µs) after SCL falls; Lab 5's repeated-START state 75 sets SDA←1 in
  the same state SCL rises, harmless only because SDA is already released high.
- Case C – DATASHEET: 2.5 µs ≥ 0.6 µs (t_HIGH) and 7.5 µs ≥ 1.3 µs (t_LOW), so it **meets Table 2** (100 kHz, 25 %
  duty); it is still a different bus ("A version whose t_LOW and t_HIGH differ has changed the bus", milestones.md).
<!-- ANSWER END -->

## Then answer

Of these three incorrect implementations:

- Which problem would you not have discovered without hardware?
- How could you modify the simulation so that it would catch that problem instead?

<!-- ANSWER START -->
**Answer.** **Case B** is the problem that, with the current verification setup, would most likely need hardware
observation. Changing SDA on the same logical edge that SCL rises creates a real setup/hold and ordering problem at
the pins (t_SU;DAT ≈ 0 against the 0.02 µs minimum, plus a risk of a false START/STOP), but the ideal zero-delay
behavioural slave cannot expose it reliably, the timing gate does not analyse the bus, and the ID read may still
succeed. To catch it in simulation, add a protocol/timing assertion that SDA is stable for at least t_SU;DAT before
and through each SCL rising edge (and t_HD;DAT after each falling edge), and flag any SDA change while SCL is high
that is not a deliberate START/STOP.
Case C is not fundamentally invisible to simulation: the current testbench simply does not assert it. t_HIGH and
t_LOW can be read from the simulation waveform or checked automatically by time-stamping SCL edges and asserting
t_HIGH ≥ 0.6 µs, t_LOW ≥ 1.3 µs (and t_HIGH = t_LOW if a 50 % duty cycle is required). Case A is already caught by
the current testbench.
<!-- ANSWER END -->
