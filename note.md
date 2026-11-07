# ECE 437 — Verilog / FPGA Notes

A running collection of reusable patterns, pitfalls and tool fixes for
synthesizable Verilog and the FPGA lab workflow. It grows as the labs go on:
each section is written to be general, so it applies to any state machine,
sequencer, protocol master (SPI / I2C / UART), or controller, not just the lab
it came from. Lab-specific code appears only as examples.

**Adding to this file:** keep points clear, concise, easy to understand, and
applicable to other scenarios. Give each new section the next number so it can
be referenced.

---

## 1. Registers vs wires: `reg` + `always` vs `wire` + `assign`

Two different kinds of hardware:

| | `reg` in `always @(posedge clk)` | `wire` with `assign` |
|---|---|---|
| Hardware | flip-flop (memory) | logic gates (a permanent connection) |
| Changes | only on a clock edge | immediately, whenever its inputs change |
| Holds a value? | yes, until assigned again | no, it is always "the formula" |
| Assigned with | `<=` inside the block | `assign x = ...;` outside any block |

```verilog
// register: remembers, updates on the clock edge
always @(posedge clk) clk_en <= 1'b1;

// wire: a comparator hard-wired to st, always correct
assign sensor_ready = (st == S_READY);
```

**Rules:**
- A `wire` can only be driven by `assign` (or a module output). Writing
  `my_wire <= ...` inside `always` is a **compile error**.
- A signal may be driven from **only one place**. Never assign the same signal
  from two `always` blocks, or from an `always` and an `assign`.
- Port declarations decide this: `output reg x` → set it in an `always` block;
  `output wire x` → drive it with `assign`. If a port list is frozen (given by
  the lab), respect what it declares.

**Which one to pick:**
- The signal is **purely a function of the current state/inputs** ("1 whenever
  we are in state X") → `assign`. It can never get out of sync, and has no
  one-cycle lag.
- The signal must **remember** something or change at a **specific moment** (a
  counter, a pulse lasting N cycles, a captured value) → `reg` in a clocked block.

---

## 2. Finite state machines (FSMs)

### When you need one
If an output must take **different values at different times under the same
inputs**, you need state. Example: with `cam_enable = 1` held constant,
`sys_res_n` must be 0 for the first 20 µs and then 1. A plain `assign` from the
inputs cannot tell those moments apart; a state register can.

### Skeleton

```verilog
// 1. Name the states (localparam = named constants, readable + debuggable)
localparam [3:0] S_OFF      = 4'd0,
                 S_CLK_ON   = 4'd1,
                 S_RES_WAIT = 4'd2,
                 S_READY    = 4'd3,
                 S_RES_LOW  = 4'd4;

// 2. The state register (and any counters)
reg [3:0]  st  = S_OFF;      // initial value = power-on state
reg [11:0] cnt = 12'd0;

// 3. One clocked block: transitions + registered outputs
always @(posedge clk) begin
    case (st)
        S_OFF: begin
            // outputs for this state
            // if (condition) st <= NEXT;
        end
        // ... one branch per state ...
        default: st <= S_OFF;   // illegal state -> recover safely
    endcase
end

// 4. Outputs that depend only on the state
assign ready = (st == S_READY);
assign state = st;             // expose the state for debugging (WireOut, ILA)
```

### Naming the state register
If a port called `state` already exists (e.g. a debug output), name your
internal register something else (`st`, `cur_state`, `fsm`) and connect it with
`assign state = st;`. You cannot turn a frozen `output wire` into a `reg`.

### Always include `default:`
If the register ever holds an unused value (glitch, bad init, 4 bits holding
only 5 states), `default: st <= S_OFF;` makes the machine recover instead of
locking up.

### Expose the state
Wiring the state register to a WireOut / LED / ILA probe lets you see **where**
a stuck machine sits. "Stuck at state 2" immediately tells you which branch to
read.

---

## 3. Two kinds of states: timed vs wait-for-event

Classify every state before writing it.

| Kind | Has a counter? | Leaves when | Examples |
|---|---|---|---|
| **Timed** | yes | the counter reaches N (or an early-abort condition) | "wait ≥ 1 µs", "hold reset low 20 µs", "SCL high for half a bit" |
| **Wait-for-event** | **no** | an input/condition becomes true | IDLE (wait for `start`), READY (stay until told to stop) |

**Common mistake:** copy-pasting the counter into a wait-for-event state. A
READY state with a timer will fall out of READY every N cycles and come back,
so the "ready" flag blinks and in-flight operations get cut off. READY / IDLE
states sit there indefinitely.

Writing a summary table like this before coding catches most bugs:

| State | Kind | Leaves when → next |
|---|---|---|
| S_OFF | event | `want_up` → CLK_ON |
| S_CLK_ON | timed | done → RES_WAIT; `!want_up` → RES_LOW |
| S_RES_WAIT | timed | done → READY; `!want_up \|\| reset` → RES_LOW |
| S_READY | event | `!want_up \|\| reset` → RES_LOW |
| S_RES_LOW | timed | done → CLK_ON if `want_up`, else OFF |

---

## 4. How to "wait" in hardware: count clock cycles

There is no `delay()` in synthesizable Verilog. `#100` delays are
**simulation-only**: synthesis ignores them. To wait, stay in a state and count
clock edges.

**Compute the count:** `N = time × f_clk`

| Clock | Period | 1 µs | 20 µs | 1 ms |
|---|---|---|---|---|
| 100.8 MHz (okClk) | 9.92 ns | ~101 | 2016 | 100,800 |
| 25 MHz | 40 ns | 25 | 500 | 25,000 |
| 200 MHz | 5 ns | 200 | 4000 | 200,000 |

**Counter width:** need `2^bits > N`. 2016 → 12 bits (4096). 100,800 → 17 bits.
Too narrow and the counter wraps before reaching N, so the state never exits.

**The pattern (if / else-if / else, all at one level):**

```verilog
S_WAIT_X: begin
    // outputs for this state
    if (abort_condition) begin      // 1. leave early
        cnt <= 0;
        st  <= S_ABORT;
    end else if (cnt == N-1) begin  // 2. done waiting
        cnt <= 0;                   //    clear for the NEXT state's wait
        st  <= S_NEXT;
    end else                        // 3. keep waiting
        cnt <= cnt + 1;
end
```

- Compare against `N-1`: counting 0 … N-1 is N cycles.
- **Always clear the counter when changing state**, so the next timed state
  starts from 0. Forgetting this makes the next wait short or skipped.
- Safety margin: if the datasheet says "≥ 1 µs", waiting 20 µs costs nothing and
  survives clock inaccuracy. Use parameters (`parameter T_WAIT = 2016`) so the
  testbench or another lab can change it.

---

## 5. Set every output in every state

In a clocked block a register **keeps its value** if you don't assign it, so
leaving an output unassigned in a state is legal. Set it anyway:

1. **`initial` runs once**, at FPGA configuration only. On the second trip around
   the machine, values come from wherever you just were, not from `initial`.
2. **A state can be entered from several places.** If you only set a signal
   where it "changes", every arriving path must leave exactly the right value.
   Miss one and you get a bug that appears only on a particular sequence
   (e.g. only after a reset).
3. **It reads as a table.** Each state is one row: easy to compare against the
   datasheet, easy for a TA to check.

| State | clk_en | sys_res_n |
|---|---|---|
| OFF | 0 | 0 |
| CLK_ON | 1 | 0 |
| RES_WAIT | 1 | 1 |
| READY | 1 | 1 |
| RES_LOW | 1 | 0 |

Alternative that avoids the problem entirely: make the outputs `assign`s from the
state (`assign clk_en = (st != S_OFF);`) when the port list allows `wire`.

---

## 6. One `always` block or two?

**Style 1: single clocked block (recommended for lab-size FSMs)**
Transitions, counters and registered outputs all in `always @(posedge clk)`.
Simple-state outputs as `assign`. Less code, no latch risk.

**Style 2: two-process FSM**
`always @(posedge clk) st <= next_st;` plus `always @*` that computes `next_st`
and outputs combinationally. More code, and:

> **Latch trap:** in `always @*`, every signal must be assigned in **every**
> branch. Missing one makes synthesis infer a latch to "remember" the old value:
> a real hardware bug. Put default assignments at the top of the block.

```verilog
always @* begin
    next_st = st;      // defaults first: no latches
    out_a   = 1'b0;
    case (st) ... endcase
end
```

**Timing note:** in Style 1, outputs set inside the block appear **one clock
after** the state changes. Usually harmless (one 10 ns cycle against µs-scale
waits). If an output must change in the same cycle as the state, use an
`assign` from `st` instead.

---

## 7. Blocking `=` vs non-blocking `<=`

| Where | Use |
|---|---|
| `always @(posedge clk)` (sequential) | `<=` always |
| `always @*` (combinational) | `=` always |

- Never mix `=` and `<=` in the same block.
- With `<=`, every right-hand side reads the **old** value (from before the clock
  edge). `a <= b; b <= a;` swaps them. That is what flip-flops do.
- With `=` in a clocked block, order matters and simulation can disagree with
  hardware. Avoid it.

---

## 8. Edge detection: turn a level into a one-cycle event

When an input can arrive as a held **level** but you must act **once**, compare
with last cycle's value:

```verilog
reg in_prev = 1'b0;
always @(posedge clk) in_prev <= in;

wire in_rise = in  & ~in_prev;   // 1 for exactly ONE cycle on 0 -> 1
wire in_fall = ~in &  in_prev;   // 1 for exactly ONE cycle on 1 -> 0
```

Use for: start buttons, trigger inputs, a host WireIn bit used as "go", a
request that may be held. Passing a held level straight through to something
that expects a pulse makes it repeat forever (e.g. a camera free-running, a
transaction re-firing).

If the input comes from **another clock domain** (or a physical pin/button),
synchronize it first with two flip-flops, then edge-detect:

```verilog
reg [2:0] s = 3'b000;
always @(posedge clk) s <= {s[1:0], async_in};
wire rise = s[1] & ~s[2];
```

---

## 9. Pulse generator: one event → a pulse exactly N cycles wide

```verilog
reg [7:0] p_cnt = 0;
reg       pulse = 0;

always @(posedge clk) begin
    in_prev <= in;
    if (!pulse) begin
        if (in_rise && enabled) begin   // start only when allowed
            pulse <= 1'b1;
            p_cnt <= 0;
        end
    end else if (p_cnt == N-1) begin    // end after N cycles
        pulse <= 1'b0;
    end else
        p_cnt <= p_cnt + 1;
end
```

- Width requirement in another clock? Convert it: "≥ 4 periods of 25 MHz" =
  160 ns → at 100.8 MHz that is ≥ 17 cycles; choose 32 for margin.
- Decide what a new trigger during an active pulse should do (ignore is usual).

---

## 10. Shutdown / reset sequencing (generic hardware lesson)

Many chips (image sensors, ADCs, codecs, radios) reset **synchronously to their
own clock**. So:

1. Assert reset **while the clock is still running**.
2. Hold it for at least the datasheet minimum.
3. **Only then** stop the clock.

Stopping the clock together with reset can mean the reset never takes effect.
Power-up is the mirror image: clock first → wait → release reset → wait → start
talking to the chip.

**"Reset and come back" vs "turn off":** you often don't need to remember *why*
you entered the reset state. At the end of the wait, check the enable condition
again: still enabled → go back up; disabled → off.

Combine "should be on" conditions into one signal so every state uses the same
test:

```verilog
wire want_up = enable & clk_locked;   // any of these false -> go down
```

---

## 11. Verilog syntax traps seen in practice

- **`begin` without `end`:**
  ```verilog
  if (x) begin st <= A;   // ✗ begin opened...
  else st <= B;           //   ...never closed -> syntax error
  ```
  A single statement doesn't need `begin`/`end`. Every `begin` needs exactly one
  `end`. Indent consistently so mismatches stand out.
- **Copy-paste next states:** after copying a state branch, re-check the
  `st <= ...` target. `st <= S_RES_WAIT` inside `S_RES_WAIT` = stuck forever.
- **Typos in state names** (`S_RES_WS_OFF`) are compile errors, which is good.
  That's another reason to use `localparam` names instead of raw numbers.
- **Skipping a state:** going `S_OFF → S_RES_WAIT` instead of
  `S_OFF → S_CLK_ON` silently breaks a datasheet ordering requirement.
  Draw the diagram first.
- **Copy-pasting a condition but forgetting a term:** e.g. `if (!want_up)` where
  it should be `if (!want_up || cam_reset)`. A request arriving in that state is
  then ignored.

---

## 12. Debugging workflow

1. **Simulate first, always.** No board needed, and failures point at the exact
   cycle.
2. Read the **first** failure; later ones often cascade from it.
3. A failing check that reports "actual = state number" tells you where the
   machine stopped. The fault is in that state's exit condition.
4. Fix one thing, re-run, compare.
5. Only synthesize (slow) when simulation passes.

---

## 13. Tool setup (Windows / campus machines)

- **Vivado on PATH** (current PowerShell only):
  ```powershell
  $env:PATH = "C:\Xilinx\Vivado\2022.2\bin;$env:PATH"
  ```
  Permanent (user PATH; reopen the terminal afterwards):
  ```powershell
  [Environment]::SetEnvironmentVariable("Path", [Environment]::GetEnvironmentVariable("Path","User") + ";C:\Xilinx\Vivado\2022.2\bin", "User")
  ```
  Check with `where.exe xsim`.
- **Network drive (U:) + Vivado:** `cmd.exe` (which runs Vivado's `.bat`
  wrappers) can't use a `\\server\share` UNC path as its working directory.
  It falls back to `C:\Windows` and fails with "Failed to open handle
  xvlog.log". In Python scripts use `Path(...).absolute()` instead of
  `.resolve()`. `resolve()` expands a mapped drive into its UNC path.
- **Long synthesis runs:** copy the project to a local disk (`C:\Users\...`)
  first. Vivado on a network drive is slow.
- **Opal Kelly `ok` module on Windows:** Python files are in
  `C:\Program Files\Opal Kelly\FrontPanelUSB\API\Python\x64`, and the DLL is in
  `...\API\lib\x64`. Add the first to `sys.path` and the second with
  `os.add_dll_directory()` before `import ok`.

---

## 14. Clock-domain crossing (CDC)

A signal **crosses clock domains** when it's produced on one clock and read on
another. If the two clocks are unrelated (different sources, no fixed phase),
the receiver can sample the signal at any moment relative to its changes. It
may miss it, see it a cycle late, or go **metastable** (an in-between value
that settles randomly).

**Spot the crossings:** for every signal, ask "which clock writes it, which
clock reads it?" Same clock → no problem. Different clocks → pick a technique
below.

| Crossing | Technique |
|---|---|
| Slow single bit (enable, reset, mode) | **2-flip-flop synchronizer** at the receiver |
| Short pulse, fast → slow clock | **stretch** it to several receiver periods |
| Pulse that must arrive exactly once | **toggle synchronizer**: flip a bit per event, sync it, edge-detect at the receiver |
| Multi-bit value (counter, data word) | **toggle/handshake**: hold the value stable, send a synced "ready" bit; or an **async FIFO** for streams |

### 14.1 2-flip-flop synchronizer (single bit)

```verilog
reg [1:0] s = 2'b00;
always @(posedge clk_dst) s <= {s[0], sig_from_other_domain};
wire sig_safe = s[1];            // use only this in the clk_dst domain
```

- Lives in the **receiving** domain. The first flop may go metastable; the
  second gives it a full cycle to settle.
- Adds 2 receiver cycles of delay.
- Only for **single bits** that stay stable for several receiver cycles.

### 14.2 Pulse stretching (fast → slow)

A 1-cycle pulse on a fast clock can fall entirely between two edges of a slow
clock, and the receiver never sees it. Hold it long enough:

```
N_cycles ≥ (required receiver periods × receiver period) / sender period
          → round UP, then add margin (≈ 2×)
```

Example: a chip needs ≥ 4 periods of its 25 MHz clock (4 × 40 ns = 160 ns).
Driven from 100.8 MHz (9.92 ns): 160 / 9.92 = 16.1 → min 17 cycles → use 32.

Also respect the **maximum**: a pulse held too long can look like a level
(e.g. "keep requesting"). Stay inside the window.

**Driving an external chip's pin is a crossing too.** Its datasheet "minimum
pulse width" / "setup and hold" numbers exist because the chip samples the pin
on its own clock. Convert them to your clock and add margin.

### 14.3 Multi-bit values: why not just 2 flops per bit?

Each bit's synchronizer can resolve in a different cycle, so the receiver can
read a **torn** value: some bits new, some old (e.g. 0x0FF → 0x100 read as
0x1FF). Instead:

1. In the source domain, copy the value into a **holding register** that stays
   stable, then **flip a toggle bit**.
2. Sync the toggle bit into the destination domain (14.1) and edge-detect it
   (§8). Any change of the toggle = "new value ready".
3. On that edge, the destination copies the holding register. It has been
   stable for several cycles, so every bit is correct.

Works when updates are rare compared with the sync delay (e.g. once per frame,
once per ms). For continuous data, use an async FIFO (Vivado FIFO Generator,
independent clocks).

### 14.4 Rules of thumb

- Synchronize **in the receiving domain**, never in the sender.
- Never feed an unsynchronized signal from another domain into logic with more
  than one destination flop: different flops can see different values.
- Signals from **physical pins / buttons** are asynchronous: always sync them.
- Read a cross-domain value only after a handshake says it's stable.
