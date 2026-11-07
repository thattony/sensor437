# Questions Worth Keeping

Technical questions worth remembering: ones that apply beyond a single lab or
show a good way of thinking about hardware. Each answer is written to make sense
on its own, months later, without redoing the lab. Concrete examples, timing
diagrams and tables are included where they help. Detailed patterns and code are
in `note.md` (section numbers given).

---

## Q1. Does this need a state machine, or can I just assign outputs from conditions?

**Short answer:** If an output must take **different values at different times
while the inputs stay the same**, you need a state register. If the output depends
only on the current inputs, an `assign` is enough.

**Example: an image sensor's power-up sequence.** The host sets `enable = 1` and
leaves it there. The FPGA must then:

```
time      0        20 µs        40 µs
enable    1 ─────────────────────────────   (never changes)
clk_en    0 ┐1 ──────────────────────────
sys_res_n 0 ───────────┐1 ───────────────
ready     0 ───────────────────────┐1 ───
          OFF │ CLK_ON  │ RES_WAIT  │ READY
```

`enable` is the same at 1 µs and at 30 µs, but `sys_res_n` must be 0 at the first
and 1 at the second. No formula of the inputs alone can do that. Something has
to remember *how far along the sequence we are*, and that something is the state
register.

| Situation | Use |
|---|---|
| Output = function of current inputs (`led = sw1 & sw2`) | `assign` |
| Output depends on order of events or elapsed time | state machine |

**Thinking direction:** freeze the inputs in your head. Should the output still
change over time? Yes → you need state.

→ note.md §2, §3

---

## Q2. How do you "wait" in Verilog?

**Short answer:** You can't pause hardware. Stay in a state and **count clock
cycles** until the count reaches the wait time.

**Why not `#20000`?** `#` delays only work in simulation. Synthesis ignores them:
real hardware has no "sleep". It only has a clock ticking.

**Compute the count:** `N = time × clock frequency`

| Clock | 1 cycle | 1 µs | 20 µs | 1 ms |
|---|---|---|---|---|
| 100.8 MHz | 9.92 ns | ~101 | 2016 | 100,800 |
| 25 MHz | 40 ns | 25 | 500 | 25,000 |

Example: "wait ≥ 1 µs" on a 100.8 MHz clock, with 20× margin → 20 µs → 2016 cycles.

**The pattern:**
```verilog
S_WAIT: begin
    if (cnt == N-1) begin   // waited N cycles (0 .. N-1)
        cnt <= 0;           // clear for the next state's wait
        st  <= S_NEXT;
    end else
        cnt <= cnt + 1;     // keep waiting
end
```

Every clock edge asks "done yet?". If not, it adds 1. After N edges the answer is
yes and the state moves on: exactly the time you wanted.

**Watch out for:**
- **Counter width:** you need `2^bits > N` (2016 → 12 bits). Too narrow and it
  wraps around before reaching N, so the state never exits.
- **Clear the counter** on every state change, or the next wait starts part-way.

→ note.md §4

---

## Q3. Do I need both an `always @(posedge clk)` block and an `always @*` block?

**Short answer:** No. One clocked block plus a few `assign`s is enough for most
state machines, and it's simpler and safer.

| Style | What it looks like | Trade-off |
|---|---|---|
| **One block** (recommended) | transitions, counters, outputs all in `always @(posedge clk)` | less code, no latch risk; outputs set inside appear 1 cycle after the state changes |
| **Two blocks** (textbook) | clocked block does `st <= next_st;`, `always @*` computes `next_st` and outputs | more code; **latch trap** |

**The latch trap:** in `always @*`, if a signal isn't assigned in *every* branch,
synthesis builds a latch to "remember" its old value. That's a real hardware
bug, not just style. Fix: put default assignments at the top of the block.

**The 1-cycle lag** in the one-block style is usually harmless: 10 ns against
waits of microseconds. If an output must change in the *same* cycle as the
state, make it an `assign` from the state instead.

→ note.md §6, §7

---

## Q4. Why set an output in every state if a register keeps its value anyway?

**Short answer:** It works without it, but only by luck of which path you came
from. Setting every output in every state makes the outputs depend only on the
current state.

**Three reasons:**
1. **`initial` runs once**, when the FPGA is configured. After the machine
   loops (OFF → … → READY → reset → OFF → …), values come from wherever it just
   was, not from `initial`.
2. **A state can be entered from several places.** Example: `CLK_ON` is entered
   from `OFF` at power-up, and again from `RES_LOW` after a reset request. If you
   only set a signal where it "changes", every arriving path must leave exactly
   the right value. Miss one and the bug appears only after a reset, which is
   hard to find.
3. **It reads as a table** you can check against the datasheet:

| State | clk_en | sys_res_n |
|---|---|---|
| OFF | 0 | 0 |
| CLK_ON | 1 | 0 |
| RES_WAIT | 1 | 1 |
| READY | 1 | 1 |
| RES_LOW | 1 | 0 |

**Not the same as `always @*`:** there, an unassigned signal makes a latch (a
real bug). In a clocked block it only costs safety and readability.

→ note.md §5

---

## Q5. Why is one output an `assign` while others are set inside the `always` block?

**Short answer:** They're different hardware. `reg` set in a clocked block is a
**flip-flop** (memory). `wire` + `assign` is **logic gates** (a permanent
formula).

| | `reg` in `always @(posedge clk)` | `wire` + `assign` |
|---|---|---|
| Hardware | flip-flop | gates (e.g. a comparator) |
| Changes | only on a clock edge | immediately when inputs change |
| Remembers? | yes, until assigned again | no, always equals its formula |
| Assigned with | `<=` inside the block | `assign x = …;` outside |

**Example:** `assign sensor_ready = (st == S_READY);` is a comparator wired to the
state flip-flops. Whenever the state is READY, the output is 1. There's nothing
to set or clear, and it can never disagree with the state. If you instead set it
by hand in each state, forgetting one state leaves it wrong.

**Rules:**
- A `wire` can't be assigned inside `always` (compile error).
- Each signal has **one driver**: never assign it from two blocks.
- The port declaration decides it: `output reg` → set in a block;
  `output wire` → `assign`.

**Rule of thumb:** "1 whenever we're in state X" → `assign`. Needs to remember
something or change at a specific moment (counter, timed pulse) → `reg`.

→ note.md §1

---

## Q6. What are `x_prev` and `x_rise` (edge detection)?

**Short answer:** `x_prev` is the input's value **one clock ago**.
`x_rise = x & ~x_prev` is 1 for **exactly one cycle** when the input goes 0 → 1.

**The problem it solves:** a host sends a "take one frame" trigger, which might
arrive as a 1-cycle pulse **or a held level**. Checking `if (x)` would fire on
every cycle the level is held, and the camera would keep capturing. You want to
act on the *moment it changes*.

```verilog
reg x_prev;
always @(posedge clk) x_prev <= x;   // delayed copy
wire x_rise = x & ~x_prev;           // "1 now, 0 last cycle"
```

| cycle | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|
| `x` (held high) | 0 | 0 | **1** | 1 | 1 | 1 |
| `x_prev` | 0 | 0 | 0 | 1 | 1 | 1 |
| `x_rise` | 0 | 0 | **1** | 0 | 0 | 0 |

In cycle 3 the input just rose but `x_prev` still remembers 0, so `x_rise` = 1.
From cycle 4 `x_prev` has caught up, so it's 0, even though `x` is still high.
One event, however long the button or trigger is held.

Falling edge: `~x & x_prev`. If `x` comes from another clock or a physical pin,
synchronize it first (Q9).

→ note.md §8

---

## Q7. Why is a pulse N clock cycles wide instead of just one cycle?

**Short answer:** The chip receiving the pulse samples it with **its own, slower
clock**. A 1-cycle pulse from a fast clock can start and end between two of its
clock edges and never be seen.

**Example:** the FPGA runs at 100.8 MHz (9.9 ns per cycle). An image sensor reads
its FRAME_REQ pin on its own 25 MHz clock (40 ns per cycle).

```
FPGA clk   _|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_     9.9 ns
sensor clk _______|‾‾‾‾‾‾‾‾|________|‾‾‾‾‾‾‾‾|_   40 ns
1-cycle    ___|‾|_____________________________   over before the sensor looks
pulse            ↑ sensor samples here → sees 0
```

**Sizing it:**
1. Datasheet minimum: ≥ 4 sensor clock periods = 4 × 40 ns = **160 ns**.
2. Convert to FPGA cycles: 160 / 9.92 = 16.1 → round **up** → **17 cycles**.
3. Add ~2× margin → **32 cycles = 317 ns**.

| | cycles | time |
|---|---|---|
| minimum | 17 | ~160 ns |
| chosen | 32 | ~317 ns |

**Too long is also wrong.** If the pulse stays high, the chip may treat it as a
level ("keep requesting"). The sensor would free-run, capturing frames forever.
Aim for a window: long enough to be seen, short enough to be one request. Here
317 ns against a 13 ms frame is negligible.

**Thinking direction:** for any pin you drive, ask *whose clock samples it, and
what does the datasheet say about minimum width / setup / hold?*

→ note.md §9, §14.2

---

## Q8. Why would a chip require a pulse of "≥ 4 of its clock periods"?

**Short answer:** The exact number is the manufacturer's spec: follow it, don't
derive it. The reasons behind rules like this are general, though.

**Typical reasons (general background, not quoted from a specific datasheet):**
1. **The pin is asynchronous** to the chip's clock. From the chip's view, it can
   change at any instant between its clock edges.
2. **The chip synchronizes it internally** (2–3 flip-flops, to avoid
   metastability). The signal must stay stable while it passes through them.
3. **Some chips filter glitches:** they accept "high" only after several
   matching samples, so noise on the wire doesn't trigger anything.
4. **Unlucky alignment costs up to one period:** if your pulse rises just after a
   chip clock edge, almost a whole period passes before the first sample. To
   guarantee N samples you need about N + 1 periods.

Together: "hold it for a few of my clock periods" is the chip maker's guarantee
that the input is recognized every time, whatever the timing.

**Practical lesson:** look up the **minimum pulse width / setup / hold** in the
datasheet for every input you drive, convert it to your clock, round up, and add
margin.

→ note.md §14

---

## Q9. Do these signals cross a clock domain?

**Short answer:** Check every signal: *which clock writes it, and which clock
reads it?* If they're different, unrelated clocks, it's a crossing and needs a
specific technique.

**Example: an FPGA driving a camera sensor.** The control logic runs on the USB
interface clock (100.8 MHz). The sensor runs on a 25 MHz clock made by a separate
clock generator. They have no fixed phase relationship, so they're
asynchronous.

```
 FPGA logic clock (100.8 MHz)            sensor clock (25 MHz)
┌───────────────────────────┐   pin   ┌────────────────────────┐
│ host trigger → pulse gen  ├────────▶│ sensor samples the pin │
│ (same clock: no crossing) │         │ on its own edges       │
└───────────────────────────┘         └────────────────────────┘
               ◀──── pixel counts (sensor clock → FPGA clock) ────
```

| Signal | Writer → reader | Crossing? | Technique |
|---|---|---|---|
| host trigger → pulse generator | FPGA → FPGA | no | edge detect only |
| frame-request pulse → sensor pin | FPGA → sensor | **yes** | stretch to ≥ N receiver periods (Q7) |
| clock-enable bit → clock gate | FPGA → 25 MHz | **yes** | 2-flip-flop synchronizer |
| 16-bit pixel count → host | sensor clock → FPGA | **yes** | toggle / handshake (not 2 flops per bit) |

**Why multi-bit values need special care:** each bit's synchronizer can settle
in a different cycle, so the reader can see a **torn** value (some bits new, some
old: 0x0FF → 0x100 read as 0x1FF). Hold the value stable, send one synced
"new value ready" toggle, and copy it on the toggle's edge.

**Rules of thumb:**
- Synchronize in the **receiving** domain.
- Physical pins and buttons are always asynchronous: sync them.
- When you drive another chip's pin, *you* hold the signal long enough; *it*
  synchronizes.

→ note.md §14
