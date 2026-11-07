# Lab 8 Run Guide

Every step has the same four parts: **Do** (with your hands), **Type**, **See** (what proves it worked) and
**Saved** (where the numbers go). After each step, tell me the step number. I'll read `results\` and write up
the numbers.

## Before anything

- Open **PowerShell** and type `cd U:\ece437\Lab8`.
- Always use **`py -3.14`**. Plain `python` is the `U:\ece437\.venv` interpreter, which has no numpy.
- **Close the FrontPanel app.** Only one program can hold the board.
- **You never touch the E3631A buttons.** Python finds it (RS-232, ASRL4), sets the **+6V output to 6.0 V
  with a 1.5 A limit**, switches it **ON** only for commands that drive the motor, and switches it **OFF**
  when the command ends, including on Ctrl-C or an error. While a command runs, the front panel shows
  `Rmt` and its keys are locked. That is normal. The E3631A just has to be switched on (mains power).
- The supply is **ON** for `interactive`, `calibrate`, `sweep`, `move`, `goto`, `stall` and `pulsetest --power`,
  and **OFF** for everything else.
- Emergency stop: **Ctrl-C**. EN1 goes low and the supply goes OFF. If anything looks wrong, press the E3631A
  **Output On/Off** key.

---

## Step 0: Wiring (once, with the E3631A output OFF)

| Wire | From | To |
|---|---|---|
| Pmod DHB1 | 2×6 header | either PMOD header on the sensor board (EN1 = pin 1, DIR1 = pin 2, GND = pin 5) |
| Motor | the motor's 2 wires | DHB1 blue screw block **M1+ / M1−** |
| Motor power | E3631A **+6V** (red) | DHB1 blue screw block **J4 VM** |
| Motor ground | E3631A **COM** (black) | DHB1 **J4 GND** |
| Board | USB | PC |
| E3631A | RS-232 cable | PC (already connected: it answers on ASRL4) |

- Read the white text next to each blue block. One is **VM/GND (J4)**, the other is the **motor output**.
  Don't swap them.
- Use the E3631A's **+6V** and **COM** terminals. Don't use ±25V or the chassis ground ⏚.
- Put a **book under one side of the base** so the shaft tilts about 15°.
- Leave the arm **in the middle**, never against the post.

---

## Step 1: Offline self-test

**Do:** nothing (no hardware needed).
**Type:** `py -3.14 lab8_python.py selftest`
**See:** the last line is `ALL PASSED`.

## Step 2: Program the board and check the accelerometer (supply OFF)

**Do:** arm still, hands off.
**Type:** `py -3.14 lab8_python.py check --load`
**See:**
- `FPGA: lab8_motor.bit loaded (ID 0xEC437008)`
- `LSM303: CTRL_REG1_A ... -> 0x77 (400 Hz)`
- about **400 samples/s**, `sequence gaps 0`, `NACKs 0`, then `PASS`
- `rotation axis ~ ...; in-plane: radial = x, tangential = z` (or similar). The rotation axis reads about 0.97 g.

**Saved:** `results\check_*.json`, `results\lab8_config.json`.

`--load` programs the FPGA. Leave it off from now on. The design stays loaded between commands, so a
latched stall fault survives until you `reset`.

## Step 3: Pulse test on the scope

**Do:** the DHB1 stays in its PMOD header. Probe the **other** PMOD header, which carries the same signals:
**CH1 → pin 1 (EN1)**, **CH2 → pin 2 (DIR1)**, ground clips → **pin 5**. Keep the clips off pins 6 and 12 (3.3 V).
The scope's **rear USB-B** port goes to the PC. If Python can't find the scope, replug that cable.
**Type:**

```
py -3.14 lab8_python.py pulsetest 10 1 --scope
py -3.14 lab8_python.py pulsetest 10 0 --scope
py -3.14 lab8_python.py pulsetest 10 1 --scope
```

(supply stays OFF). `--scope` makes Python set up the scope (1 V/div, single shot on the EN1 rising edge),
capture both channels and measure them. The capture also stays on the scope screen for the TA.
**See:**
- `FPGA counted 10 pulses (requested 10); EN1 now 0` and `PASS`
- `SCOPE CH1 EN1: 10 pulses seen (requested 10), period 4.999 ms (200.0 Hz), high 2.496 ms (49.9 %), low after: True`
- `SCOPE CH2 DIR1: level 0 (requested 0), changes while EN1 high: 0, DIR1 edge 0.99 ms before the first pulse`
- `SCOPE: PASS`

**Saved:** `results\pulsetest_scope_*.csv / .json / .png` (waveforms of EN1 and DIR1).

**Then, with motor power:** `py -3.14 lab8_python.py pulsetest 10 --power`
**See:** `SUPPLY: E3631A P6V ON, set 6.000 V / limit 1.500 A, measured ~6.0 V ...`, the arm **twitches**,
then `SUPPLY: output OFF`.
If the measured voltage is low, or the arm doesn't move, check J4 polarity and the motor wires on M1.

## Step 4: Put the arm at the centre and record home

**Type:** `py -3.14 lab8_python.py interactive`   (supply ON for the whole session)
At the `lab8>` prompt:

```
lab8> 1 20      <- DIR1 = 1, 20 pulses (0.1 s)
lab8> 0 10      <- the other way, 10 pulses
lab8> read      <- prints the angle
```

**Do:** jog until the arm is **halfway between the two sides of the post**. Then:

```
lab8> home
lab8> q
```

**See:** `Home recorded ...` and, after `q`, `SUPPLY: output OFF`.
From now on every angle is relative to home, so the travel is about −150° to +150°.

## Step 5: Tilt β, first look (TA check 1, rough)

**Do:** arm stopped, hands off.
**Type:** `py -3.14 lab8_python.py tilt`
**See:** `TILT beta = xx.xx deg`, near 15°. This still includes the accelerometer offsets. Step 7 removes them.
**Saved:** `results\tilt_*.json`.

## Step 6: Calibrate the stall detector

**Do:** arm at home. Stand clear: the arm moves by itself for about 10 s.
**Type:** `py -3.14 lab8_python.py calibrate`
**See:**
- `DIR1 = 1 turns phi UP` (or DOWN): **note which**, because you'll need it for `move` and `stall`
- `EXPECTED = xx.x deg / 100 ms -> trip below x.x deg (25 %)`
- `back home`, then `SUPPLY: output OFF`

**Saved:** `results\calibrate_free_*.csv` and the EXPECTED value in `lab8_config.json`.
If it says `arm moved only 0.x deg`, check the supply line it printed (V and A), J4 and the motor wires.

## Step 7: Sweep and offset-free β (the β you report)

**Do:** stand clear. It stops at 12 positions from −110° to +110° and returns home.
**Type:**

```
py -3.14 lab8_python.py sweep
py -3.14 lab8_python.py tilt
```

**See:** `circle fit: centre (offset) ...` and `TILT beta (offset-free) = xx.xx deg`. Then `tilt` prints
`offsets from the sweep circle fit removed` and the final β.
**Saved:** `results\sweep_*.csv / .json / .png`, `results\tilt_*.json`.
**Write down:** β from the second `tilt`. This is TA check 1.

## Step 8: Resolution (TA check 3, post lab 1a)

**Do:** arm stopped. **Don't touch the bench** for about 7 s.
**Type:** `py -3.14 lab8_python.py resolution`
**See:** `100 readings: ... STANDARD DEVIATION = x.xxx deg` and
`predicted: pre lab ... = 0.221 deg; from the measured noise ... = x.xxx deg`.
**Saved:** `results\resolution_*.csv / .json / .png`.
**Write down:** the std, the predicted values, and the radial/tangential noise in mg.

## Step 9: Ringing log (post lab 3b)

**Do:** stand clear. Use the DIR that turns phi **UP** (from Step 6). The commands below assume DIR 1.
**Type:**

```
py -3.14 lab8_python.py goto -60
py -3.14 lab8_python.py move 60 1 --label ring
py -3.14 lab8_python.py analyze results\ring_<time>.csv
```

(Copy the exact CSV name that `move` printed after `-> analyze it:`.)
**See:** `overshoot_deg`, `settle_s`, `max_rate_per_s` (= trustworthy stop-and-read measurements per second).
**Saved:** `results\ring_*.csv / .json`, `results\ring_*_analysis.png`.

## Step 10: Torque at 6 V and 4 V (post lab 4a, 4c)

**Do:** stand clear. Python sets the voltage itself with `--volts`.
**Type:**

```
py -3.14 lab8_python.py goto -60
py -3.14 lab8_python.py move 60 1 --label 6V --volts 6
py -3.14 lab8_python.py goto -60
py -3.14 lab8_python.py move 60 1 --label 4V --volts 4
py -3.14 lab8_python.py analyze results\6V_<time>.csv results\4V_<time>.csv
```

**See:** `SUPPLY: ... set 4.000 V` on the 4 V run. `analyze` prints `alpha_mean_first_40ms`, `torque_mean_Nm`
for each file and a **Torque comparison (4c)** table.
**Saved:** `results\6V_*`, `results\4V_*` (each JSON records the set and measured supply voltage), and the
`*_analysis.png` plots.
Every later command goes back to the 6 V default, so you have nothing to reset.

## Step 11: Drive into the post (TA check 8)

**Do:** arm at home (`py -3.14 lab8_python.py goto 0` if not). Stand clear. Practice once before the TA comes.
**Type:** `py -3.14 lab8_python.py stall 1`   (`stall 0` drives the other way)
**See:**
1. a row every 100 ms: `phi_now  moved  moved_atan2  thresh  verdict`
2. the arm hits the post: `moved` drops to about 0: one row says `STRIKE 1`, the next (strike 2) says `TRIP`
3. the **STALL LOG**:
   - `DETECTOR TRIPPED at t = ... s ... phi = ... moved ... < ...`
   - `FPGA fault latched: code 1 (1 = stall trip), pulses sent at trip ...`
   - `SAFE STATE: EN1 = 0, running = 0`
   - `samples after the trip with EN1/running high: 0 of ...`
4. `SUPPLY: output OFF`

**Saved:** `results\stall_*.csv`, `results\stall_*_windows.csv`, `results\stall_*.json`.
**Then clear the fault and re-home:**

```
py -3.14 lab8_python.py reset
py -3.14 lab8_python.py goto 0
```

Any move before `reset` is refused. That is the required "no moves until reset" behaviour.

---

## Demo for the TA (in this order)

Do Steps 1 to 11 before the TA arrives, so the arm is homed, calibrated and the sweep is done.

| # | TA asks | You type | You say |
|---|---|---|---|
| 1 | Tilt angle | `py -3.14 lab8_python.py tilt` | "β = __°, from √(a_r² + a_t²) = g·sin β. The accelerometer offsets are removed by a circle fit over a sweep of the arm." |
| 3 | Std of 100 stationary readings | `py -3.14 lab8_python.py resolution` (or open the saved PNG) | "std = __°. Predicted 0.22° from 1 mg at 15°; with my measured noise of __ mg it predicts __°." |
| 4 | When do you read the accelerometer? | `py -3.14 lab8_python.py interactive`, then `1 20`, `read`, `q` | "Only when stopped: command, stop, wait to settle, read. While it turns, ω²r adds to the radial axis (about 0.6 g at full speed, more than g·sin β ≈ 0.26 g), so the angle is wrong. I also wait out the ringing: settling takes __ ms." |
| 8 | Drive into the post | `py -3.14 lab8_python.py stall 1`, then `reset`, `goto 0` | Point at the TRIP row, `SAFE STATE: EN1 = 0, running = 0` and the LED **D3 (FAULT)** on the sensor board. "Safe state is EN1 low: coasting. The motor stops drawing stall current, and the supply has a 1.5 A limit as a backstop." |

Upload to Canvas: `U:\ece437\Lab8\lab8_motor.v` and `U:\ece437\Lab8\lab8_python.py`.

---

## If something goes wrong

| Symptom | Fix |
|---|---|
| `No module named 'numpy'` | You typed `python`. Use `py -3.14`. |
| `E3631A not found` | Switch the E3631A on and check the RS-232 cable. To set it by hand instead: front panel 6.0 V / 1.5 A, Output ON, and add `--no-supply`. |
| `Design ID ... expected 0xEC437008` | Add `--load` once. |
| `REFUSED ... fault latched` | `reset`, then `goto 0`. |
| `run 'calibrate' first` | Do Step 6 (`goto` and `sweep` need it). |
| The arm doesn't move but the supply says ON | The motor wires aren't on M1, or the Pmod is in the wrong place. |
| Measured VMOTOR well below the set value | Short or reversed wires on J4. The script refuses and switches OFF. |

## Why the stall detector uses the chord, not atan2

At 45° per 100 ms, ω²r ≈ 0.6 g lands on the radial axis. That is more than the whole gravity signal
g·sin β ≈ 0.26 g, so atan2 is wrong while the arm turns and can even run backwards. The chord between two
window averages, 2·asin(|v_now − v_prev| / 2G), cancels the centripetal term because it is the same in both,
and at a stall (ω = 0) it equals the atan2 change. The atan2 change is still printed in every row. Otherwise it
is the pre-lab pseudocode: 100 ms windows, `moved < 25 % of EXPECTED` is a strike,
2 strikes trip. One more change: each window averages all 40 samples, not 8, because the arm vibrates while
pinned against the post and 8-sample averages wander by 10° or more (the first stall test took 3.9 s to trip). A second layer: the FPGA has a 250 ms watchdog, so if Python hangs mid-move, EN1 goes low
(fault code 2).

## Endpoint map

| Endpoint | Use |
|---|---|
| WireIn 0x04 / 0x05 | pulse count N; direction (bit 0), duty % (bits 15:8) |
| WireIn 0x06 | sampler enable, watchdog enable, watchdog timeout in ms |
| TriggerIn 0x40 | bit 2 GO, 3 STALL TRIP, 4 clear fault, 5 FIFO reset, 6 watchdog kick, 7 STOP |
| WireOut 0x26 | running, EN1, DIR1, fault, fault code, ... |
| WireOut 0x27 / 0x2C / 0x2D | pulses sent, fault timestamp, pulses sent at the fault |
| PipeOut 0xA0 | samples, 16 bytes each: timestamp, X/Y/Z, status, EN1/DIR1/running/fault, pulses |
| Sensor-board LEDs | D0 = EN1, D1 = DIR1, D2 = running, D3 = FAULT |
