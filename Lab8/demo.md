# Lab 8 Demo

**Before the TA comes:** `cd U:\ece437\Lab8`. Close FrontPanel. Turn on the E3631A (don't touch its buttons). Arm at home.
Always type `py -3.14` (not `python`).

---

## Check 1: Tilt angle (should be about 15°)

```
py -3.14 lab8_python.py tilt
```

- **Show:** `TILT beta = xx.xx deg`
- **Saved result:** `results\tilt_20261103_154112.json`, β = **14.83°**
- **Say:** "β comes from √(a_r² + a_t²) = g·sin β. A circle fit over an arm sweep removes the accelerometer offsets."

---

## Check 3: Std of 100 stationary readings (close to predicted)

Hands off the bench for about 7 s.

```
py -3.14 lab8_python.py resolution
```

- **Show:** `STANDARD DEVIATION = x.xxx deg` and the `predicted:` line
- **Saved result:** `results\resolution_20261103_154331.png`, `.json`
  - std = **0.63°**
  - predicted (pre-lab, 1 mg noise) = **0.22°**
  - predicted (measured noise 2.0 / 3.0 mg) = **0.66°**, which matches
- **Say:** "Predicted 0.22° assumed 1 mg of noise. The real noise is 2 to 3 mg, which predicts 0.66°, and I measured 0.63°."

---

## Check 4: When do you read the accelerometer?

```
py -3.14 lab8_python.py interactive
lab8> 1 20
lab8> read
lab8> q
```

- **Saved result:** `results\ring_20261103_154409_analysis.png` (settling = **0.40 s**)
- **Say:** "Only when the arm is stopped: move, stop, wait for it to settle, then read. While it turns, ω²r adds about 0.6 g to the radial axis, which is more than g·sin β ≈ 0.26 g, so the angle would be wrong. I also wait out the ringing, which takes about 0.4 s to settle."

---

## Check 8: Drive into the post (the detector trips, EN1 goes low, the log shows it)

Stand clear.

```
py -3.14 lab8_python.py stall 1
```

- **Show:**
  - the `TRIP` row (after `STRIKE 1`)
  - `FPGA fault latched: code 1`
  - `SAFE STATE: EN1 = 0, running = 0`
  - LED **D3 (FAULT)** lit on the sensor board
  - `SUPPLY: output OFF`
- **Saved result:** `results\stall_20261103_154815.json`, `.csv`, `_windows.csv` (tripped at 1.6 s, 320 pulses)
- **Say:** "The safe state is EN1 low, so the motor coasts and stops drawing stall current. The supply's 1.5 A limit is the backstop."

Then clear the fault and re-home:

```
py -3.14 lab8_python.py reset
py -3.14 lab8_python.py goto 0
```

---

**If something fails:** `REFUSED ... fault latched` → run `reset` and then `goto 0`. `Design ID` error → add `--load` once.
