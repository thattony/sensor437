# Midterm Part 1 — Demo Guide

Work in **PowerShell**, in the midterm folder:

```powershell
cd U:\ece437\midterm
```

Every command below is `py -3.14 midterm_p1_python.py <mode>`. Each one loads the bit file, runs, prints a
PASS/FAIL line, and saves its numbers to `results\` for Report 1.

Close the FrontPanel application before you run anything: only one program can hold the board.

---

## Part A — Before the demo day: collect the data (about 15 min)

Do these once, in this order. The TA looks at the six-orientation table and the broken run, so they
must exist before you arrive.

### A1. Power-cycle the board, then run the known-answer test FIRST

The accelerometer keeps its registers until it loses power. Reloading the FPGA does not reset it.
Step 1 of the test needs the true power-on value, so:

1. Unplug the board's power, wait 5 s, and plug it back in.
2. Run:

   ```powershell
   py -3.14 midterm_p1_python.py kat
   ```

3. You should see:

   ```
   Step 1: read CTRL_REG1_A (0x20). Documented power-on default 0x07, read 0x07  -> PASS
   Step 2: wrote 0x37 to CTRL_REG1_A, read back 0x37  -> PASS
   Step 2: wrote 0x57 to CTRL_REG1_A, read back 0x57  -> PASS
   Extra:  magnetometer IRA/IRB/IRC ... = 48 34 33 ... -> PASS
   ```

   If step 1 shows anything other than `0x07`, something wrote the register since power-up. Power-cycle again
   and run `kat` before any other mode.

Saved to: `results\kat.json`

### A2. Six-orientation gravity table

```powershell
py -3.14 midterm_p1_python.py six
```

For each of the 6 prompts:

1. Lay the board on one face or edge on the bench, so that it is **flat and still**.
2. Type a short description, e.g. `component side up` or `USB edge down`, and press **Enter**.
3. **Take your hands off the board.** The script waits 1 s, then averages 400 new samples (2 s).

Use all six directions: component side up, component side down, then the four edges, each standing on the bench.
If you repeat a direction the script tells you and asks again.

At the end you should see `spread ... -> PASS (limit 0.15 g)`.

**Write down which physical position each line was**, for example "+X = USB edge pointing down". Report
question 1c needs it.

Saved to: `results\six_orientations.csv`, `results\six_summary.json`, `results\six_raw_samples.csv`

*(Hands-free option: `py -3.14 midterm_p1_python.py six --auto` captures each new still orientation
automatically while you turn the board.)*

### A3. Broken on purpose: auto-increment bit cleared

```powershell
py -3.14 midterm_p1_python.py broken
```

Do 2 orientations, e.g. component side up and one edge. Hold each still and press **Enter**.

For each one the script reads correctly first (SUB = `0xA8`), then with bit 7 cleared (SUB = `0x28`) and
nothing else changed. You should see:

- all six bytes identical in every read;
- ax = ay = az;
- the line `every axis = (XL<<8|XL)>>4 mg: True`, which means the arithmetic prediction matches.

The bit is "put back" automatically: every other mode uses `0xA8`.

Saved to: `results\broken_autoinc.json`

### A4. Magnetometer (not graded, but the report asks what you saw)

```powershell
py -3.14 midterm_p1_python.py mag
```

Slowly turn the board through every orientation for 20 s.

Saved to: `results\mag_summary.json`, `results\mag_rotation.csv`

### A5. Sustained rate (already done, re-run if you like)

```powershell
py -3.14 midterm_p1_python.py rate
```

Takes about 45 s. Saved to: `results\rate_summary.json`, `results\rate_30s_samples.csv`

When A1–A4 are done, ask Claude to build `midterm_report1.pdf` from `results\`.

---

## Part B — Upload BEFORE the TA starts (or the demo scores zero)

Canvas → **Midterm – Part 1** → upload these two files, not zipped:

| File | Where it is |
|---|---|
| `midterm_p1_i2c.v` | `U:\ece437\midterm\midterm_p1_i2c.v` |
| `midterm_p1_python.py` | `U:\ece437\midterm\midterm_p1_python.py` |

`midterm_report1.pdf` goes to the separate **Midterm – Report 1** assignment.

Check that both files show as submitted in Canvas before you call the TA.

---

## Part C — At the bench: what the TA checks, in order

Power-cycle the board just before the TA arrives, so check 1 shows `0x07`.

### Check 0 — Code is on Canvas

Show the Canvas page with both files.

### Check 1 — Known-answer test

```powershell
py -3.14 midterm_p1_python.py kat
```

**Show:** step 1 reads `0x07` = the documented default; step 2 writes `0x37` and `0x57` and reads each back exactly.

**Say:** "The accelerometer has no ID register, so I read a register whose power-on default is documented and is not zero.
CTRL_REG1_A at 0x20 powers up as 0x07. That proves the read path. Then I write values I chose and read them back,
which proves the write path and that I'm addressing the register I think I am. The magnetometer ID 48 34 33 only proves
the magnetometer, which is a different device at 0x1E."

### Check 2 — Tilt the board live

```powershell
py -3.14 midterm_p1_python.py live
```

Tilt the board slowly. The axis pointing down moves toward ±1 g, the others go to 0, and **|a| stays ≈ 1.00**.
Press **Ctrl+C** to stop.

**Say:** "CTRL_REG1_A = 0x67: 200 Hz, normal mode, X/Y/Z on. CTRL_REG4_A = 0x88: block data update, ±2 g,
high resolution. That gives 12-bit data at 1 mg per LSB. All six output bytes come from one burst read starting at
SUB = 0xA8, which is 0x28 with bit 7 set for auto-increment."

### Check 3 — Six-orientation table

Open `results\six_orientations.csv`, or the table in your report.

**Show:** all six rows, the |a| column, and the **spread (max − min) under 0.15 g**.

### Check 4 — Broken-on-purpose run

Open `results\broken_autoinc.json`, or re-run `py -3.14 midterm_p1_python.py broken` live.

**Say (the signature, one sentence):** "With SUB(7) cleared, the pointer never leaves OUT_X_L_A, so all six bytes are
the X low byte. ax, ay and az come out identical, equal to (XL·257) >> 4 mg, and they don't follow the orientation."

### Check 5 — 100 samples/s for 30 s

```powershell
py -3.14 midterm_p1_python.py rate
```

**Show:** the running counter, then the line `PASS: >= 100 samples/s for 30 s`. The measured rate is about 169 new
samples/s and the slowest single second is about 160.

**Say:** "SCL is 100 kHz. Each poll is one 7-byte burst from STATUS_REG_A: 3 header bytes + 7 data bytes = 10 bytes on
the bus, about 0.92 ms. The rest of the 1.46 ms per poll is USB."

### Check 6 — How do you know the samples are independent?

**Say:**

1. "I read STATUS_REG_A in the same burst as the data and only count a sample when ZYXDA = 1, meaning a new X/Y/Z set
   has arrived since the last read. A poll without ZYXDA is thrown away."
2. "The count follows the ODR setting, not my poll rate. I poll at about 685/s, but 50, 100, 200 and 400 Hz give 43,
   86, 167 and 341 counted samples/s. Timing the data changes directly shows the sensor converts at its nominal rate:
   10 Hz → 101 ms period, 100 Hz → 10.2 ms."
3. "The counted rate is about 15% below the ODR because of a race. When a conversion lands mid-burst, after the
   status byte has gone out, that poll is discarded. It costs rate, not independence, so I chose 200 Hz: at 100 Hz
   I'd count only about 86/s and fail."
4. "If I ignore ZYXDA, I get about 685 reads/s, but most of them are re-reads of the same conversion: numbers without
   information. Only 13 of 5059 counted samples repeated exactly, which is what noise alone predicts."

---

## Quick reference

| Item | Value |
|---|---|
| I²C machine | Lab 6 generated one, extended (burst read 1–8 bytes, register write, run-time SCL divide) |
| Bus / addresses | I²C bus 1; accelerometer 0x19 (0x32/0x33), magnetometer 0x1E (0x3C/0x3D) |
| Known-answer register | CTRL_REG1_A 0x20, default 0x07 |
| Configuration | CTRL_REG1_A = 0x67 (200 Hz), CTRL_REG4_A = 0x88 (BDU, ±2 g, HR) |
| Scale | 12-bit left-justified, 1 mg/LSB → `g = (int16(H:L) >> 4) / 1000` |
| Burst read | SUB = 0xA8 (6 bytes) or 0xA7 (status + 6 bytes) |
| Broken read | SUB = 0x28 |
| SCL | 100 kHz (TICK_DIVIDE 252 at 100.8 MHz) |
| Scope marker | PMOD A1 = read-address ACK window, PMOD A2 = busy (whole transaction), GND = PMOD pin 5 |
| Rebuild FPGA | `cd SensorBoard_I2C; $env:Path="C:\Xilinx\Vivado\2022.2\bin;$env:Path"; & "C:\Program Files\Git\bin\bash.exe" ./build.sh` |
| Simulation | 47 checks pass, including the broken-auto-increment case (`SensorBoard_I2C\sim\sim.log`) |
