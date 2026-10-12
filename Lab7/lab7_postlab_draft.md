# Lab 7 Post Lab — draft (M1, M2a and M2c parts)

Status: all four questions are complete.
All numbers come from `results/` (files named next to each answer). Nothing new was measured for this draft.

Setup used for all M1 data: ADT7420 in 16-bit mode (config register 0x03 = 0x80, read back and checked
before every run), 1 LSB = 1/128 °C = 0.0078125 °C, read through the FPGA I2C master, one transaction per
Python trigger (FPGA read counter checked).

---

## Question 1: Where averaging stops helping

### 1a. Log-log plot of sigma against N

Figure: `results/m1b_averaging.png` (std of the mean of N consecutive fast readings vs N, with the
sigma_1/sqrt(N) reference line).

Data: 2048 consecutive readings, taken as fast as the loop runs, at room temperature with the heater off.
The run took 2.476 s wall-clock (827 readings/s, 1.21 ms per reading). Source: `m1b_fast.csv`, `m1b_fast_runinfo.csv`, `m1b_averaging.csv`.

| N | measured sigma of N-averages (°C) | sigma_1/sqrt(N) (°C) | measured / ideal |
|---|---|---|---|
| 1 | 0.01689 | 0.01689 | 1.00 |
| 2 | 0.01685 | 0.01194 | 1.41 |
| 4 | 0.01684 | 0.00844 | 2.00 |
| 8 | 0.01683 | 0.00597 | 2.82 |
| 16 | 0.01674 | 0.00422 | 3.97 |
| 32 | 0.01657 | 0.00299 | 5.55 |
| 64 | 0.01617 | 0.00211 | 7.66 |
| 128 | 0.01529 | 0.00149 | 10.2 |
| 256 | 0.01432 | 0.00106 | 13.6 |

Answer: the data only matches the line at N = 1, where the line is defined to pass through the data.
It has already left the line at N = 2, where averaging two readings changes sigma by 0.2 % instead of the expected 29 %.
The plot's marker, which flags the first N where the data is more than 1.5 times the line, puts the departure at N = 4.
The real departure is the first step, N = 2.
The curve stays flat up to N ≈ 64. It starts to fall slowly only at N = 128–256, which is when one block of N readings begins to span more than one sensor conversion (see 1b).

### 1b. The two hundred-reading sets

| Set | Standard deviation | Distinct values |
|---|---|---|
| First 100 fast readings (all within 0.12 s) | **0.0000 °C** | 1 (every reading 28.5234 °C, raw 0x0E43) |
| 100 readings, one every 300 ms (30 s) | **0.0180 °C** (2.3 LSB) | 11 |

Source: `m1b_summary.csv`, `m1b_fast.csv`, `m1b_slow300ms.csv`, figure `results/m1b_traces.png`.

Explanation: the ADT7420 makes a new conversion only about every 240 ms. Between conversions the
temperature register keeps returning the last result. In the 2048-reading fast run the value changed only
8 times, after runs of 203–204 identical readings. 203 readings × 1.21 ms = 0.245 s, which matches the
datasheet's 240 ms conversion time.

- The first 100 fast readings took 0.12 s, half a conversion period. They are 100 copies of a single conversion.
  Their std is zero because nothing was measured 100 times: one measurement was read 100 times.
  The whole 2048-reading run holds only about 2.48 s / 0.24 s ≈ 10 independent conversions.
- The 300 ms set waits longer than one conversion between reads, so every reading is a new conversion.
  The lag-1 autocorrelation of that set is 0.005, so successive readings are independent.
  The linear drift over the 30 s is −0.023 °C/min, and removing it leaves the std at 0.0177 °C.
  So 0.018 °C is the sensor's real reading-to-reading noise.

Which set averaging could help: only the 300 ms set. Averaging its 100 independent readings
gives an uncertainty of the mean of about 0.018/sqrt(100) ≈ 0.0018 °C.
Averaging the 100 fast readings gives back exactly the one conversion they all copy, so its uncertainty is still 0.018 °C.
The zero std of that set is a false precision. This is why the curve in 1a is flat: until a block of N readings spans several conversions, averaging more of them averages the same few numbers.

### 1c. Flat region vs one LSB and LSB/sqrt(12)

| Quantity | Value (°C) |
|---|---|
| Flat region of the 1a curve (N = 1 to 64) | ≈ 0.0168 |
| One 16-bit count (1 LSB) | 0.0078 |
| LSB/sqrt(12) (ideal quantization noise) | 0.0023 |
| Std of the independent 300 ms set | 0.0180 |
| Std of M1a levels (180 s wait), linear drift removed (`m1a_summary.csv`) | 0.015 – 0.021 |

The flat region is the largest: about 2.2 LSB and about 7.4 × LSB/sqrt(12).

So the measurement was not limited by the 16-bit resolution: quantization alone would give only 0.0023 °C.
Once the curve was flat, it was measuring the sensor's own conversion-to-conversion noise, which is about 0.017–0.019 °C. The same value appears in three independent places: the flat region, the 300 ms set, and the unheated level of M1a.
The flat region shows that noise frozen. Averaging back-to-back readings only re-averages the same ≈10 conversion results, so it can never go below that floor. Averaging can reduce it only with more independent conversions, which means more time, at about 240 ms per conversion.

---

## Question 2: What integration time does

Setup (M2a): Keysight 34461A, DC volts, measured across the 47 Ω power resistor with the supply output OFF.
Leads in the V HI/LO jacks, front terminals, 10 MΩ input, range fixed at 100 mV (autorange's choice), autozero ON.
200 readings at each NPLC. The meter supports NPLC 0.02 to 100. A requested NPLC of 0.06 is not available, and the meter set 0.2 instead.
Source: `results/m2a_nplc_summary.csv`, `m2a_nplc_raw.csv`.

### 2a. Plot

Figure: `results/m2a_nplc.png` (std of 200 readings vs integration time NPLC/60 Hz, log-log axes, 1 PLC marked).
Supporting figure: `results/m2a_raw_readings.png` (the 200 raw readings at each setting).

| NPLC | Integration time | Mean | Std of 200 readings | Time for 200 readings |
|---|---|---|---|---|
| 0.02 | 0.333 ms | 426 µV | **200.3 µV** | 0.2 s |
| 0.2 | 3.33 ms | 402 µV | **149.1 µV** | 1.3 s |
| 1 | 16.7 ms | 457 µV | **26.3 µV** | 7.1 s |
| 10 | 167 ms | 463 µV | **23.1 µV** | 68 s |
| 100 | 1.667 s | 462 µV | **25.9 µV** | 676 s |

The std falls by a factor of 7.6 between NPLC 0.02 and NPLC 1. Almost all of that drop happens between 0.2 and 1 PLC.
Above 1 PLC it stays flat at 23–26 µV: a 100 times longer integration bought nothing.

### 2b. What NPLC = 1 does to 60 Hz, and what the meter does that a scope sample does not

**The interference is 60 Hz.** At NPLC 0.02 the raw readings form a repeating waveform about 1 mV peak-to-peak (`m2a_raw_readings.png`, top panel).
The readings repeat every 16 samples: the autocorrelation at a lag of 16 readings is 0.97.
At about 1.04 ms per reading (0.33 ms of integration plus the autozero measurement and overhead), 16 readings is 16.7 ms, one mains period.
So the 200 µV spread at the shortest setting is not random meter noise.
It is the 60 Hz pickup in the lead and resistor loop, sampled at a different phase every reading.

**Integrating over exactly one period cancels it.** A reading is the average of the input over the integration time T.
For a sine at frequency f, that average is multiplied by |H(f)| = |sin(πfT)/(πfT)|. This response is zero whenever f·T is a whole number.
With T = 1 PLC = 1/60 s, the nulls fall exactly at 60, 120, 180, … Hz, so the mains signal and all of its harmonics integrate to zero.
Every positive half-cycle is cancelled by the negative half-cycle, whatever phase the reading starts at.
Shorter windows hardly suppress it:

| Frequency | \|H\| at NPLC 0.02 | \|H\| at NPLC 0.2 | \|H\| at NPLC 1 |
|---|---|---|---|
| 60 Hz | 0.999 | 0.935 | 0 |
| 180 Hz | 0.994 | 0.505 | 0 |
| 300 Hz | 0.984 | 0 | 0 |

At NPLC 0.02 the window is 2 % of a cycle, a near-instantaneous snapshot, so the full interference shows up in the readings (200 µV).
At NPLC 0.2 the harmonics are partly removed but the 60 Hz fundamental passes almost untouched (0.935), so the std only falls to 149 µV.
At NPLC 1 the fundamental and every harmonic are nulled, and the std falls to 26 µV.
Integrating longer than 1 PLC (10 or 100 PLC) is also a whole number of cycles, so it adds nothing for 60 Hz.

**The 23–26 µV floor is not 60 Hz and not white meter noise.** It doesn't fall from NPLC 1 to 100, whereas white noise would fall as 1/sqrt(T), by a factor of 10.
The NPLC 10 and 100 traces wander slowly over tens of readings: successive readings are correlated (lag-1 correlation 0.5–0.7).
So what's left is the node itself drifting at frequencies far below 1 Hz, which no window up to 1.67 s can average away.
The likely sources are thermoelectric voltages in the leads and resistor and the offset from the switched-off supply, which also gives the 0.46 mV DC level.

**What the meter does that a scope sample doesn't:** the DMM's ADC **integrates**: each reading is the time average of the input over the whole window.
That makes it a filter (the sinc response above) with nulls at every multiple of 1/T, and it rejects the mains completely when T is a whole number of line cycles.
A scope sample is an instantaneous snapshot of the voltage at one moment, a few ns wide.
It does no averaging at all, so each sample carries the full instantaneous value of the 60 Hz interference.
That is exactly what the NPLC 0.02 readings look like, and it is why the scope and its FFT in M2b see the 60 Hz in full.

---

## Question 3: The loop you built by accident

Setup (M2b):
- **Node:** the same as in M2a, across the 47 Ω resistor with the supply output OFF.
- **Scope:** MSO-X 3024T CH1, AC coupled, 20 mV/div, NORMAL acquisition (no averaging).
- **Record:** 1 s at 2 MSa/s, 2,000,000 points, giving 1 Hz FFT bins.
- **FFT method:** subtract the mean, apply a Hann window, compute the rFFT, and scale to peak volts.
- **Two captures:**
  - **Short loop:** probe ground spring from the + leg to the − leg of the resistor.
  - **Long loop:** probe tip on the same + leg; probe ground through a long wire to the supply's − (COM) terminal, laid out as a large loop.
- **Source:** `results/m2b_summary.csv`, `m2b_short_wave.npz`, `m2b_long_wave.npz`.

### 3a. FFTs, 60 Hz amplitudes, and loop areas

Figure: `results/m2b_fft.png` (both spectra on the same log axes). Supporting figure: `m2b_waveforms.png`.

| | Short loop | Long loop |
|---|---|---|
| **60 Hz amplitude** | **0.207 mV peak** (0.146 mV rms) | **4.81 mV peak** (3.40 mV rms) |
| 60 Hz above the FFT noise floor | 127× | 1511× |
| Total AC rms of the record | 0.46 mV | 3.72 mV |
| Probe-ground loop area | ≈ 1 cm² (ground spring, about 1 cm × 1 cm) | 900 cm² (30 cm × 30 cm) |

**Amplitude ratio (long/short): 23.3.**
**Area ratio (long/short): 900.** Amplitude ratio / area ratio = 0.026.

**Does it match?** Not if the short loop really is 1 cm². The induced voltage is V = dΦ/dt = 2π·60 Hz·B·A, so a uniform field would make the amplitudes scale with area.
The amplitude moved in the right direction, but by 23×, not 900×. The probe-ground loop is not the only loop in the circuit.

The resistor is wired to the supply by its own leads, and those leads enclose an area of their own. That loop is present in **both** configurations and picks up 60 Hz no matter how the probe is grounded.
If the effective background area is A_b, then (A_b + 900)/(A_b + 1) = 23.3, which gives **A_b ≈ 39 cm²**: about a 6 cm × 6 cm loop.
That is a plausible size for the supply leads and the resistor board, so the data are consistent with induction proportional to area once the fixed background loop is included.
Even the "short" capture is dominated by that background, so the 0.21 mV is not really the spring's pickup.

Other reasons the agreement can only be rough:
- The mains magnetic field is not uniform over a 30 cm loop; it is strongest near the supply's transformer and the power cords.
- The loop's orientation relative to the field matters; only the flux through it counts.
- Part of the pickup is capacitive (electric field) rather than magnetic, and does not scale with area.

The long loop also raised the **odd harmonics** of the mains by about the same factor as the fundamental. Peak amplitudes, short loop → long loop:

| Frequency | Short | Long | Increase |
|---|---|---|---|
| 300 Hz (5th) | 17 µV | 0.47 mV | about 28× |
| 660 Hz (11th) | 30 µV | 0.68 mV | about 23× |
| 780 Hz (13th) | 32 µV | 0.73 mV | about 23× |

Odd harmonics come from the currents drawn by rectifiers and switching supplies in the room. They are picked up by the same loop, and their ratio is similar to the fundamental's.

*Probe note:* both captures used the same probe at 1× (the scope reported 1× for CH1), so the absolute values and the ratio are consistent.

### 3b. Random noise, interference, or bias?

**Interference.** The 60 Hz line is not random. It sits at exactly 60.00 Hz, with a fixed amplitude, and its harmonics are exactly 180, 300, 420, 660 and 780 Hz.
It appears whenever the loop is present and grows with loop area (23× here), which identifies a deterministic external source: the magnetic and electric fields from the mains wiring coupling into the probe's ground loop.
It is not bias either. Bias would be a constant offset, and this averages to zero over a cycle.

**What that means for averaging:** averaging only reduces noise that is uncorrelated from one reading to the next.
Interference is perfectly correlated with the mains.
- Averaging scope records that are triggered in step with the line, or that happen to sample at the same phase, adds the 60 Hz coherently and never reduces it.
- Averaging at random phases reduces it only slowly, and only by luck of the timing.

It must be removed at the source, by making the loop small (ground spring), or by integrating over exactly whole mains cycles (NPLC = 1, as in Question 2).
Averaging would never have removed it.

---

## Question 4: What the bus can really do

Setup (M2c):
- **Scope:** Keysight MSO-X 3024T, 200 MHz, 2.5 GSa/s, 0.4 ns per sample. DC coupled, no bandwidth limit, both probes at 10×.
- **CH1:** SDA_0, test point TP10, probe ground on the JTAG header GND.
- **CH2:** the FPGA marker on PMOD J1 pin 1, ground on J1 pin 5. The scope triggers on its rising edge.
- **The marker:** it is high from just before the last bit of the read-address byte (0x91) until the ACK clock.
  - The FPGA releases SDA for that bit, because bit 0 = 1 and the line is open-drain, so only the pull-up raises the line.
  - The ADT7420 then pulls SDA low for the ACK.
- **Runs:** 10 captures, each triggered by one temperature read.
- **Source:** `results/m2c_edges.csv`, `m2c_summary.csv`, `m2c_ack_edge.npz`.

### 4a. The ACK edge

Figure: `results/m2c_ack_edge.png`. The top panel shows SDA and the marker over the whole bit. The bottom panels zoom in on the rise and the fall, with the 10 % and 90 % levels marked.

| | Mean ± std of 10 captures |
|---|---|
| **10–90 % rise time** | **925 ± 11 ns** |
| **90–10 % fall time** | **8.32 ± 0.05 ns** |
| Rise / fall | ≈ 111 |
| SDA levels | low −0.04 V, high 3.36 V |
| ACK delay | the sensor pulls SDA low 0.15 µs after SCL falls |

The scope's 200 MHz bandwidth gives it a rise time of about 0.35/200 MHz ≈ 1.75 ns. The true fall is therefore about √(8.32² − 1.75²) ≈ 8.1 ns, so the measured value is essentially the real edge.

**Why the edges are so different:** I2C is open-drain, so nothing ever drives the line high.

- **Rise.** After the FPGA releases SDA, the only thing pulling it up is the **10 kΩ pull-up resistor (R6)**. It charges the whole bus capacitance: the wiring, the pins of three sensors on bus 0, the FPGA pin, and the scope probe. That is a passive RC charge, so it is slow, about 1 µs. The rise has the exponential shape of an RC charge, fast at first and then levelling off.
- **Fall.** The ADT7420's output transistor turns on and pulls SDA to ground. Its on-resistance is roughly tens of ohms, compared with 10 kΩ, so it empties the same capacitance about a hundred times faster: 8 ns.
- **Ringing after the fall.** The fall undershoots to about −0.5 V and rings with a period of roughly 15–20 ns. That comes from the probe's ground lead (its inductance resonating with the capacitance). It isn't on the bus itself, which is why a short probe ground matters for fast edges.
- **Small bumps on SDA at the SCL edges** (t ≈ 5 and 10 µs) are capacitive crosstalk between the two traces.

### 4b. Time constant, bus capacitance, and supported mode

An RC charge from V_low to V_high crosses 10 % at t = τ·ln(10/9) and 90 % at t = τ·ln 10.
So t_rise(10–90) = τ·ln 9 = 2.197 τ.

| | Value |
|---|---|
| τ from the rise time: 925 ns / ln 9 | **421 ns** |
| τ from a log-linear fit of the samples between 10 % and 90 % | 431 ns |
| Pull-up on SDA_0 (schematic: R6) | 10.00 kΩ |
| **Bus capacitance C = τ/R** | **≈ 42–43 pF** |

The two τ values agree within 2.5 %, which confirms the rise is a single RC charge.
The 42 pF includes the 10× scope probe, which typically adds about 10–15 pF (check the probe's datasheet). The bus alone is therefore roughly 30 pF.

**Which I2C mode the board can support:**
- **Standard mode (100 kHz, t_r ≤ 1000 ns):** yes, but only just. 925 ns leaves 75 ns of margin, and that is with the probe attached.
- **Fast mode (400 kHz, t_r ≤ 300 ns):** no. The rise is 3× too slow.

**What the ADT7420 datasheet allows (Table 2, I2C timing):**
- SCL frequency up to **400 kHz**.
- SDA/SCL rise time t_R up to **0.3 µs**.
- Bus capacitance C_B up to **400 pF**.

The sensor itself is a fast-mode part, and the 42 pF bus is far below its 400 pF limit.
**So the real limit is the board's pull-up, not the sensor.** 10 kΩ × 42 pF gives τ = 0.42 µs and a 0.93 µs rise, which only standard mode tolerates.
For 400 kHz, the pull-up would have to be below about 300 ns / (2.197 × 42 pF) ≈ 3.2 kΩ, for example 2.2 kΩ.
Our design runs SCL at about 100 kHz (four 2.48 µs FSM states per bit), so it is inside what the bus can do.

---

## Question 4c: random noise, interference and bias

| Type | Example from my data | What to do about it |
|---|---|---|
| **Random noise** | ADT7420 conversion noise, std 0.018 °C on independent readings 300 ms apart (M1b), and 0.015–0.021 °C after removing drift at every M1a level | Average more **independent** readings, spaced at least one conversion (240 ms) apart; sigma falls as 1/sqrt(N). This is the only one of the three that more readings help. |
| **Interference** | 60 Hz mains pickup across the resistor in M2a: about 1 mV peak-to-peak, repeating every 16.7 ms, giving a 200 µV std at NPLC 0.02. It dropped to 26 µV at NPLC 1. On the scope (M2b) the same 60 Hz line was 0.21 mV peak with the ground spring and 4.8 mV peak with the long ground loop, 23× larger. | Remove it at the source: shorten the probe ground loop, or integrate over a whole number of mains cycles (NPLC ≥ 1). Averaging asynchronous samples is a slow, poor fix; averaging samples locked to the mains phase does nothing. |
| **Bias** | (1) Settling bias in M1a: readings taken before the board reaches equilibrium read low. With a 90 s wait, about 16 % of each step's rise was still to come and the temperature was still rising 0.15–0.24 °C/min. (2) The power axis: the supply's \|V×I\| includes current in a parallel path on the resistor board. The supply sees about 37 Ω at 2 V, while the resistor measures 46.7 Ω on the DMM, so the power axis overstates the power in the resistor. (3) In M2c, the 10× probe on SDA adds roughly 10–15 pF, so the 42 pF bus capacitance and the 925 ns rise read higher than the bus alone. | Taking more readings never removes a bias, because every reading carries it. For (1), wait long enough, about 2–3 slow time constants (180 s); see the M1a notes. For (2), calibrate: measure the resistor's own voltage with the DMM and use V²/R, or remove the parallel path. For (3), subtract the probe's specified capacitance, or use a lower-capacitance active probe. |

---

## M1a notes (not asked in the post lab; kept for the demo's "justify your wait" question)

Settling time chosen from my own data. I fit a two-time-constant step response, T = T0 + Σ ΔP·[a(1−e^(−t/τ1)) + b(1−e^(−t/τ2))], to the whole 607 s temperature record of the 90 s run (`results/wait_90s_attempt/`):

- fast part: τ1 ≈ 10 s, a ≈ 9.7 °C/W (resistor to sensor)
- slow part: τ2 ≈ 80 s, b ≈ 9.1 °C/W (whole board warming)
- fit error 0.025 °C rms, about the sensor noise. Forcing τ2 to 200 s or more makes the fit clearly worse.

For one 0.07 W step, the part of the rise still to come is 0.21 °C after 90 s, 0.07 °C after 180 s and 0.03 °C after 240 s.
At 180 s, the drift during the 30 s of readings is about 0.02 °C, about the sensor's own std. That is why 180 s was chosen: a longer wait gains less than one LSB.

M1a final run (180 s wait, 1057 s total, no safety trips; `results/m1a_summary.csv`,
figures `m1a_mean_temperature_vs_power.png` and `m1a_std_vs_mean_temperature.png`):

| Level | Wait | Measured P | Mean T | Std T | Std with linear drift removed | Drift during the 100 readings | Last-10 − first-10 mean |
|---|---|---|---|---|---|---|---|
| 1 | 180.0 s | 0.0000 W | 29.691 °C | 0.0152 °C | 0.0153 °C | +0.001 °C/min | −0.002 °C |
| 2 | 180.0 s | 0.0690 W | 30.963 °C | 0.0196 °C | 0.0187 °C | +0.044 °C/min | +0.005 °C |
| 3 | 180.0 s | 0.1411 W | 32.268 °C | 0.0229 °C | 0.0194 °C | +0.086 °C/min | +0.035 °C |
| 4 | 180.0 s | 0.2118 W | 33.632 °C | 0.0261 °C | 0.0207 °C | +0.111 °C/min | +0.062 °C |
| 5 | 180.0 s | 0.2827 W | 34.783 °C | 0.0212 °C | 0.0208 °C | −0.031 °C/min | −0.011 °C |

- Mean temperature vs power is linear: the fit gives 18.1 °C/W (sensor temperature rise per watt of supply output),
  and every point is within 0.08 °C of the line.
- The drift during collection fell from 0.15–0.24 °C/min with the 90 s wait to between −0.03 and +0.11 °C/min.
  The 100-reading std is now 2.0–3.4 LSB, and removing the remaining linear drift brings every level to 0.015–0.021 °C.
  That is the sensor noise found in M1b (0.018 °C), so the std vs temperature plot shows no real dependence on temperature.
  Its upward slope at levels 2–4 is the small remaining drift, at most 0.06 °C over the 30 s of readings.
- Level 1 started at 29.9 °C because the board was still warm from the previous run. It had cooled to a steady
  29.69 °C (+0.001 °C/min) by the time the readings started.
