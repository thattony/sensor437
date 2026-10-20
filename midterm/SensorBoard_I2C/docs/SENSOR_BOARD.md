# ECE 437 Sensor Board — hardware reference

Source of truth for wiring: `datasheet/Sensor_Board_Schematic.pdf` (UIUC ECE, drawing
100134, 2017) and `datasheet/Sensor_Board_Layout.PNG`. Pin numbers come from the Opal Kelly
constraints file. Section 3 was checked against the component datasheets in `datasheet/`
(page numbers refer to those PDFs); the datasheet wins whenever this page and it disagree.

## 1. The FPGA module — Opal Kelly XEM7310-A75

| Item | Value |
|---|---|
| FPGA | Xilinx Artix-7 `xc7a75tfgg484-1` |
| Board oscillator | 200 MHz LVDS on `sys_clkp`/`sys_clkn` (W11/W12), constrained as `sys_clk` in `constraints/xem7310_v1.xdc` |
| Host interface | FrontPanel over USB 3.0 (`datasheet/FrontPanel-UM.pdf`); the FrontPanel Subsystem IP supplies `okClk` = **100.8 MHz** — the clock the starter FSM runs on |
| On-module LEDs | `led[7:0]`, **active low**, bank at 1.5 V (`LVCMOS15`) |
| Expansion | two 80-pin BTE-040 connectors MC1 / MC2 — the sensor board plugs onto these; every sensor signal is on MC1/MC2 pins (noted `# MCx-nn` in the XDC) |
| Configuration | over USB from Python (`dev.ConfigureFPGA(bitfile)`) — no JTAG cable needed |

## 2. Sensors on the board

All sensors are powered from +3.3 V (U8, LD1117). The imager has its own
2.2 V / 2.5 V / 3.0 V regulators (U9, U7, U10). Both I2C buses have 10 kΩ
pull-ups to 3.3 V (R4–R7) — lines are **open-drain**: pull low or release, never
drive high.

### I2C bus 0 — `I2C_SCL_0` = H3, `I2C_SDA_0` = G3 (MC2-45 / MC2-47)

| Ref | Part | Function | 7-bit addr | Side-band pins (FPGA pin) | Safe default |
|---|---|---|---|---|---|
| U1 | **ADT7420** (Analog Devices) | temperature, 13/16-bit | `0x48` — set by `ADT7420_A0` (G4) / `ADT7420_A1` (J4); `0x48 + A0 + 2·A1` | `ADT7420_CT` (B1) and `ADT7420_INT` (A1): open-drain alarm outputs, inputs to FPGA | A0 = A1 = 0 |
| U4 | **HTS221** (ST) | relative humidity + temperature | `0x5F` | `HTS221_SPIenable` (F3) — **must be 1 for I2C mode**; `HTS221_DRDY` (E3) input | SPIenable = 1 |
| U6 | **LPS35HW** (ST) | barometric pressure + temperature | `0x5C` when `LPS35_SDO` = 0, `0x5D` when 1 | `LPS35_CS` (K1) — **must be 1 for I2C mode**; `LPS35_SDO` (J1) = SA0 address bit, driven by FPGA; `LPS35_INT_DRDY` (J2) input | CS = 1, SDO = 0 |

### I2C bus 1 — `I2C_SCL_1` = D2, `I2C_SDA_1` = E2 (MC2-51 / MC2-49)

| Ref | Part | Function | 7-bit addr | Side-band pins (FPGA pin) | Safe default |
|---|---|---|---|---|---|
| U5 | **LSM303DLHC** (ST) | 3-axis accelerometer + 3-axis magnetometer (two I2C devices in one package) | accel `0x19`, magnetometer `0x1E` | `LSM303_DRDY` (J5), `LSM303_INT1` (H5), `LSM303_INT2` (H2): inputs | — |
| U2 | **AD7156** (Analog Devices) | 2-channel capacitance-to-digital converter (touch pads NetC49 / NetC50 on the PCB) | `0x48` — same as the ADT7420, which is why it sits on a separate bus | `AD7156_OUT1` (D1), `AD7156_OUT2` (E1): "proximity detected" comparator outputs, inputs to FPGA | — |

### SPI + parallel video — CMV300 image sensor (U3)

CMOSIS **CMV300** (`datasheet/CMV300.pdf`; the schematic, XDC and HDL spell it `CVM300`):
648 × 488 global-shutter CMOS imager, configured over a 4-wire SPI and read out here as
10-bit parallel CMOS data (pin B2 `Enable_LVDS` = GND selects the parallel mode, §4.2 p.22;
the LVDS outputs are not used).

| Signal | FPGA pin | Dir | Notes (CMV300.pdf) |
|---|---|---|---|
| `CVM300_SPI_EN` | N3 | out | high during a transfer: ≥ ½ SPI_CLK before the first bit, 1 SPI_CLK after the last (§3.9.1 p.15) |
| `CVM300_SPI_CLK` | L5 | out | ≤ 40 MHz; sensor samples `SPI_IN` on the **rising** edge and launches `SPI_OUT` on the **falling** edge |
| `CVM300_SPI_IN` | N4 | out | MOSI. Frame = control bit (1 = write, 0 = read) + 7 address bits + 8 data bits, MSB first (Figures 7–9, p.15–16). Burst: keep SPI_EN high |
| `CVM300_SPI_OUT` | P4 | in | MISO, data MSB first after the 8 address/control bits (Figure 9, p.16) |
| `CVM300_CLK_IN` | H4 | out | master clock, **10–40 MHz** per §3.6 p.13 (the pin table on p.43 says 25 MHz max — start ≤ 25 MHz). Start it only after the supplies are stable (§3.7 p.14) |
| `CVM300_CLK_OUT` | K4 | in | pixel clock returned by the sensor — sample `D[9:0]` on it |
| `CVM300_SYS_RES_N` | M5 | out | reset, active low; release before the SPI upload, then FRAME_REQ (§3.7 p.14–15) |
| `CVM300_Enable_LVDS` | P5 | out | 0 = CMOS parallel output |
| `CVM300_FRAME_REQ` | M1 | out | ≥ 1 CLK_IN period high requests the number of frames programmed in registers 55/56 (default 1) (§3.10 p.16) |
| `CVM300_T_EXP1/2` | L4 / M6 | out | external exposure control inputs of the sensor (§3.4 p.12) |
| `CVM300_D[9:0]` | R6 T6 W9 Y9 AA5 AB5 T5 U5 R4 T4 | in | pixel data, 1.8 V signalling |
| `CVM300_Line_valid` / `Data_valid` | U6 / V5 | in | video framing (§4.2) |

On-chip temperature: SPI registers 78/79 read in burst mode (§3.9.2). Test points TP1–TP6
sit on the SPI/control lines next to U3.

### User I/O on the sensor board

| Signal | FPGA pins | Notes |
|---|---|---|
| `button[3:0]` | Y11 Y12 Y14 W14 | SW0–SW3, switch to GND → **pressed = 0** (internal pull-up enabled in `sensor_board.xdc`) |
| `s_LED[3:0]` | U15 V15 T14 T15 | D0–D3 red LEDs through 330 Ω → **1 = on** |
| `PMOD_A*`, `PMOD_B*` | see XDC | two 12-pin PMOD headers J1/J2 (3.3 V, ESD-protected) |
| `USER_33[7:0]`, `USER_25[7:0]`, `USER_SRCC_P/N` | see XDC | user expansion headers JP2/JP3 |
| TP7–TP10 | — | test points on the four I2C lines (silkscreen `SCL_0 SDA_0 SCL_1 SDA_1`) — put a scope here |
| P1–P3, D4 | — | "DUT" diode test fixture (analog lab), not connected to the FPGA |

## 3. Register cheat-sheet (checked against `datasheet/*.pdf`; page numbers point there)

**ADT7420** (`0x48`, SCL ≤ 400 kHz) — `ADT7420.pdf`. Verified on this hardware.
`0x00/0x01` temperature MSB/LSB (13-bit default: `T = signed13(raw16 >> 3) × 0.0625 °C`;
16-bit mode: `signed16(raw16) / 128`), `0x02` status (bit 7 = ¬RDY), `0x03` configuration
(bit 7 = 16-bit mode), `0x0B` ID = **`0xCB`**. Register pointer auto-increments. New
conversion every 240 ms.

**HTS221** (`0x5F`, SCL ≤ 400 kHz — Table 6, p.10) — `HTS221.pdf`.
I2C operation §5.1.1 p.15: address bytes `0xBE` write / `0xBF` read (Table 10);
**bit 7 of the sub-address = auto-increment**; write sequence Table 11, single-byte read
Table 13, multi-byte read Table 14 (p.16). Register map Table 15 p.20. `0x0F` WHO_AM_I =
**`0xBC`** (§7.1 p.21, verified on hardware). `0x10` AV_CONF (Table 16). `0x20` CTRL_REG1
(§7.3 p.22: bit 7 PD must be 1 to leave power-down, bit 2 BDU, ODR[1:0] per Table 17: 00
one-shot, 01 1 Hz, 10 7 Hz, 11 12.5 Hz). `0x21` CTRL_REG2 (bit 7 BOOT, bit 1 heater, bit 0
ONE_SHOT). `0x27` STATUS_REG (§7.6 p.24: bit 1 H_DA, bit 0 T_DA). `0x28/0x29` HUMIDITY_OUT
L/H and `0x2A/0x2B` TEMP_OUT L/H, both s16 (§7.7–7.9 p.25). Calibration (Table 19, §8
p.26–27): `0x30` H0_rH_x2, `0x31` H1_rH_x2, `0x32` T0_degC_x8, `0x33` T1_degC_x8, `0x35`
bits[3:2]/[1:0] = T1/T0 bits 9:8, `0x36/0x37` H0_T0_OUT, `0x3A/0x3B` H1_T0_OUT, `0x3C/0x3D`
T0_OUT, `0x3E/0x3F` T1_OUT; output = linear interpolation between the two calibration points.

**LPS35HW** (`0x5C` with SA0 = 0, `0x5D` with SA0 = 1, SCL ≤ 400 kHz — Table 6, p.10) — `LPS35HW.pdf`.
I2C operation §6.3 p.24: address bytes `0xB8` write / `0xB9` read for SA0 = 0 (Table 10);
sequences Tables 11–14 (p.24–25); auto-increment is **IF_ADD_INC in CTRL_REG2 (0x11), default 1**
(no sub-address bit needed). Register map §8 p.29. `0x0F` WHO_AM_I = **`0xB1`** (Table 17 p.32).
`0x10` CTRL_REG1 (p.33: ODR[6:4] per Table 19: 000 power-down/one-shot, 001 1 Hz, 010 10 Hz,
011 25 Hz, 100 50 Hz, 101 75 Hz; bit 1 BDU). `0x11` CTRL_REG2 (p.34: bit 7 BOOT, bit 4
IF_ADD_INC, bit 2 SWRESET, bit 0 ONE_SHOT). `0x27` STATUS. `0x28/0x29/0x2A` PRESS_OUT XL/L/H:
`hPa = signed24 / 4096` (4096 LSB/hPa, p.8 and p.13); read PRESS_OUT_H last when BDU = 1.
`0x2B/0x2C` TEMP_OUT L/H: `°C = signed16 / 100` (100 LSB/°C, p.8).

**LSM303DLHC** — `LSM303DLHC.pdf`, I2C timing Table 6 p.12 (SCL ≤ 400 kHz).
*Accelerometer* `0x19` (§5.1.2 p.20: address bytes `0x32` write / `0x33` read, Table 14;
**multi-byte reads need sub-address bit 7 set**, Table 15). Register map Table 17 p.22–23.
`0x20` CTRL_REG1_A (§7.1.1 p.24; power-on `0x07` = power-down with X/Y/Z enabled — e.g. `0x57`
= 100 Hz, all axes). `0x23` CTRL_REG4_A (Table 27: bit 7 BDU, bit 6 BLE, FS[5:4] 00 ±2 g …
11 ±16 g, bit 3 HR). `0x27` STATUS_REG_A. `0x28–0x2D` OUT_X/Y/Z L,H (§7.1.9 p.28): 16-bit
two's complement, little-endian, 12-bit resolution (left-justified → `value >> 4`), 1 mg/LSB at
±2 g (p.10). No WHO_AM_I on the accelerometer.
*Magnetometer* `0x1E` (§5.1.3 p.21: `0x3C` write / `0x3D` read, Table 16). `0x00` CRA_REG_M
(ODR, §7.2.1 p.36), `0x01` CRB_REG_M (gain; 1100 LSB/gauss at GN = 001, p.10), `0x02` MR_REG_M
(§7.2.3 p.37: `0x00` = continuous conversion; power-on `0x03` = sleep). `0x03–0x08` OUT
**X_H, X_L, Z_H, Z_L, Y_H, Y_L** (big-endian, X-Z-Y order — Table 17 p.23, §7.2.4–7.2.6 p.38).
`0x09` SR_REG_M. `0x0A/0x0B/0x0C` IRA/IRB/IRC = **`0x48 0x34 0x33`** (Tables 81–83, p.38–39)
— use these as the ID check. `0x31/0x32` TEMP_OUT_H/L_M (8 LSB/°C, 12-bit).

**AD7156** (`0x48` on bus 1, SCL ≤ 400 kHz — Table 2 p.5, timing Figure 2) — `AD7156.pdf`.
Address bytes `0x90` write / `0x91` read (p.23). Register summary Table 5 p.15: `0x00` status
(bit 7 PwrDown, bit 5 OUT2, bit 3 OUT1, bit 2 C1/C2, bit 1 RDY2, bit 0 RDY1), `0x01/0x02` CH1
data H/L, `0x03/0x04` CH2 data H/L (**12-bit result in the 12 MSBs of the 16-bit word**,
`0x0000`–`0xFFF0`, p.17), `0x05–0x08` CH1/CH2 averages, `0x09/0x0A` CH1
sensitivity/timeout (defaults `0x08`/`0x86`), `0x0B` CH1 setup (default `0x0B`), `0x0C–0x0E`
same for CH2, `0x0F` configuration (default **`0x19`** = both channels enabled, continuous
conversion, adaptive threshold — p.20), `0x10` power-down timer, `0x11/0x12` CAPDAC,
`0x13–0x16` serial number, `0x17` chip ID (factory value, **not published** — use the `0x0F`
default as the ID check). Power-on default = converting both channels continuously; OUT1/OUT2
go high when proximity is detected (p.7).

## 4. FrontPanel endpoints of the starter design

| Endpoint | Address | Direction | Meaning (see `hdl/SensorBoard_Top.v`, `python/sensor_board.py`) |
|---|---|---|---|
| WireIn | `0x00` | PC → FPGA | ctrl: bit 0 start (rising edge), bit 1 reset (level), `[31:8]` free (`ctrl_user`) |
| WireIn | `0x01`–`0x03` | PC → FPGA | `param1`…`param3` — meaning defined by your FSM. As shipped: `param1[7:0]` = address byte `{addr7, R/W}`, `param1[8]` = bus (0/1), `param2/3` unused |
| WireOut | `0x20`–`0x22` | FPGA → PC | `result0`…`result2`. As shipped: `result0[0]` = ACK bit (0 = ACK), `[15:8]` = byte sent, `[16]` = bus; `result1` = 0; `result2` = clock cycles busy |
| WireOut | `0x23` | FPGA → PC | status: bit 0 busy, bit 1 done, bit 2 error, `[15:8]` dbg_state, `[31:16]` status_user |
| WireOut | `0x3F` | FPGA → PC | design ID `0xEC437001` |
| TriggerIn | `0x40` | PC → FPGA | bit 0 start pulse, bit 1 reset pulse |
| TriggerOut | `0x60` | FPGA → PC | bit 0 done |
| PipeOut | `0xA0` | FPGA → PC | bulk 32-bit words (`po_read` strobes one word per clock) |
