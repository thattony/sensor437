# Lab 9 - the frozen contract

This page is self-contained: the three port lists you implement against, the
complete endpoint map, and every named check in the testbench. It is written
to be pasted into a prompt.

## 1. `hdl/CMV300_Ctrl.v` - port list (FROZEN)

| Signal | Dir | Width | Clock | Meaning |
|---|---|---|---|---|
| `clk` | in | 1 | - | okClk, 100.8 MHz; everything in this module runs on it |
| `cam_enable` | in | 1 | okClk | level: 1 = bring the sensor up and keep it up; 0 = take it down (reset first, then stop the clock) |
| `cam_reset` | in | 1 | okClk | one-clock pulse: full reset sequence, then back up if `cam_enable` is still 1 |
| `clk_locked` | in | 1 | okClk | the MMCM producing the 25 MHz CLK_IN is locked; treat 0 like `cam_enable` = 0 |
| `frame_req_in` | in | 1 | okClk | trigger for one frame sequence; may arrive as a pulse OR a held level - edge-detect it |
| `t_exp_in` | in | 1 | okClk | trigger for a T_EXP1 pulse; same rules as `frame_req_in` |
| `clk_en` | out | 1 | okClk | 1 = CLK_IN runs (gates the ODDR in the wrapper). Clock BEFORE reset release; clock still on WHILE resetting |
| `sys_res_n` | out | 1 | okClk | CVM300_SYS_RES_N. Release >= 1 us after the clock starts; hold low >= 1 us (clock running) on the way down |
| `frame_req` | out | 1 | okClk | CVM300_FRAME_REQ pin: ONE pulse of >= 4 CLK_IN periods (>= 160 ns) per trigger, only while ready |
| `t_exp1` | out | 1 | okClk | CVM300_T_EXP1 pin: same pulse rules |
| `sensor_ready` | out | 1 | okClk | 1 = start-up complete; SPI traffic is allowed only now |
| `state` | out | 4 | okClk | your state register, on WireOut 0x24 [27:24]: 0 OFF, 1 CLK_ON, 2 RES_WAIT, 3 READY, 4 RES_LOW |

Parameters (keep them): `T_WAIT` (cycles for each >= 1 us step; suggested 2016
= 20 us) and `T_REQ` (FRAME_REQ pulse width in okClk cycles; suggested 32).

## 2. `hdl/CMV300_Spi.v` - port list (FROZEN)

| Signal | Dir | Width | Clock | Meaning |
|---|---|---|---|---|
| `clk` | in | 1 | - | okClk, 100.8 MHz |
| `rst` | in | 1 | okClk | synchronous reset: abort, bus to idle |
| `go` | in | 1 | okClk | one-clock pulse: run one transfer (ignore while `busy`) |
| `wr` | in | 1 | okClk | 1 = write, 0 = read (the frame's control bit) |
| `addr` | in | 7 | okClk | register address of the first frame |
| `wr_data` | in | 8 | okClk | data byte (a burst write repeats it) |
| `burst` | in | 2 | okClk | EXTRA frames back to back with SPI_EN held high, addresses addr, addr+1, ... |
| `busy` | out | 1 | okClk | 1 while the transfer runs |
| `done` | out | 1 | okClk | one-clock pulse when it completes |
| `rd_data` | out | 32 | okClk | read byte i of the burst in [8i+7:8i] |
| `SPI_CLK` | out | 1 | okClk | idles LOW; <= 10 MHz (HALF = 5 -> 10.08 MHz) |
| `SPI_EN` | out | 1 | okClk | high frames the transfer: half a period before the first rising edge until one period after the last |
| `SPI_IN` | out | 1 | okClk | MOSI: change on FALLING SPI_CLK edges; MSB first: control bit, addr[6:0], data[7:0] |
| `SPI_OUT` | in | 1 | - | MISO: the sensor launches on falling edges - sample on RISING edges |

## 3. `hdl/CMV300_Counters.v` - port list (FROZEN)

| Signal | Dir | Width | Clock | Meaning |
|---|---|---|---|---|
| `clk_cmv` | in | 1 | - | the sensor's CLK_OUT; sample D/LVAL/DVAL on its FALLING edge |
| `d` | in | 10 | CLK_OUT | pixel data |
| `lval` | in | 1 | CLK_OUT | line valid |
| `dval` | in | 1 | CLK_OUT | data valid - study Figure 26 before assuming a row's shape |
| `clk` | in | 1 | - | okClk: the four outputs below are read by the host in this domain |
| `rst` | in | 1 | okClk | synchronous reset |
| `clear` | in | 1 | okClk | one-clock pulse: `lval_edges` to 0 (synchronise it into the CLK_OUT domain) |
| `lval_edges` | out | 32 | okClk | cumulative LVAL rising edges |
| `last_row_pixels` | out | 16 | okClk | DVAL-high CLK_OUT cycles during the last completed LVAL-high period (648) |
| `last_frame_rows` | out | 16 | okClk | LVAL rising edges of the last completed frame (488); a frame ends when LVAL stays low `GAP` cycles |
| `or_mask` | out | 10 | okClk | bitwise OR of every valid pixel of the last frame |
| `and_mask` | out | 10 | okClk | bitwise AND of every valid pixel of the last frame |

Parameter: `GAP` = LVAL-low CLK_OUT cycles that end a frame (keep 64: between
rows LVAL is low for ONE cycle, between frames for thousands). Hand the values
to okClk at a frame end with a toggle synchroniser so the host never reads a
torn value.

## 4. Endpoint map (complete)

| Endpoint | Meaning |
|---|---|
| WireIn `0x00` | bit0 I2C start (edge), bit1 reset (level), [11:8] sensor-board LEDs |
| WireIn `0x01`-`0x03` | I2C param1..3 - exactly the I2C lab |
| WireIn `0x04` | [0] cam_enable |
| WireIn `0x05` | [7:0] SPI write data, [14:8] register address, [15] 1 = write / 0 = read, [17:16] burst |
| WireOut `0x20`-`0x23` | I2C results / status - exactly the I2C lab |
| WireOut `0x24` | [0] clk_locked, [1] sensor_ready, [3] spi_busy, [10] FRAME_REQ pin (live), [11] cam_enable echo, [23:16] SPI read byte 0, [27:24] sequencer state |
| WireOut `0x25` | [15:0] frame counter (meters), [31:16] LVAL edge counter low 16 bits (your counters) |
| WireOut `0x26` | [15:0] DVAL-high cycles in the last row (648), [31:16] rows in the last frame (488) |
| WireOut `0x27` | CLK_OUT cycles per 1.000 ms (25,000 +/- 16 at 25 MHz) |
| WireOut `0x28` | SPI read data, byte i of a burst in [8i+7:8i] |
| WireOut `0x29` | frame-to-frame period in us (needs >= 2 frames in one request) |
| WireOut `0x2A` | [9:0] OR of every pixel of the last frame, [25:16] AND |
| WireOut `0x2B` | FRAME_REQ -> first LVAL in us (the exposure + FOT silence of a single frame) |
| WireOut `0x2C` | first LVAL rise -> last LVAL fall in us (the readout) |
| WireOut `0x3F` | design ID `0xEC437901` |
| TriggerIn `0x40` | I2C start / reset |
| TriggerIn `0x41` | [0] spi_go, [1] frame_req, [4] cam_reset, [5] clear counters, [8] t_exp1 |
| TriggerOut `0x60` | I2C done |
| TriggerOut `0x61` | [0] spi_done, [1] frame_done |

## 5. The testbench's named checks (`sim/tb_lab9.v`)

Every check prints `SCEN s CHECK n @time PASS/FAIL name: expected E actual A`.

| Scen | Checks | What is proven |
|---|---|---|
| 1 | 1-5 | ready/SYS_RES_N/CLK_IN after cam_enable, zero model violations, state = 3 |
| 1 | 6 | the model CATCHES a deliberately forced SPI access during reset |
| 2 | 1-10 | the datasheet defaults read back (regs 1, 2, 42, 61, 63, 77, 81, 83, 85, 99) |
| 2 | 11-12 | burst reads: Number_lines = 488 from regs 1/2; temperature 0x1464 from regs 78/79 |
| 3 | 1-2 | write 100 to reg 1, read it back; reset restores 232 |
| 4 | 1 | CLK_OUT meter = 0 before reg 69 = 9 |
| 5 | 1 | meter ~36,700/ms with the default PLL band (reg 83 = 155) |
| 6 | 1 | meter 25,000 +/- 16 with reg 83 = 251 |
| 7 | 1-3 | ONE frame: 488 LVAL edges, 648 DVAL cycles/row, 488 rows |
| 7 | 4-5 | sequential regime: FRAME_REQ->first LVAL 6.55 ms +/- 1%, readout 12.69 ms +/- 1% |
| 8 | 1-2 | pipelined regime (Number_frames = 4): period 12.88 ms +/- 1% AND measurably shorter than the 19.24 ms of scenario 7 |
| 9 | 1-5 | exposure sweep 1/6/12/20/40 ms: period = FOT + max(readout, exposure), +/- 1% |
| 9 | 6-7 | flat below the knee (period(12 ms) = period(1 ms)); the knee itself (Exp_time ~= 976) |
| 10 | 1 | 648 valid pixels per row - 649 means the mid-row INVALID slot (DVAL low after pixel 324) was counted; the failure message names it |
| 11 | 1 | Number_frames = 0 -> no frame |
| 12 | 1 | a LEVEL forced onto the sensor's FRAME_REQ pin free-runs it |
| 12 | 2-4 | a held `frame_req_in` level -> exactly ONE pulse, >= 160 ns wide, one frame |
| 13 | 1-2 | OR/AND of a full ramp frame = 0x3FF / 0x000 |
