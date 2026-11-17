# Lab 10 — how to run it

## What is already done

| Item | Where | State |
|---|---|---|
| Capture module (the packer) | `hdl/CMV300_Capture.v` | written |
| Your Lab 9 modules | `hdl/CMV300_Ctrl.v`, `hdl/CMV300_Spi.v` | copied from `Lab9/hdl`; Ctrl has one added line, `req_cnt` (only a name the testbench prints) |
| Your I2C master | `hdl/I2C_Transmit.v` | copied from `Lab9/hdl` (replaced the stub) |
| Simulation | `results/tb_lab10_sim.log` | `RESULT: ALL PASS`, 99 checks |
| Bitstream | `bitfile/SensorBoard_Top.bit` | built 2026-11-17, Vivado 2024.1 on EWS Linux; timing met (WNS +0.208 ns, WHS +0.010 ns); I2C pads IOBUF/OBUFT (your real I2C master) |
| Utilization | `build/post_route_utilization.rpt` | Block RAM 87.5 of 105 tiles (83.3%): snapshot buffer 62, pipe FIFO 16, pixel FIFO 5.5, FrontPanel 2, write DMA 2. LUTs 22.6%, the DDR3 controller is 7,777 of 10,677 |
| Host script | `python/lab10_host.py` | written, not yet run on the board |
| Upload copies | `submit/lab10_CMV300_Capture.v`, `submit/lab10_host.py` | ready |

`python/sensor_board.py` and `python/fp_api.py` are your Lab 9 files. `sensor_board.py` gained the two pipe reads Lab 10 needs; `fp_api.py` keeps your lab-PC DLL fix. `opalkelly/ip_repos` is the complete FrontPanel IP (from Lab 6; the Lab 9 copy is missing `gateware/common`).

## At the lab PC

1. Plug the XEM7310 into a **USB 3.0** port (the script warns if it links at USB 2.0).
2. Open a terminal in `U:\ece437\Lab10\python`.
3. If needed: `pip install numpy matplotlib`.
4. Point the camera at something with dark and bright parts.

Every command loads `..\bitfile\SensorBoard_Top.bit` and saves to `..\results\`.

| Step | Command | Time |
|---|---|---|
| Milestone 1 | `python lab10_host.py m1` | ~10 s |
| Milestone 2 | `python lab10_host.py m2` | ~10 s |
| Milestone 3a | `python lab10_host.py m3a` | ~10 s |
| Milestone 3b | `python lab10_host.py m3b` | ~1 min |
| Milestone 3c | `python lab10_host.py m3c` | ~10 s |
| Everything | `python lab10_host.py all` | ~2 min |
| Redraw the plot | `python lab10_host.py plot` | no board |

Options: `--rates 20,40,60,75` (m3b host rates, fps), `--frames 100`, `--exp-short 1 --exp-long 10` (ms).

Each step prints PASS / FAIL lines. If m1's masks say CHECK, give the scene more contrast; if the exposure says CHECK, raise `--exp-long`.

## What each step measures (the TA checks)

- **m1**: 5 test-pattern frames (244 rows) through your packer, every pixel checked → mismatched-pixel count. A real 244-row image through the snapshot BRAM → `lab10_m1_first_light.png`. Geometry 648 × 244, OR mask 0x3FF, AND mask 0. Mean DN at a short and a long exposure.
- **m2**: 30+ full 648 × 488 test-pattern frames through DDR3, every pixel exact, no generator frame skipped, no sequence gap. Then 30+ real frames in mode A: sequence numbers step by one, 0 dropped.
- **m3a**: mode A vs free-run: delivered fps, sensor period (FPGA-measured), drops.
- **m3b**: free-run with the host slowed to each rate: delivered fps, drop count, sequence numbers, drop fraction vs the pre lab's `1 − h/s` (s = 77.6 fps) → `lab10_m3b_drop_fraction.png`. Then the test pattern in free-run at the slowest rate: lost frames show as sequence gaps while every received pixel is exact.
- **m3c**: host as fast as it can: how long it waits for a frame, how full the ring gets, USB time per frame, and streaming-mode MB/s → names the limit.

## Post lab: where each answer comes from

| Question | Source |
|---|---|
| 1a | `lab10_m1_first_light.png`, `lab10_m1.json` (test-pattern mismatches) |
| 2a | `build/post_route_utilization.rpt`, Block RAM Tile line |
| 3b | 4 px × 100 MHz / 2 clk = 200 Mpx/s vs 25 Mpx/s |
| 4a | `lab10_m3a.json` |
| 4b | `lab10_m3b_drop_fraction.png`, `lab10_m3b.json` |
| 4c | `lab10_m3b.json` (integrity), `lab10_m3c.json` |
| 5a | 3 frames × the free-run period from `lab10_m3a.json` |

## Uploads

Milestones: `submit/lab10_CMV300_Capture.v` and `submit/lab10_host.py`. The Lab 10 overview page calls the second one `lab10_python.py`; the Milestones page says `lab10_host.py`. Check which one Canvas wants.

## Rebuilding (only if you change HDL)

- EWS Linux: `module load xilinx/2024.1`, then `./build.sh` in a copy of this folder outside U: (a build over the network mount is slow).
- Lab PC: `python build.py` with Vivado on PATH.
- Simulation first: `python build.py --sim`, look for `RESULT: ALL PASS`.
