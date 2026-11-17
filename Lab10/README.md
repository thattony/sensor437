# ECE 437 Lab 10 — Getting the Pixels Out

You write **one module**: `hdl/CMV300_Capture.v`, the four-pixel packer.
Read `INTERFACE.md` first — the entry format is frozen and the testbench checks it.

## Before anything else: add your own Lab 9 modules

This package does **not** contain them, by design:

    hdl/CMV300_Ctrl.v      your power-up sequencer and FRAME_REQ pulse generator
    hdl/CMV300_Spi.v       your SPI master

Copy your working Lab 9 files into `hdl/`. Until you do, the build stops with
`module CMV300_Ctrl not declared` — that is the expected failure, not a broken package.
If either is not working, fix it **before** your session. A camera that will not
configure cannot produce a frame to capture.

## Build

    python build.py --sim     run sim/tb_lab10.v in xsim (seconds). Do this first.
    python build.py           synthesise and generate the bitstream (20–40 min)

The AXI RAM model stands in for the DDR3, so **the whole path simulates without the
memory controller existing**. Every acceptance number in Milestones 1 and 2 is reachable
in simulation. Only Milestone 3, which measures the real USB link, needs the board.

Budget for at most two bitstream builds: one for the snapshot path, one for the ring.
This design contains a DDR3 controller and a full build is not quick.

## What is here

    hdl/CMV300_Capture.v       YOU WRITE THIS
    hdl/CMV300_CaptureCore.v   given — sampling, counters, masks, snapshot BRAM, test pattern
    hdl/CMV300_WriteDma.v      given — packs entries into 128-bit AXI beats
    hdl/CMV300_ReadDma.v       given
    hdl/CMV300_FrameRing.v     given — three 1 MiB slots, sequence numbers, drop counter
    hdl/CMV300_Subsystem.v     given — the camera, wired to the host endpoints
    hdl/SensorBoard_Top.v      given — top level, FrontPanel, MIG
    hdl/I2C_Transmit.v         your I2C machine from Labs 5 and 6
    sim/tb_lab10.v             the testbench that decides whether you are finished
    sim/cmv300_model.v         behavioural CMV300
    sim/axi_ram_model.v        4 MiB AXI slave standing in for the DDR3
    python/cmv300.py           host API: acquisition, the ring protocol, frame unpacking
    vivado/build.tcl           project + MIG generation
    vivado/mig_xem7310_a75.prj MIG pin and timing settings — do not run the wizard

## What to paste when you ask for help

The testbench's failing line, the entry format and the port list from `INTERFACE.md`.
A model cannot see your Vivado log, your MIG configuration or your board. If the answer
depends on something you cannot paste into the chat, it is a question for the testbench.

## Upload

    lab10_CMV300_Capture.v     your front end
    lab10_host.py              the host code

Both go to **Lab 10 – Milestones** before your demonstration.
