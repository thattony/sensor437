# INTERFACE.md — the frozen contract

ECE 437 Lab 10 · `SensorBoard_CMV300_Capture`

You write one module, `hdl/CMV300_Capture.v`. Everything downstream is given and will not
change. This file is that boundary. **Paste "The entry" and "Your module's ports" into any
prompt you write.**

---

## Your module's ports

`CMV300_CaptureCore.v` instantiates you as `u_pack`. Everything below is already registered
in the CLK_OUT **rising-edge** domain — the core samples the sensor on the falling edge in
IOB flops and hands the result across, so you do not write a `negedge` block.

```verilog
module CMV300_Capture (
    input  wire        clk,          // CLK_OUT, rising edge
    input  wire [9:0]  pixel,        // the pixel on this cycle
    input  wire        pixel_valid,  // high when `pixel` is real (LVAL and DVAL both high)
    input  wire        line_valid,   // registered LVAL. LOW between rows.
    input  wire        stream_en,    // acquisition gate. Push nothing when low.
    input  wire        frame_end,    // one cycle, at the end of a frame
    output reg         pix_valid,    // one cycle, when an entry is complete
    output reg  [47:0] pix_data,     // the entry
    output reg         pix_eof       // one cycle, the end-of-frame token
);
```

Names and widths are checked by `sim/tb_lab10.v`. Do not rename them.

---

## The entry

**48 bits: four pixels in 12-bit lanes.**

| Bits | Field |
|---|---|
| `[47:36]` | `px3`, the fourth pixel of the group |
| `[35:24]` | `px2` |
| `[23:12]` | `px1` |
| `[11:0]` | `px0`, the **first** pixel of the group |

A parallel-mode pixel is 10 bits and sits in the low bits of its lane: `{2'b00, pixel}`.
The lanes are 12 bits because the same entry format carries a 12-bit front end later, and
widening it afterwards would have meant re-verifying the DMA, the ring and the host format.

The pixel FIFO is **49 bits wide**: bit `[48]` is the end-of-frame token, `[47:0]` is the
entry. The core builds that word from your `pix_data` and `pix_eof`; you drive them separately.

| Quantity | Value |
|---|---|
| Pixels per entry | 4 |
| Pixels per row | 648 |
| **Entries per row** | **162, exactly** — no row ever leaves a partial entry |
| Entries per frame, 488 rows | 79,056 |

---

## Where your entries go

The write DMA pops the FIFO and **packs**: the 48 payload bits of every entry go into a
176-bit accumulator and a 128-bit AXI beat is emitted whenever one is complete. The stored
stream is therefore **packed little-endian 12-bit, two pixels per three bytes, 1.5 bytes per
pixel** — not 16-bit words. At the end-of-frame token the tail is zero-padded up to a whole
number of 1024-byte BTPipe blocks.

| Quantity | Value |
|---|---|
| Bytes per pixel, stored | **1.5** |
| One 648 × 488 frame | **474,336 B** |
| Padded to 1024-byte blocks | **464 blocks** = 475,136 B (800 B of padding) |
| Ring | three slots, 1 MiB stride, sequence-numbered |
| Entry pace through the DMA | one entry per two `ui_clk` (100 MHz) |
| Pixel rate that supports | 4 px ÷ 2 clk × 100 MHz = **200 Mpixel/s** |
| What the parallel port delivers | **25 Mpixel/s** |

The host unpacks with `cmv300.unpack_frame()`; `python/cmv300.py` is the reference for the
byte order.

> An entry of any other width silently breaks the packing and every frame after it. A capture
> module that produces a beautiful image with a one-pixel entry will compile, will pass a
> testbench of its own, and will destroy the frame in memory.

---

## The rule that costs an afternoon

**Hold the packer cleared while `line_valid` is low.**

Clear the 0–3 position counter continuously while `line_valid == 0`, so you enter each row at
position 0. **Do not clear on the rising edge of `line_valid`.** On the test pattern LVAL and
DVAL rise together, so a reset triggered there lands in the same cycle as the first pixel of
the row and swallows it. The image then shears by one further pixel on every row.
`tb_lab10.v` catches this. The image almost does not.

`frame_end` is one cycle, produced by the core after LVAL has been low for 64 clocks. Gate
`pix_eof` with `stream_en` so a frame that was already running when acquisition stopped does
not emit a token.

---

## What you may not change

- The entry layout above, including lane order and the first pixel in `[11:0]`.
- The module name and the port names and widths.
- `hdl/CMV300_CaptureCore.v`, and anything else in `hdl/`.

Milestone 2 requires that your Milestone 1 and Milestone 2 capture modules are
**byte-identical**. If you find yourself editing this module to make the ring path work, the
entry format was wrong, not the ring.
