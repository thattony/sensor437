# Lab 10 Demo

**Before the TA arrives:** upload `submit/lab10_CMV300_Capture.v` and `submit/lab10_host.py` to Canvas → Lab 10 – Milestones.

**Setup:** board on a USB 3.0 port, camera pointed at something with dark and bright areas.

```
cd U:\ece437\lab10\python
```

## 1. Run and show

| Run | Show |
|---|---|
| `python lab10_host.py m1` | `TEST PATTERN: 0 mismatched → PASS`. Open `results\lab10_m1_first_light.png`: no shear, 648 × 244, OR = 0x3FF, AND = 0 |
| `python lab10_host.py m2` | 100 full 648 × 488 frames through DDR3: **0 mismatched, 0 gaps**; real frames 0 dropped |
| `python lab10_host.py m3a` | Mode A **39 fps**, 0 dropped; free-run **78 fps** |
| `python lab10_host.py m3b` | Drops appear as **sequence gaps**, received frames **0 mismatched**. Open `results\lab10_m3b_drop_fraction.png` |
| `python lab10_host.py m3c` | Limit = **the sensor** |

## 2. TA questions and answers

**What does your packer do at the start of a row?**
It is held at position 0 while LVAL is low. It is not reset on LVAL rising, because that would swallow the first pixel and shear the image by 1 more pixel per row.

**Are your Milestone 1 and Milestone 2 modules the same?**
Yes, byte-identical: one file and one bitstream, never edited.

**Block RAM used vs a full frame?**
The design uses 87.5 of 105 tiles (83.3 %). A full 16-bit frame would need 130.7 %, so it goes to DDR3.

**Why is mode A slower than free-run?**
Mode A takes one frame at a time: exposure, then readout, 25.6 ms. Free-run overlaps the next exposure with the current readout, 12.88 ms.

**Why are dropped frames whole, not damaged?**
Each slot goes FREE → WRITING → READY → READING. A frame with no free slot is discarded whole, but it still uses up a sequence number, so the host sees a gap.

**What limits your best rate? Prove it.**
The sensor. The host waits 72 % of the time and the ring never holds more than 1 frame. USB needs only 2 ms per frame, about 486 fps of capacity.

## 3. If something shows CHECK
- Masks: add contrast to the scene.
- Exposure: rerun `python lab10_host.py m1 --exp-long 20`.
