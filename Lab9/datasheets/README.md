# datasheets/ - the documents this package cites

The module headers cite these by filename; the figure and section numbers in
the HDL comments refer to these exact versions.

## Included

| File | Document | Cited by |
|---|---|---|
| `CMV300.pdf` | CMOSIS CMV300 datasheet v2.1 (start-up 3.7 Fig 4-5, FRAME_REQ 3.8 Fig 6, SPI 3.9 Fig 7-9, video 4.2.1 Fig 26-27, registers section 6) | all three stubs, the sensor model, cmv300.py |
| `Sensor_Board_Schematic.pdf` | UIUC ECE 437 sensor board schematic | pin questions, I2C bus wiring |
| `ADT7420.pdf` | ADI ADT7420 temperature (0x48, bus 0; I2C Figures 14/16/17) | I2C_Transmit.v, temperature.py |
| `HTS221.pdf` | ST HTS221 humidity (0x5F, bus 0) | temperature.py |
| `LPS35HW.pdf` | ST LPS35HW pressure (0x5C, bus 0) | temperature.py |
| `LSM303DLHC.pdf` | ST LSM303DLHC accel/mag (0x19/0x1E, bus 1) | temperature.py |
| `AD7156.pdf` | ADI AD7156 capacitance (0x48, bus 1) | temperature.py |

## Not bundled (later labs; fetch when needed)

- Opal Kelly XEM7310 user manual - docs.opalkelly.com
- Xilinx UG471, 7 Series SelectIO - docs.amd.com
- Xilinx UG586, 7 Series MIG - docs.amd.com
