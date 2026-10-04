# datasheet/

Component datasheets, so prompts can point at them by file name, page, figure or table
("implement the multi-byte read of Table 14, page 16 of `datasheet/HTS221.pdf`").
`docs/SENSOR_BOARD.md` summarises addresses, registers and timing limits **with page
references into these files**; the datasheet is authoritative whenever they disagree.

| File | Part | Pages |
|---|---|---|
| `Sensor_Board_Schematic.pdf` | ECE 437 Sensor Board schematic (UIUC drawing 100134, 2017) - source of truth for wiring | 1 |
| `Sensor_Board_Layout.PNG` | board layout / silkscreen - where each part and test point is | - |
| `ADT7420.pdf` | Analog Devices ADT7420 temperature sensor (I2C bus 0, 0x48) | - |
| `HTS221.pdf` | ST HTS221 humidity + temperature (I2C bus 0, 0x5F) | 34 |
| `LPS35HW.pdf` | ST LPS35HW pressure + temperature (I2C bus 0, 0x5C) | 48 |
| `LSM303DLHC.pdf` | ST LSM303DLHC accelerometer 0x19 + magnetometer 0x1E (I2C bus 1) | 42 |
| `AD7156.pdf` | Analog Devices AD7156 capacitance-to-digital converter (I2C bus 1, 0x48) | 29 |
| `CMV300.pdf` | CMOSIS **CMV300** image sensor, datasheet v2.1 (SPI control, 10-bit parallel video). The schematic, XDC and HDL spell it `CVM300` - same part. | 50 |
| `FrontPanel-UM.pdf` | Opal Kelly FrontPanel user manual - WireIn/WireOut/Trigger/Pipe semantics | - |
