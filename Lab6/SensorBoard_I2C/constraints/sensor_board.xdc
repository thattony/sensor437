############################################################################
# sensor_board.xdc - ECE 437 Sensor Board pins that xem7310_v1.xdc leaves
# commented out, plus input pull-ups for the push buttons.
#
# xem7310_v1.xdc (unmodified OpalKelly/UIUC file) already maps:
#   I2C_SCL_0 H3, I2C_SDA_0 G3, ADT7420_A0 G4, ADT7420_A1 J4,
#   I2C_SDA_1 E2, I2C_SCL_1 D2, AD7156_OUT1 D1, AD7156_OUT2 E1,
#   CVM300_* (SPI, clocks, FRAME_REQ, D[9:0], Line/Data_valid),
#   button[3:0], s_LED[3:0], led[7:0], sys_clkp/n, okUH/okHU/okUHU/okAA.
# Ports listed in that file but absent from the design only produce benign
# "[Common 17-55] set_property expects at least one object" warnings.
############################################################################

# ---- I2C bus 0 side-band --------------------------------------------------
# MC2-39
set_property PACKAGE_PIN J2 [get_ports {LPS35_INT_DRDY}]
set_property IOSTANDARD LVCMOS33 [get_ports {LPS35_INT_DRDY}]
# MC2-41
set_property PACKAGE_PIN K1 [get_ports {LPS35_CS}]
set_property IOSTANDARD LVCMOS33 [get_ports {LPS35_CS}]
# MC2-43
set_property PACKAGE_PIN J1 [get_ports {LPS35_SDO}]
set_property IOSTANDARD LVCMOS33 [get_ports {LPS35_SDO}]
# MC2-53
set_property PACKAGE_PIN F3 [get_ports {HTS221_SPIenable}]
set_property IOSTANDARD LVCMOS33 [get_ports {HTS221_SPIenable}]
# MC2-57
set_property PACKAGE_PIN E3 [get_ports {HTS221_DRDY}]
set_property IOSTANDARD LVCMOS33 [get_ports {HTS221_DRDY}]
# MC2-59
set_property PACKAGE_PIN B1 [get_ports {ADT7420_CT}]
set_property IOSTANDARD LVCMOS33 [get_ports {ADT7420_CT}]
# MC2-61
set_property PACKAGE_PIN A1 [get_ports {ADT7420_INT}]
set_property IOSTANDARD LVCMOS33 [get_ports {ADT7420_INT}]

# ---- I2C bus 1 side-band --------------------------------------------------
# MC2-42
set_property PACKAGE_PIN J5 [get_ports {LSM303_DRDY}]
set_property IOSTANDARD LVCMOS33 [get_ports {LSM303_DRDY}]
# MC2-44
set_property PACKAGE_PIN H5 [get_ports {LSM303_INT1}]
set_property IOSTANDARD LVCMOS33 [get_ports {LSM303_INT1}]
# MC2-46
set_property PACKAGE_PIN H2 [get_ports {LSM303_INT2}]
set_property IOSTANDARD LVCMOS33 [get_ports {LSM303_INT2}]

# ---- CVM300 exposure trigger inputs --------------------------------------
# MC2-25
set_property PACKAGE_PIN L4 [get_ports {CVM300_T_EXP1}]
set_property IOSTANDARD LVCMOS33 [get_ports {CVM300_T_EXP1}]
# MC2-27
set_property PACKAGE_PIN M6 [get_ports {CVM300_T_EXP2}]
set_property IOSTANDARD LVCMOS33 [get_ports {CVM300_T_EXP2}]

# ---- Push buttons: weak pull-up so an unpressed button reads 1 ------------
set_property PULLUP true [get_ports {button[*]}]

# ---- The FSM clock is okClk (mmcm0_clk0 from the FrontPanel IP); it is
# already constrained by the IP. clk200 (sys_clk) is constrained in
# xem7310_v1.xdc and declared asynchronous to okClk there.
