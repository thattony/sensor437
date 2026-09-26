# -*- coding: utf-8 -*-
"""
ECE 437 - Lab 4 milepoint2
Sweep the DC power supply across the test diode circuit and record the
supplied voltage and current at each step.

If an instrument beeps and shows an error, it will ignore everything you send
afterwards. Clear it with:   power_supply.write("*CLS")
"""

import time
import numpy as np
import matplotlib.pyplot as plt
import pyvisa as visa

# Find every instrument on the USB bus and work out which is which.
#
# The match below looks for the MODEL NUMBER inside the *IDN? string rather
# than comparing the whole string. Full-string matching breaks the moment an
# instrument is swapped or its firmware is updated.

device_manager = visa.ResourceManager()
devices = device_manager.list_resources()
number_of_device = len(devices)

power_supply_id       = -1
waveform_generator_id = -1
digital_multimeter_id = -1
oscilloscope_id       = -1

for i in range(0, number_of_device):
    try:
        device_temp = device_manager.open_resource(devices[i])
        idn = device_temp.query("*IDN?")
        print("Instrument on USB port [" + str(i) + "] is " + idn.strip())

        if "E3631A" in idn:
            power_supply_id = i
        elif "33511B" in idn:
            waveform_generator_id = i
        elif "34461A" in idn:
            digital_multimeter_id = i
        elif "MSO-X 3024T" in idn:
            oscilloscope_id = i

        device_temp.close()
    except Exception:
        print("Instrument on USB port [" + str(i) + "] cannot be connected. "
              "It may be powered off, or it may not be an instrument.")

# Open the instruments. Never assume it is there.

if power_supply_id == -1:
    print("Power supply is not powered on or not connected to the PC.")
    raise SystemExit
else:
    print("Power supply is connected to the PC.")
    power_supply = device_manager.open_resource(devices[power_supply_id])

if digital_multimeter_id == -1:
    print("Multimeter is not powered on or not connected to the PC.")
    raise SystemExit
else:
    print("Multimeter is connected to the PC.")
    multimeter = device_manager.open_resource(devices[digital_multimeter_id])

if oscilloscope_id == -1:
    print("Oscilloscope is not powered on or not connected to the PC.")
    raise SystemExit
else:
    print("Oscilloscope is connected to the PC.")
    oscilloscope = device_manager.open_resource(devices[oscilloscope_id])

# Error checking
def check_errors(instrument):
    err = instrument.query("SYSTem:ERRor?").strip()
    if "No error" not in err:
        print(err)
    instrument.write("*CLS")

# Multimeter Configuration
print("Configuring multimeter...")
multimeter.write("CURRent:DC:TERMinals 3")
multimeter.write("CONFigure:CURRent:DC 0.1")

print("Terminal:", multimeter.query("CURRent:DC:TERMinals?"))
print("Range:", multimeter.query("CURRent:DC:RANGe?"))
print("Autorange:", multimeter.query("CURRent:DC:RANGe:AUTO?"))
check_errors(multimeter)
print("Multimeter configuration done.")

# Oscilloscope Configuration
print("Configuring oscilloscope...")
oscilloscope.write(":CHANnel1:DISPlay ON")
oscilloscope.write(":CHANnel2:DISPlay ON")
# oscilloscope.write(":ACQuire:TYPE HRESolution")
oscilloscope.write(":ACQuire:TYPE AVERage")
oscilloscope.write(":ACQuire:COUNt 16")
oscilloscope.write(":TRIGger:SWEep AUTO")
oscilloscope.write(":RUN")

print("Acquire type:", oscilloscope.query(":ACQuire:TYPE?"))
print("Trigger sweep:", oscilloscope.query(":TRIGger:SWEep?"))
check_errors(oscilloscope)
print("Oscilloscope configuration done.")

# Power Supply Configuration
print("Configuring power supply...")
power_supply.write("APPLy P25V, 0.00, 0.03")
check_errors(power_supply)
print("Power supply configuration done.")

# Sweep voltage from 0 to 8 V in steps of 0.1 V
output_voltage    = np.arange(0, 8.1, 0.1)
measured_voltage  = np.array([])
measured_current  = np.array([])

power_supply.write("OUTPUT ON")
print("Begin sweeping...")

for v in output_voltage:
    # APPLy <channel>, <volts>, <current limit in amps>
    power_supply.write("APPLy P25V, %0.2f, 0.03" % v)

    # Let the supply settle before reading. See post lab question 3.
    time.sleep(0.2)

    # Current measured by multimeter
    measured_current = np.append(measured_current, float(multimeter.query("READ?")))

    v_ch1 = float(oscilloscope.query(":MEASure:VAVerage? DISPlay,CHANnel1"))
    time.sleep(0.1)
    v_ch2 = float(oscilloscope.query(":MEASure:VAVerage? DISPlay,CHANnel2"))
    time.sleep(0.1)
    measured_voltage = np.append(measured_voltage, v_ch1-v_ch2)

power_supply.write("OUTPUT OFF")
print("Sweep done...")

# Check errors
print("Checking power supply:")
check_errors(power_supply)
print("Checking multimeter:")
check_errors(multimeter)
print("Checking oscilloscope:")
check_errors(oscilloscope)

# Close all instrument
power_supply.close()
multimeter.close()
oscilloscope.close()

# Plot
plt.figure()
plt.plot(measured_voltage, measured_current)
plt.title("Measured voltage vs. measured supplied current")
plt.xlabel("Measured voltage [V]")
plt.ylabel("Measured current [A]")
plt.grid(True)

plt.show()

