RESET_ENABLED = True
# -*- coding: utf-8 -*-
"""
ECE 437 - Lab 2 example
Send two values to the FPGA over WireIn, read the sum and the difference back
over WireOut, and print them.
"""

# %%
# Import the libraries needed to run this code
import time     # time related library
import sys      # system related library
import os
import msvcrt

ok_loc = 'C:\\Program Files\\Opal Kelly\\FrontPanelUSB\\API\\Python\\x64'
ok_dll_loc = r'C:\Program Files\Opal Kelly\FrontPanelUSB\API\lib\x64'
sys.path.append(ok_loc)   # add the patqqh of the OK library
os.add_dll_directory(ok_dll_loc)
import ok                 # OpalKelly library

# %%
# Define the FrontPanel device, open USB communication, load the bit file
dev = ok.okCFrontPanel()
SerialStatus = dev.OpenBySerial("")
ConfigStatus = dev.ConfigureFPGA(r"lab2_m2.bit")

# Always check both. Python will not tell you if either one failed.
print("----------------------------------------------------")
if SerialStatus == 0:
    print("FrontPanel host interface was successfully initialized.")
else:
    print("FrontPanel host interface not detected. Error code: " + str(int(SerialStatus)))
    print("Exiting the program.")
    sys.exit()

if ConfigStatus == 0:
    print("Your bit file is successfully loaded in the FPGA.")
else:
    print("Your bit file did not load. Error code: " + str(int(ConfigStatus)))
    print("Exiting the program.")
    sys.exit()
print("----------------------------------------------------")

# %%
# Send two values to the FPGA using WireIn
variable_1 = 50
variable_2 = 14
print("Variable 1 is initialized to " + str(int(variable_1)))
print("Variable 2 is initialized to " + str(int(variable_2)))

mode = 0
divide = 20000000
reset = 0

dev.SetWireInValue(0x00, variable_1)   # variable_1 goes to address 0x00
dev.SetWireInValue(0x01, variable_2)   # variable_2 goes to address 0x01
dev.SetWireInValue(0x02, mode)   
dev.SetWireInValue(0x03, divide)   
dev.SetWireInValue(0x04, reset)
dev.UpdateWireIns()                    # nothing is sent until this runs



# %%
# Read the results back.
#
# The sum is computed by an assign statement, so it is ready immediately.
# The difference is computed in a register clocked by slow_clk (10 Hz), so it
# is not ready until the next rising edge, which can be up to 100 ms away.
# That is what this sleep is waiting for. Try deleting it.
time.sleep(0.5)

dev.UpdateWireOuts()                   # fetch a snapshot of all 32 WireOuts
result_sum        = dev.GetWireOutValue(0x20)
result_difference = dev.GetWireOutValue(0x21)
counter = dev.GetWireOutValue(0x22)

print("The sum of the two numbers is " + str(int(result_sum)))
print("The difference between the two numbers is " + str(int(result_difference)))

print("----------------------------------------------------")
print("Press 0, 1, 2, or 3 to change mode.")
print("Press c to change the clock divider.")
print("Press q to quit.")
print("----------------------------------------------------")

try:
    while True:
        if msvcrt.kbhit():
            key = msvcrt.getwch()

            # Modes
            if key in ('0', '1', '2', '3'):

                mode = int(key)
                dev.SetWireInValue(0x02, mode)
                dev.UpdateWireIns()

                print("Mode changed to", mode)

            # Clock Divide
            elif key.lower() == 'c':
                divide = int(input("Enter new clock divider: "))

                if divide > 0:
                    dev.SetWireInValue(0x03, divide)
                    dev.UpdateWireIns()

                    print("Divider changed to", divide)

            # quit
            elif key.lower() == 'q':
                print("Exiting...")
                break


        # Read counter
        dev.UpdateWireOuts()
        counter = dev.GetWireOutValue(0x22)

        # Reset
        if RESET_ENABLED and counter >= 100:
            print("Counter reached 100. Sending reset.")
            dev.SetWireInValue(0x04, 1)
            dev.UpdateWireIns()

            slow_clk_period = (
                2 * divide / 200_000_000
            )

            time.sleep(slow_clk_period + 0.05)
            dev.SetWireInValue(0x04, 0)
            dev.UpdateWireIns()
            print("Reset signal returned to 0.")

        print("Counter =", counter)
        time.sleep(0.1)

finally:
    dev.Close()