import sys
import os
import time

ok_loc = r'C:\Program Files\Opal Kelly\FrontPanelUSB\API\Python\x64'
ok_dll_loc = r'C:\Program Files\Opal Kelly\FrontPanelUSB\API\lib\x64'

sys.path.append(ok_loc)
os.add_dll_directory(ok_dll_loc)

import ok

# Create FrontPanel device
dev = ok.okCFrontPanel()

# Open the first available device
error = dev.OpenBySerial("")

if error != ok.okCFrontPanel.NoError:
    print("ERROR: Could not open the FPGA device.")
    sys.exit(1)

print("FrontPanel host interface successfully initialized.")


# Configure FPGA
bitfile = "lab3_m2_top.bit"

error = dev.ConfigureFPGA(bitfile)

if error != ok.okCFrontPanel.NoError:
    print("ERROR: FPGA configuration failed.")
    print("Error code:", error)
    sys.exit(1)

print("FPGA configured successfully.")


# Start with pedestrian request LOW
dev.SetWireInValue(0x00, 0x00, 0x01)
dev.UpdateWireIns()

print("Traffic light running.")
print("Press Enter to send a pedestrian request.")
input()


# Pedestrian request HIGH
dev.SetWireInValue(0x00, 0x01, 0x01)
dev.UpdateWireIns()

print("Pedestrian request sent.")

time.sleep(0.1)


# Pedestrian request LOW
dev.SetWireInValue(0x00, 0x00, 0x01)
dev.UpdateWireIns()

print("Pedestrian request released.")

# NS_GREEN  01001100
# NS_YELLOW 01001010
# EW_GREEN  01100001
# EW_YELLOW 01010001
# PED_GREEN 10001001