import time
import pyvisa as visa

SCOPE = "USB0::0x2A8D::0x1776::MY54440275::INSTR"

rm = visa.ResourceManager()
scope = rm.open_resource(SCOPE)
scope.timeout = 10000

try:
    print("ID:")
    print(scope.query("*IDN?").strip())

    scope.write("*CLS")

    # Enable both channels
    scope.write(":CHANnel1:DISPlay ON")
    scope.write(":CHANnel2:DISPlay ON")

    # High-resolution acquisition
    scope.write(":ACQuire:TYPE HRESolution")

    # Important for DC / slowly changing signals:
    # force acquisitions even if trigger condition is not met
    scope.write(":TRIGger:SWEep AUTO")

    print("Acquire type:",
          scope.query(":ACQuire:TYPE?").strip())

    print("Trigger sweep:",
          scope.query(":TRIGger:SWEep?").strip())

    print("Error before measurement:",
          scope.query("SYSTem:ERRor?").strip())

    # Start continuous acquisition
    scope.write(":RUN")

    # Allow a few acquisitions to occur
    time.sleep(1.0)

    print("Testing CH1 VAverage...")

    v1 = scope.query(
        ":MEASure:VAVerage? DISPlay,CHANnel1"
    ).strip()

    print("CH1 =", v1)

    print("Error:",
          scope.query("SYSTem:ERRor?").strip())

finally:
    scope.close()
    rm.close()