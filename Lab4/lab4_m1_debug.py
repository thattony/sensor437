# -*- coding: utf-8 -*-

import pyvisa as visa

rm = visa.ResourceManager()
devices = rm.list_resources()

power_supply = None

# Find the E3631A
for dev in devices:
    try:
        inst = rm.open_resource(dev)
        idn = inst.query("*IDN?")

        if "E3631A" in idn:
            power_supply = inst
            print("Found:", idn.strip())
            break

        inst.close()

    except Exception:
        pass

if power_supply is None:
    raise RuntimeError("E3631A not found")

try:
    print("\nErrors before *CLS:")

    # Read all queued errors
    while True:
        err = power_supply.query("SYSTem:ERRor?").strip()
        print(err)

        if err.startswith("+0") or "No error" in err:
            break

    print("\nClearing status/errors with *CLS...")
    power_supply.write("*CLS")

    print("Error queue after *CLS:")
    print(power_supply.query("SYSTem:ERRor?").strip())

finally:
    try:
        power_supply.write("OUTPUT OFF")
        print("\nOutput OFF")
    except Exception as e:
        print("Could not turn output off:", e)

    power_supply.close()
    print("VISA session closed")