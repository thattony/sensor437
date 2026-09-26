import pyvisa as visa

rm = visa.ResourceManager()
devices = rm.list_resources()

print("Resources:", devices)

for resource in devices:
    inst = None

    try:
        print("\nOpening:", resource)

        inst = rm.open_resource(resource)
        inst.timeout = 2000

        try:
            idn = inst.query("*IDN?").strip()
            print("Found:", idn)

            # Safety only: make sure power supply output is off
            if "E3631A" in idn:
                inst.write("OUTPUT OFF")
                print("Power supply output OFF")

        except Exception as e:
            print("Could not communicate:", e)

    finally:
        # IMPORTANT:
        # close the VISA session even if *IDN? timed out
        if inst is not None:
            try:
                inst.close()
                print("Closed:", resource)
            except Exception as e:
                print("Close failed:", e)

rm.close()

print("\nAll VISA sessions released.")