"""
sensor_board.py - Thin Python wrapper around the FrontPanel endpoints of
SensorBoard_Top.v.  Import it from your own scripts:

    from sensor_board import SensorBoard
    with SensorBoard() as sb:                       # loads bitfile/SensorBoard_Top.bit
        r0, r1, r2 = sb.run(param1=0x90009102)      # start, wait for done, read results
        print(sb.status())

Endpoint map (must match hdl/SensorBoard_Top.v and vivado/build.tcl):
    WireIn  0x00 ctrl   bit0 start (rising edge), bit1 reset (level), [31:8] user
    WireIn  0x01 param1 [7:0] SAD+W (7-bit address << 1), [8] bus (0/1)
    WireIn  0x02 param2 [7:0] register, [15:8] data byte to write, [16] 1 = write / 0 = read
    WireIn  0x03 param3 [7:0] number of bytes to read (1..4)
    WireOut 0x20 result0 [3:0] ACK of frames 1..3 (1 = NACK), [15:8] SAD+W sent, [16] bus
    WireOut 0x21 result1 bytes received, first byte in [31:24]
    WireOut 0x22 result2 clock cycles from start to done
    WireOut 0x23 status bit0 busy, bit1 done, bit2 error, [15:8] dbg_state, [31:16] status_user
    WireOut 0x3F        design ID (0xEC437901: Lab 9 control plane)
    Camera (cmv300.py): WireIn 0x04 cam_ctrl, 0x05 spi_cmd, WireOut 0x24..0x2A, TriggerIn 0x41, TriggerOut 0x61
    TriggerIn  0x40     bit0 start pulse, bit1 reset pulse
    TriggerOut 0x60     bit0 done
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fp_api import ok  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_BITFILE = os.path.normpath(os.path.join(_HERE, "..", "bitfile", "SensorBoard_Top.bit"))
DESIGN_ID = 0xEC437901

EP_CTRL, EP_PARAM1, EP_PARAM2, EP_PARAM3 = 0x00, 0x01, 0x02, 0x03
EP_RESULT0, EP_RESULT1, EP_RESULT2, EP_STATUS, EP_ID = 0x20, 0x21, 0x22, 0x23, 0x3F
EP_TRIG_IN, EP_TRIG_OUT = 0x40, 0x60
TRIG_START, TRIG_RESET = 0, 1
ST_BUSY, ST_DONE, ST_ERROR = 0x1, 0x2, 0x4


class SensorBoard:
    def __init__(self, bitfile=None, open_now=True):
        self.bitfile = os.path.normpath(bitfile or DEFAULT_BITFILE)
        self.dev = None
        if open_now:
            self.open()

    # ---- lifecycle ------------------------------------------------------
    def open(self):
        dev = ok.okCFrontPanel()
        rc = dev.OpenBySerial("")
        if rc != 0:
            raise RuntimeError("OpenBySerial failed (%d): no XEM7310 on USB, or another program holds it" % rc)
        rc = dev.ConfigureFPGA(self.bitfile)
        if rc != 0:
            raise RuntimeError("ConfigureFPGA failed (%d) for %s" % (rc, self.bitfile))
        if not dev.IsFrontPanelEnabled():
            raise RuntimeError("FrontPanel not enabled in this bit file (IP built for the wrong BOARD?)")
        self.dev = dev
        self.model = dev.GetBoardModelString(dev.GetBoardModel())
        self.serial = dev.GetSerialNumber()
        return self

    def close(self):
        if self.dev is not None:
            self.dev.Close()
            self.dev = None

    def __enter__(self):
        return self if self.dev else self.open()

    def __exit__(self, *exc):
        self.close()

    # ---- low level ------------------------------------------------------
    def wire_in(self, addr, value, mask=0xFFFFFFFF, update=True):
        self.dev.SetWireInValue(addr, value & 0xFFFFFFFF, mask)
        if update:
            self.dev.UpdateWireIns()

    def wire_out(self, addr):
        self.dev.UpdateWireOuts()
        return self.dev.GetWireOutValue(addr) & 0xFFFFFFFF

    def wire_outs(self, *addrs):
        self.dev.UpdateWireOuts()
        return tuple(self.dev.GetWireOutValue(a) & 0xFFFFFFFF for a in addrs)

    def trigger(self, bit):
        self.dev.ActivateTriggerIn(EP_TRIG_IN, bit)

    def done_triggered(self):
        self.dev.UpdateTriggerOuts()
        return bool(self.dev.IsTriggered(EP_TRIG_OUT, 1 << 0))

    def usb_speed(self):
        """Negotiated USB speed: 3 = USB 3.0 SuperSpeed (~300 MB/s bulk transfers), 2 = USB 2.0 High Speed (~43 MB/s
        measured: a USB 2.0 cable or hub on the XEM7310's USB 3.0 Micro-B socket), 1 = full speed, 0 = unknown."""
        info = ok.okTDeviceInfo()
        self.dev.GetDeviceInfo(info)
        return info.usbSpeed

    def warn_if_usb2(self, need_mb_s=None):
        """Print a one-line warning when the link is not USB 3.0; returns True if it is."""
        sp = self.usb_speed()
        if sp == 3:
            return True
        print("WARNING: XEM7310 linked at %s -> bulk transfers ~43 MB/s (%s). Use a USB 3.0 (SuperSpeed Micro-B) cable "
              "on a USB 3.0 port for ~300 MB/s." % ({2: "USB 2.0 High Speed", 1: "USB 1.1"}.get(sp, "unknown speed"),
              "need ~%d MB/s for this mode" % need_mb_s if need_mb_s else "full 648x488 frames <= ~65 fps"))
        return False

    def design_id(self):
        return self.wire_out(EP_ID)

    def reset(self):
        self.trigger(TRIG_RESET)

    def set_params(self, param1=None, param2=None, param3=None, ctrl_user=None):
        if param1 is not None: self.dev.SetWireInValue(EP_PARAM1, param1 & 0xFFFFFFFF)
        if param2 is not None: self.dev.SetWireInValue(EP_PARAM2, param2 & 0xFFFFFFFF)
        if param3 is not None: self.dev.SetWireInValue(EP_PARAM3, param3 & 0xFFFFFFFF)
        if ctrl_user is not None:   # bits [31:8] of ctrl; bits 1:0 are start/reset, left alone
            self.dev.SetWireInValue(EP_CTRL, (ctrl_user << 8) & 0xFFFFFF00, 0xFFFFFF00)
        self.dev.UpdateWireIns()

    def start(self):
        self.trigger(TRIG_START)

    def status(self):
        s = self.wire_out(EP_STATUS)
        return {
            "busy": bool(s & ST_BUSY), "done": bool(s & ST_DONE), "error": bool(s & ST_ERROR),
            "state": (s >> 8) & 0xFF, "user": (s >> 16) & 0xFFFF, "raw": s,
        }

    def wait_done(self, timeout=1.0, poll=0.001):
        t0 = time.time()
        while True:
            st = self.status()
            if st["done"] and not st["busy"]:
                return st
            if time.time() - t0 > timeout:
                raise TimeoutError("FSM did not finish within %.3f s: status=%s" % (timeout, st))
            time.sleep(poll)

    def results(self):
        return self.wire_outs(EP_RESULT0, EP_RESULT1, EP_RESULT2)

    def run(self, param1=None, param2=None, param3=None, ctrl_user=None, timeout=1.0):
        """Set parameters, pulse start, wait for done, return (result0, result1, result2)."""
        self.set_params(param1, param2, param3, ctrl_user)
        self.start()
        self.wait_done(timeout)
        return self.results()

    # ---- I2C transactions (the protocol implemented in hdl/I2C_Transmit.v) --
    def i2c_read(self, addr7, reg, nbytes=1, bus=0, timeout=0.5):
        """START, SAD+W, reg, repeated START, SAD+R, nbytes bytes (last one NACKed), STOP.
        Returns (bytes, info): bytes = list of ints (first byte received first),
        info = {'acks': per-frame NACK bits (0 = every frame ACKed), 'error', 'cycles', 'us', 'state'}."""
        nbytes = max(1, min(4, nbytes))
        r0, r1, r2 = self.run(param1=(bus << 8) | ((addr7 & 0x7F) << 1),
                              param2=reg & 0xFF, param3=nbytes, timeout=timeout)
        st = self.status()
        data = [(r1 >> (24 - 8 * i)) & 0xFF for i in range(nbytes)]
        return data, {"acks": r0 & 0xF, "error": st["error"], "cycles": r2, "us": r2 / 100.8, "state": st["state"]}

    def i2c_write(self, addr7, reg, data, bus=0, timeout=0.5):
        """START, SAD+W, reg, data byte, STOP. Returns info as for i2c_read."""
        r0, r1, r2 = self.run(param1=(bus << 8) | ((addr7 & 0x7F) << 1),
                              param2=(1 << 16) | ((data & 0xFF) << 8) | (reg & 0xFF), param3=0, timeout=timeout)
        st = self.status()
        return {"acks": r0 & 0xF, "error": st["error"], "cycles": r2, "us": r2 / 100.8, "state": st["state"]}


def parse_bitfile_arg(argv=None):
    """Common CLI convention: optional first argument = path to a .bit file."""
    argv = sys.argv[1:] if argv is None else argv
    return argv[0] if argv and argv[0].endswith(".bit") else None


if __name__ == "__main__":
    # Run directly = just load the bit file and report (LED7 on the XEM7310 blinks at 1 Hz when it is live).
    with SensorBoard(parse_bitfile_arg()) as sb:
        print("Board      : %s  (serial %s)" % (sb.model, sb.serial))
        print("Bit file   : %s" % sb.bitfile)
        did = sb.design_id()
        print("Design ID  : 0x%08X  (%s)" % (did, "OK" if did == DESIGN_ID else "UNEXPECTED, want 0x%08X" % DESIGN_ID))
        sb.reset()
        print("Status     : %s" % sb.status())
        print("Loaded. Now run temperature.py (or 'temperature.py scan') to talk to the sensors.")
