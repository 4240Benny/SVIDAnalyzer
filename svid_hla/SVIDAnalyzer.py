"""
SVID (Serial VID) Protocol High Level Analyzer
For Saleae Logic 2

HOW TO USE
----------
1.  Wire VCLK and VDIO into Saleae Logic 2.
2.  Add the built-in "SPI" Low Level Analyzer:
      Clock  → VCLK
      MOSI   → VDIO
      MISO   → (none, or same channel)
      Enable → (none)
      Bits per transfer = 1
      CPOL=1, CPHA=0, MSB first
3.  Add THIS HLA on top of that SPI LLA.
    (The SPI LLA produces one "result" frame per bit.)

FRAME STRUCTURE (36 clock cycles total)
----------------------------------------
  [3b]  Start   010
  [4b]  Address A3..A0
  [5b]  Command C4..C0
  [8b]  Payload D7..D0  (MSB first)
  [1b]  Parity  (even over 17 bits: addr+cmd+payload)
  [3b]  End     011
  [~2b] Turnaround
  [2b]  ACK[1:0]
  [8b]  VR Data  (GETREG only)
  [1b]  VR Parity (GETREG only, even over VR data)
  [2b]  Turnaround / frame fill
"""

from saleae.analyzers import HighLevelAnalyzer, AnalyzerFrame, ChoicesSetting
import enum


# ═══════════════════════════════════════════════════════════════════════════════
#  Lookup tables
# ═══════════════════════════════════════════════════════════════════════════════

COMMANDS = {
    0x00: "Extended",   0x01: "SetVID_Fast", 0x02: "SetVID_Slow",
    0x03: "SetVID_Decay",0x04: "SetPS",      0x05: "SetRegADR",
    0x06: "SetRegDAT",  0x07: "GETREG",      0x08: "TestMode",
    0x09: "SetWP",
}
for _i in range(0x0A, 0x20):
    COMMANDS[_i] = f"Rsvd(0x{_i:02X})"

ACK_STR = {0b00: "AllCall-ERR", 0b01: "NAK", 0b10: "ACK", 0b11: "REJECT"}

REGISTERS = {
    0x00: "VendorID",      0x01: "ProductID",      0x02: "ProductRev",
    0x03: "DateCode",      0x04: "LotCode",         0x05: "ProtocolID",
    0x06: "Capability",
    0x10: "Status1",       0x11: "Status2",         0x12: "TempZone",
    0x13: "PMICGlobalSts", 0x14: "IoutHiByte",      0x15: "Iout",
    0x16: "Vout",          0x17: "VRTemp",          0x18: "Pout",
    0x19: "Iin",           0x1A: "Vin",             0x1B: "Pin",
    0x1C: "Status2_last",
    0x21: "ICC_MAX",       0x22: "TempMax",         0x23: "DC_LL",
    0x24: "SlewFast",      0x25: "SlewSlow",        0x26: "Vboot",
    0x27: "VR_Tol",        0x28: "IoffCal",         0x29: "ToffCal",
    0x2A: "SlowSlewSel",   0x2B: "PS4Latency",      0x2C: "PS3Latency",
    0x2D: "EnToSVIDRdy",   0x2E: "Pwr_in_Max",      0x2F: "Pwr_in_Alert",
    0x30: "VoutMax",       0x31: "VIDSetting",      0x32: "PowerState",
    0x33: "Offset",        0x34: "MultiVRCfg",      0x35: "SetRegADRpad",
    0x36: "CPUStatus",     0x37: "TrimCode1",       0x38: "TrimCode2",
    0x39: "S0iX_VID",
    0x3A: "WP0", 0x3B: "WP1", 0x3C: "WP2", 0x3D: "WP3",
    0x3E: "WP4", 0x3F: "WP5", 0x40: "WP6", 0x41: "WP7",
    0x42: "IVID1_VID", 0x43: "IVID1_I",
    0x44: "IVID2_VID", 0x45: "IVID2_I",
    0x46: "IVID3_VID", 0x47: "IVID3_I",
}

PROTOCOL_IDS = {
    0x01: "VR12.0/IMVP7", 0x02: "VR12.5",      0x03: "VR12.6",
    0x04: "VR13(10mV)",   0x05: "IMVP8",        0x06: "VR12.1",
    0x07: "VR13(5mV)",
}

VENDOR_IDS = {
    0x10: "IR",    0x11: "Fairchild", 0x12: "Intersil", 0x13: "Infineon",
    0x16: "LTC",   0x17: "Maxim",     0x18: "NXP",      0x19: "O2Micro",
    0x1A: "ONSemi",0x1B: "Powervation",0x1D: "Renesas", 0x1E: "Richtek",
    0x1F: "Rohm",  0x20: "Semtech",   0x21: "STMicro",  0x22: "TI",
    0x24: "Volterra",0x27: "UPI",     0x28: "IDT",      0x29: "Vicor",
    0x2A: "ZMDI",  0x2B: "Dialog",
}

POWER_STATES = {
    0x00: "PS0(Full)", 0x01: "PS1(Light)", 0x02: "PS2(VLight)",
    0x03: "PS3(Ultra)", 0x04: "PS4",
}

SLOW_SLEW_BITS = {0x01: "Fast/2", 0x02: "Fast/4", 0x04: "Fast/8", 0x08: "Fast/16"}
WP_SLEW        = {0: "Fast", 1: "Slow", 2: "Decay", 3: "Rsvd"}
ALL_CALL       = {0xF: "AllCall(F)", 0xE: "AllCall(E)"}


# ═══════════════════════════════════════════════════════════════════════════════
#  Helper functions
# ═══════════════════════════════════════════════════════════════════════════════

def _b2i(bits):
    v = 0
    for b in bits:
        v = (v << 1) | b
    return v


def _even_parity(value, n):
    p = 0
    for _ in range(n):
        p ^= value & 1
        value >>= 1
    return p


def _reg(a):  return REGISTERS.get(a, f"Reg(0x{a:02X})")
def _addr(a): return ALL_CALL.get(a, f"VR{a}")
def _cmd(c):  return COMMANDS.get(c, f"0x{c:02X}")


def _payload_str(cmd, payload, last_reg):
    if cmd in (0x01, 0x02, 0x03):
        return f"VID=0x{payload:02X}"
    if cmd == 0x04:
        return POWER_STATES.get(payload, f"PS=0x{payload:02X}")
    if cmd == 0x05:
        return f"→{_reg(payload)}"
    if cmd == 0x06:
        suffix = f"→{_reg(last_reg)}" if last_reg is not None else ""
        return f"0x{payload:02X}{suffix}"
    if cmd == 0x07:
        return f"[{_reg(payload)}]"
    if cmd == 0x09:
        wp = payload & 0x07
        slew = (payload >> 3) & 0x03
        alert = (payload >> 5) & 0x01
        return f"WP{wp}/{WP_SLEW[slew]}/Alrt={'Y' if alert else 'N'}"
    return f"0x{payload:02X}"


def _vrdata_str(reg_addr, vrd):
    if reg_addr == 0x00: return VENDOR_IDS.get(vrd, f"0x{vrd:02X}")
    if reg_addr == 0x05: return PROTOCOL_IDS.get(vrd, f"0x{vrd:02X}")
    if reg_addr == 0x06:
        caps = [n for m,n in [(1,"Iout"),(2,"Vout"),(4,"Pout"),(8,"Iin"),
                               (16,"Vin"),(32,"Pin"),(64,"Temp")] if vrd & m]
        return "[" + ",".join(caps) + "]" if caps else "[basic]"
    if reg_addr == 0x10:
        flags = [n for m,n in [(1,"Settled"),(2,"Therm"),(4,"ICCMax"),
                                (8,"DACHi"),(128,"RdSts2")] if vrd & m]
        return "[" + ",".join(flags) + "]" if flags else "[clear]"
    if reg_addr == 0x11:
        flags = [n for m,n in [(1,"ParErr"),(2,"FrmErr")] if vrd & m]
        return "[" + ",".join(flags) + "]" if flags else "[clear]"
    if reg_addr == 0x15: return f"{vrd*100//255}%IccMax"
    if reg_addr == 0x17: return f"{vrd}°C"
    if reg_addr == 0x21: return f"{vrd}A"
    if reg_addr == 0x22: return f"{vrd}°C"
    if reg_addr in (0x24, 0x25): return f"{vrd}mV/µs"
    if reg_addr == 0x2A:
        for m,n in SLOW_SLEW_BITS.items():
            if vrd & m: return n
    if reg_addr in (0x2B, 0x2C, 0x2D):
        x = vrd & 0x0F; y = (vrd >> 4) & 0x0F
        return f"{(x/16.0)*(2**y):.2f}µs"
    if reg_addr == 0x31: return f"VID=0x{vrd:02X}"
    if reg_addr == 0x32: return POWER_STATES.get(vrd, f"0x{vrd:02X}")
    if reg_addr == 0x33:
        sign = "-" if vrd & 0x80 else "+"
        return f"{sign}{vrd & 0x7F}steps"
    if 0x3A <= reg_addr <= 0x41:
        return f"WP{reg_addr-0x3A}=0x{vrd:02X}"
    return f"0x{vrd:02X}"


# ═══════════════════════════════════════════════════════════════════════════════
#  State machine
# ═══════════════════════════════════════════════════════════════════════════════

class St(enum.Enum):
    IDLE = 0
    CMD  = 1   # collecting 21 command-phase bits
    TA   = 2   # 2-bit turnaround
    RSP  = 3   # collecting response bits


CMD_BITS        = 21  # addr(4)+cmd(5)+payload(8)+parity(1)+end(3)
#   bit 24      = 1-bit turnaround  (master releases bus)
#   bits 25-26  = ACK[1:0]
#   bits 27-34  = VR data  (GETREG only)
#   bit  35     = VR parity (GETREG only)
TA_BITS         = 1   # single turnaround clock
RSP_BITS_SET    = 2   # ACK[1:0] only, no fill collected
RSP_BITS_GETREG = 11  # ACK(2) + VRData(8) + VRParity(1)


# ═══════════════════════════════════════════════════════════════════════════════
#  HLA
# ═══════════════════════════════════════════════════════════════════════════════

class SVIDAnalyzer(HighLevelAnalyzer):
    """SVID Protocol Decoder for Saleae Logic 2."""

    show_raw = ChoicesSetting(
        label="Show raw hex in annotations",
        choices=("No", "Yes"),
    )

    result_types = {
        "cmd":   {"format": "{{data.summary}}"},
        "ack":   {"format": "{{data.summary}}"},
        "error": {"format": "ERR: {{data.summary}}"},
    }

    # ──────────────────────────────────────────────────────────────────────────

    def __init__(self):
        self._st           = St.IDLE
        self._win          = []          # sliding 3-bit window for start detection
        self._win_ts       = []          # timestamps for each window bit
        self._bits         = []          # accumulated bits in current phase
        self._t0           = None        # start time of current frame
        self._t_last_end   = None        # end time of most recent bit
        self._rsp_t0       = None
        # decoded fields
        self._addr         = None
        self._cmd_code     = None
        self._payload      = None
        self._par_ok       = True
        self._end_ok       = True
        # cross-transaction state
        self._last_reg     = None        # last register targeted by SetRegADR
        # deferred emission queue
        self._queue        = []

    # ──────────────────────────────────────────────────────────────────────────

    def _reset(self):
        self._st       = St.IDLE
        self._win      = []
        self._win_ts   = []
        self._bits     = []
        self._t0       = None
        self._rsp_t0   = None
        self._addr     = None
        self._cmd_code = None
        self._payload  = None
        self._par_ok   = True
        self._end_ok   = True

    # ──────────────────────────────────────────────────────────────────────────

    def _parse_cmd(self):
        b = self._bits
        self._addr    = _b2i(b[0:4])
        self._cmd_code = _b2i(b[4:9])
        self._payload = _b2i(b[9:17])
        par_bit       = b[17]
        end           = b[18:21]

        data17 = (self._addr << 13) | (self._cmd_code << 8) | self._payload
        self._par_ok  = (_even_parity(data17, 17) == par_bit)
        self._end_ok  = (end == [0, 1, 1])

    # ──────────────────────────────────────────────────────────────────────────

    def _make_cmd_frame(self, t_end):
        addr_s = _addr(self._addr)
        cmd_s  = _cmd(self._cmd_code)
        pay_s  = _payload_str(self._cmd_code, self._payload, self._last_reg)

        if self._cmd_code == 0x05:
            self._last_reg = self._payload

        errs = []
        if not self._par_ok: errs.append("P_ERR")
        if not self._end_ok: errs.append("END_ERR")
        err_s = " !" + ",".join(errs) if errs else ""

        raw_s = f" [A={self._addr:04b} C={self._cmd_code:05b} D=0x{self._payload:02X}]" \
                if self.show_raw == "Yes" else ""

        summary = f"{addr_s} {cmd_s}({pay_s}){raw_s}{err_s}"
        ftype   = "error" if errs else "cmd"
        return AnalyzerFrame(ftype, self._t0, t_end, {"summary": summary})

    # ──────────────────────────────────────────────────────────────────────────

    def _make_rsp_frame(self, t_end):
        b   = self._bits
        ack = _b2i(b[0:2])
        ack_s = ACK_STR.get(ack, f"ACK?{ack:02b}")

        if self._cmd_code == 0x07 and len(b) >= 11:
            vrd      = _b2i(b[2:10])
            vpar     = b[10]
            vpar_ok  = (_even_parity(vrd, 8) == vpar)
            vrd_s    = _vrdata_str(self._payload, vrd)
            raw_s    = f" [0x{vrd:02X}]" if self.show_raw == "Yes" else ""
            par_s    = "" if vpar_ok else " P_ERR"
            summary  = f"{ack_s} ← {vrd_s}{raw_s}{par_s}"
            ftype    = "error" if (ack != 0b10 or not vpar_ok) else "ack"
        else:
            summary = ack_s
            ftype   = "error" if ack != 0b10 else "ack"

        return AnalyzerFrame(ftype, self._rsp_t0, t_end, {"summary": summary})

    # ──────────────────────────────────────────────────────────────────────────

    def decode(self, frame: AnalyzerFrame):
        # Only process data result frames from the SPI LLA.
        # Check type FIRST before touching frame.data to avoid phantom bits
        # from non-data frames (e.g. 'enable', 'error') that may carry stale data.
        if frame.type == "enable":
            self._reset()
            return None

        if frame.type not in ("result", "data"):
            return None

        # Extract bit value. SPI LLA delivers mosi as bytes (b'\x01'), int, or bool.
        bit = None
        for key in ("mosi", "value", "data"):
            raw = frame.data.get(key)
            if raw is not None:
                if isinstance(raw, (bytes, bytearray)):
                    raw = raw[0]
                bit = int(raw) & 1
                break

        if bit is None:
            return None

        # Clock-stop detection: gap > 1 µs → bus returned to Rest state → reset.
        # SaleaeTimeDelta cannot be compared to float directly; use as_seconds().
        if self._t_last_end is not None:
            try:
                gap = (frame.start_time - self._t_last_end).as_seconds()
            except AttributeError:
                gap = float(frame.start_time - self._t_last_end)
            if gap > 1e-6:
                self._reset()
        self._t_last_end = frame.end_time

        return self._feed(bit, frame.start_time, frame.end_time)

    # ──────────────────────────────────────────────────────────────────────────

    def _feed(self, bit, ts, te):
        # ── IDLE ──────────────────────────────────────────────────────────────
        # Bus idles high between frames (preamble = min 5 clock cycles of 1s).
        # We use a 5-bit sliding window and require [1,1,0,1,0]: two consecutive
        # preamble 1s must precede the 010 start pattern to prevent false triggers
        # on 010 sequences that appear naturally inside data bytes.
        #
        # Window positions when match fires:
        #   [0]=1  [1]=1  [2]=0  [3]=1  [4]=0   <- current bit is [4]
        #   The 010 start pattern is at positions [2],[3],[4].
        #   _t0 = timestamp of bit [2] = first '0' of 010.
        #   CMD bits start on the NEXT call (bit after the matched window).
        if self._st == St.IDLE:
            self._win.append(bit)
            self._win_ts.append(ts)
            if len(self._win) > 5:
                self._win.pop(0)
                self._win_ts.pop(0)
            if self._win == [1, 1, 0, 1, 0]:
                self._t0     = self._win_ts[2]   # start of the '0' in 010
                self._bits   = []
                self._win    = []
                self._win_ts = []
                self._st     = St.CMD
            return None

        # ── CMD ───────────────────────────────────────────────────────────────
        if self._st == St.CMD:
            self._bits.append(bit)
            if len(self._bits) == CMD_BITS:
                self._parse_cmd()
                f = self._make_cmd_frame(te)
                self._bits = []
                self._st   = St.TA
                return f
            return None

        # ── TA ────────────────────────────────────────────────────────────────
        # Exactly 1 turnaround bit (master releases bus; VR takes over).
        # _rsp_t0 is set to te (end of TA bit) so the RSP annotation begins
        # exactly where the ACK bits start.
        if self._st == St.TA:
            self._bits.append(bit)
            if len(self._bits) == TA_BITS:
                self._bits   = []
                self._rsp_t0 = te
                self._st     = St.RSP
            return None

        # ── RSP ───────────────────────────────────────────────────────────────
        if self._st == St.RSP:
            self._bits.append(bit)
            need = RSP_BITS_GETREG if self._cmd_code == 0x07 else RSP_BITS_SET
            if len(self._bits) == need:
                f = self._make_rsp_frame(te)
                self._reset()
                return f
            return None

        return None