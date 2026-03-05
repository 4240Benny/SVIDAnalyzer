# SVID Protocol Analyzer — Saleae Logic 2 HLA

(vibe coded with claude - WTFPL license)

Decodes **Serial VID (SVID)** protocol.

---

## Installation

1. Copy the entire `svid_hla/` folder somewhere convenient.
2. In **Logic 2** → *Extensions* → *Load Existing Extension…*  
   Select the `svid_hla` folder.

---

## Wiring

| Signal | Logic Channel |
|--------|--------------|
| VCLK   | Any digital  |
| VDIO   | Any digital  |

---

## Setup in Logic 2

### Step 1 — Add a Low-Level Analyzer

Add the built-in **SPI** analyzer with these settings:

| Setting | Value |
|---------|-------|
| Clock (SCLK) | VCLK channel |
| MOSI | VDIO channel |
| MISO | (none) |
| Enable | (none) |
| Bits per Transfer | **1** |
| Clock Polarity (CPOL) | **1** |
| Clock Phase (CPHA) | 0 |
| MSB/LSB First | MSB First |

> **Why 1-bit SPI?**  SVID is a bidirectional single-wire bus. The SPI LLA
> with 1-bit transfers gives the HLA one frame per clock edge, which is what
> the state machine expects.

### Step 2 — Add the SVID HLA

*Analyzers → Add HLA → SVID Protocol Analyzer*  
Select the SPI LLA as its input.

### Option: Show Raw Hex

The HLA has a **"Show raw hex in annotations"** setting (Yes/No).  
When enabled, every annotation includes the raw bit fields:
`[A=0000 C=00111 D=0x10]`

---

## What Gets Decoded

### Command frames

```
VR0 SetVID_Fast(VID=0xFB)
VR1 GETREG([Status1])
VR0 SetPS(PS3(Ultra))
VRF AllCall(F) SetWP(WP1/Fast/Alrt=Y)
VR2 SetRegADR(→Vboot)
VR2 SetRegDAT(0x9A→Vboot)
```

### Response frames

```
ACK
NAK
REJECT
AllCall-ERR
ACK ← [Settled,Therm]         (GETREG Status1)
ACK ← 42°C                    (GETREG VRTemp)
ACK ← 75A                     (GETREG ICC_MAX)
ACK ← VR12.5                  (GETREG ProtocolID)
ACK ← [Iout,Vout,Temp]        (GETREG Capability)
ACK ← 1.25µs                  (GETREG PS3Latency)
ACK ← Fast/4                  (GETREG SlowSlewSel)
```

---

## Protocol Summary (for reference)

```
36-clock data frame:

  [3]  Start pattern   010
  [4]  Address         VR0–VR14, AllCall(F/E)
  [5]  Command         SetVID_Fast/Slow/Decay, SetPS, SetRegADR,
                       SetRegDAT, GETREG, SetWP, TestMode, Extended
  [8]  Payload         VID code / register address / power state / WP
  [1]  Parity          Even parity over addr+cmd+payload (17 bits)
  [3]  End pattern     011
  [~2] Turnaround
  [2]  ACK[1:0]        10=ACK  01=NAK  11=REJECT  00=AllCall-ERR
  [8]  VR Data         (GETREG only)
  [1]  VR Parity       (GETREG only, even over VR data)
  [2]  Frame fill / TA
```

---

## Error Highlighting

Frames are coloured **red** in Logic 2 when:
- Command parity error
- Command end-pattern not `011`
- ACK is NAK / REJECT / AllCall-ERR
- GETREG VR-data parity error

---

## Clock-Stop Handling

Any gap > 1 µs on the clock line resets the link.  
The HLA detects this and resets its state machine automatically.
