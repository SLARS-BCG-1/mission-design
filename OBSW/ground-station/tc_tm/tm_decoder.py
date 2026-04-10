#!/usr/bin/env python3
"""
SLARS-BCG-1 Ground Station — TM Frame Decoder
tm_decoder.py

Decodes a 128-byte CSP/AX.25 housekeeping TM frame
into human-readable fields.

The frame layout matches exactly what tm_encoder.c produces:
  Bytes 0-3   : CSP header
  Byte  4     : Frame type (0x01 = housekeeping)
  Byte  5     : Sequence counter
  Bytes 6-7   : Mission elapsed time (seconds)
  Byte  8     : System mode (0=SAFE 1=NOMINAL 2=SCIENCE)
  Bytes 9-10  : Battery SoC (x10000)
  Bytes 11-12 : Battery voltage (millivolts)
  Byte  13    : Battery temperature (degC + 40 offset)
  Bytes 14-15 : Solar power (milliwatts)
  Bytes 16-17 : Total draw (milliwatts)
  Byte  18    : OBC temperature (degC + 40 offset)
  Byte  19    : ADCS mode
  Byte  20    : Pointing error (degrees x 2)
  Byte  21    : Angular rate (deg/s x 4)
  Bytes 22-23 : Reaction wheel speed (int16 RPM)
  Bytes 24-27 : Orbit count (uint32)
  Byte  28    : Science passes today
  Byte  29    : APRS queue length
  Bytes 30-33 : Unix timestamp (uint32)
  Byte  34    : FDIR level
  Bytes 126-127: CRC16-CCITT

SLARS Team — Bandaranayake College, Gampaha
"""

import struct
import time
from datetime import datetime, timezone


# ── CRC16-CCITT (matches the C implementation in tm_encoder.c) ──────────────

def crc16_ccitt(data: bytes) -> int:
    """Compute CRC16-CCITT over data. Polynomial 0x1021, init 0xFFFF."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ── CSP header decoder ───────────────────────────────────────────────────────

def decode_csp_header(frame: bytes) -> dict:
    """Unpack the 4-byte CSP header from bytes 0-3."""
    hdr = struct.unpack(">I", frame[0:4])[0]
    return {
        "src_addr":  (hdr >> 26) & 0x3F,
        "dst_addr":  (hdr >> 20) & 0x3F,
        "dst_port":  (hdr >> 14) & 0x3F,
        "src_port":  (hdr >>  8) & 0x3F,
        "flags":      hdr        & 0xFF,
    }


# ── Main decoder ─────────────────────────────────────────────────────────────

def decode_tm_frame(frame: bytes) -> dict:
    """
    Decode a 128-byte housekeeping TM frame.
    Returns a dict of all fields plus crc_valid flag.
    Returns None if frame is the wrong length.
    """
    if len(frame) != 128:
        print(f"[TM DECODER] ERROR: frame is {len(frame)} bytes, expected 128")
        return None

    # Verify CRC16 over bytes 0-125
    expected_crc = crc16_ccitt(frame[0:126])
    received_crc = struct.unpack(">H", frame[126:128])[0]
    crc_valid = (expected_crc == received_crc)

    if not crc_valid:
        print(f"[TM DECODER] CRC MISMATCH: "
              f"expected 0x{expected_crc:04X} got 0x{received_crc:04X}")

    csp = decode_csp_header(frame)

    # Decode all telemetry fields
    frame_type   = frame[4]
    seq_counter  = frame[5]
    met_s        = struct.unpack(">H", frame[6:8])[0]
    system_mode  = frame[8]
    bat_soc_raw  = struct.unpack(">H", frame[9:11])[0]
    bat_v_raw    = struct.unpack(">H", frame[11:13])[0]
    bat_temp_raw = frame[13]
    solar_raw    = struct.unpack(">H", frame[14:16])[0]
    draw_raw     = struct.unpack(">H", frame[16:18])[0]
    obc_temp_raw = frame[18]
    adcs_mode    = frame[19]
    point_raw    = frame[20]
    rate_raw     = frame[21]
    rw_rpm       = struct.unpack(">h", frame[22:24])[0]   # signed int16
    orbit_count  = struct.unpack(">I", frame[24:28])[0]
    passes_today = frame[28]
    aprs_queue   = frame[29]
    unix_ts      = struct.unpack(">I", frame[30:34])[0]
    fdir_level   = frame[34]

    # Convert raw values to engineering units
    bat_soc_pct    = bat_soc_raw / 100.0        # 8500 -> 85.00%
    bat_voltage_v  = bat_v_raw   / 1000.0       # 7800 -> 7.800 V
    bat_temp_c     = bat_temp_raw - 40          # +40 offset removed
    solar_w        = solar_raw   / 1000.0       # 2000 -> 2.000 W
    draw_w         = draw_raw    / 1000.0       # 1850 -> 1.850 W
    obc_temp_c     = obc_temp_raw - 40          # +40 offset removed
    pointing_deg   = point_raw   / 2.0          # 9 -> 4.5 deg
    rate_degs      = rate_raw    / 4.0          # 3 -> 0.75 deg/s

    # UTC timestamp
    try:
        utc_str = datetime.fromtimestamp(
            unix_ts, tz=timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")
    except Exception:
        utc_str = f"Unix: {unix_ts}"

    mode_names = {0: "SAFE", 1: "NOMINAL", 2: "SCIENCE"}
    adcs_names = {1: "B-dot", 2: "Coarse PD", 3: "Nadir PID"}

    return {
        # CSP header
        "csp_src":        csp["src_addr"],
        "csp_dst":        csp["dst_addr"],
        "csp_dst_port":   csp["dst_port"],
        "csp_src_port":   csp["src_port"],
        "csp_flags":      csp["flags"],
        # Frame metadata
        "frame_type":     frame_type,
        "seq_counter":    seq_counter,
        "met_seconds":    met_s,
        "crc_valid":      crc_valid,
        # Power
        "bat_soc_pct":    bat_soc_pct,
        "bat_voltage_v":  bat_voltage_v,
        "bat_temp_c":     bat_temp_c,
        "solar_w":        solar_w,
        "draw_w":         draw_w,
        "system_mode":    system_mode,
        "mode_name":      mode_names.get(system_mode, f"UNKNOWN({system_mode})"),
        # OBC
        "obc_temp_c":     obc_temp_c,
        "orbit_count":    orbit_count,
        "unix_time":      unix_ts,
        "utc_str":        utc_str,
        # ADCS
        "adcs_mode":      adcs_mode,
        "adcs_name":      adcs_names.get(adcs_mode, f"UNKNOWN({adcs_mode})"),
        "pointing_deg":   pointing_deg,
        "rate_degs":      rate_degs,
        "rw_rpm":         rw_rpm,
        # Payload
        "passes_today":   passes_today,
        "aprs_queue":     aprs_queue,
        "fdir_level":     fdir_level,
    }


# ── Pretty printer ───────────────────────────────────────────────────────────

def print_tm_frame(tm: dict) -> None:
    """Print a decoded TM frame in a clean, readable format."""
    if tm is None:
        print("[TM DECODER] Cannot print — decode failed")
        return

    crc_str = "VALID" if tm["crc_valid"] else "INVALID"
    fdir_str = ("NOMINAL" if tm["fdir_level"] == 0
                else f"LEVEL {tm['fdir_level']}")

    print()
    print("  ╔══════════════════════════════════════════════════╗")
    print("  ║       SLARS-BCG-1  HOUSEKEEPING TELEMETRY        ║")
    print("  ╠══════════════════════════════════════════════════╣")
    print(f"  ║  Seq: {tm['seq_counter']:<5}  "
          f"MET: {tm['met_seconds']}s  "
          f"CRC: {crc_str:<8}              ║")
    print(f"  ║  Time: {tm['utc_str']:<43}║")
    print("  ╠══════════════════════════════════════════════════╣")
    print(f"  ║  MODE:    {tm['mode_name']:<8}  "
          f"FDIR: {fdir_str:<8}  "
          f"Orbit: {tm['orbit_count']:<6}║")
    print("  ╠══════════════════════════════════════════════════╣")
    print("  ║  POWER SUBSYSTEM                                 ║")
    print(f"  ║    Battery:  {tm['bat_soc_pct']:6.2f}%  "
          f"{tm['bat_voltage_v']:.3f}V  "
          f"{tm['bat_temp_c']:+.0f}C             ║")
    print(f"  ║    Solar:    {tm['solar_w']:.3f}W   "
          f"Draw: {tm['draw_w']:.3f}W  "
          f"OBC: {tm['obc_temp_c']:+.0f}C          ║")
    print("  ╠══════════════════════════════════════════════════╣")
    print("  ║  ADCS SUBSYSTEM                                  ║")
    print(f"  ║    Mode:     {tm['adcs_name']:<12}  "
          f"RW: {tm['rw_rpm']:+6} RPM          ║")
    print(f"  ║    Pointing: {tm['pointing_deg']:5.1f} deg  "
          f"Rate: {tm['rate_degs']:.2f} deg/s              ║")
    print("  ╠══════════════════════════════════════════════════╣")
    print("  ║  PAYLOAD                                         ║")
    print(f"  ║    Passes today: {tm['passes_today']:<4}  "
          f"APRS queue: {tm['aprs_queue']:<4}                ║")
    print("  ╚══════════════════════════════════════════════════╝")
    print()


# ── Standalone test ───────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("SLARS-BCG-1 TM Decoder — self test")
    print("Building a test frame manually...")

    # Build a test frame that matches what tm_encoder.c would produce
    frame = bytearray(128)

    # CSP header: src=5 dst=1 dport=9 sport=10 flags=0x01
    hdr = (5 << 26) | (1 << 20) | (9 << 14) | (10 << 8) | 0x01
    struct.pack_into(">I", frame, 0, hdr)

    frame[4]  = 0x01           # housekeeping type
    frame[5]  = 42             # seq counter
    struct.pack_into(">H", frame, 6, 3600)   # MET = 1 hour
    frame[8]  = 1              # NOMINAL mode
    struct.pack_into(">H", frame, 9, 8500)   # SoC = 85.00%
    struct.pack_into(">H", frame, 11, 7968)  # voltage = 7.968V
    frame[13] = 22 + 40        # battery temp 22C
    struct.pack_into(">H", frame, 14, 2000)  # solar 2.000W
    struct.pack_into(">H", frame, 16, 1850)  # draw 1.850W
    frame[18] = 35 + 40        # OBC temp 35C
    frame[19] = 2              # ADCS Mode 2 (Coarse PD)
    frame[20] = int(8.5 * 2)   # pointing 8.5 deg
    frame[21] = int(0.8 * 4)   # rate 0.8 deg/s
    struct.pack_into(">h", frame, 22, 1200)  # RW 1200 RPM
    struct.pack_into(">I", frame, 24, 47)    # orbit 47
    frame[28] = 2              # passes today
    frame[29] = 0              # APRS queue
    struct.pack_into(">I", frame, 30, int(time.time()))
    frame[34] = 0              # FDIR nominal

    # Compute and append CRC
    crc = crc16_ccitt(bytes(frame[0:126]))
    struct.pack_into(">H", frame, 126, crc)

    tm = decode_tm_frame(bytes(frame))
    print_tm_frame(tm)

    print("CRC verification:")
    print(f"  Computed: 0x{crc:04X}")
    print(f"  Valid:    {tm['crc_valid']}")

    # Corrupt test
    bad = bytearray(frame)
    bad[10] ^= 0xFF
    tm_bad = decode_tm_frame(bytes(bad))
    print(f"  Corrupted frame CRC valid: {tm_bad['crc_valid']}  (should be False)")
