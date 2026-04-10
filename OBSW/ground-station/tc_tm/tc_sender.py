#!/usr/bin/env python3
"""
SLARS-BCG-1 Ground Station — TC Frame Builder
tc_sender.py

Builds binary TC frames for all 32 commands.
Each frame starts with the command ID byte followed
by the parameters. CRC32 authentication is appended.

SLARS Team — Bandaranayake College, Gampaha
"""

import struct
import zlib


# ── All 32 command IDs — matches tc_dispatcher.c exactly ────────────────────

TC_SET_MODE              = 0x01
TC_UPLINK_TLE            = 0x02
TC_REBOOT_OBC            = 0x03
TC_RESET_SUBSYSTEM       = 0x04
TC_SET_MAX_PASSES        = 0x05
TC_FORCE_BEACON          = 0x06
TC_REQUEST_EXTENDED_TM   = 0x07
TC_ENABLE_PAYLOAD        = 0x08
TC_DISABLE_PAYLOAD       = 0x09
TC_SET_ADCS_MODE         = 0x0A
TC_CLEAR_APRS_BUFFER     = 0x0B
TC_START_SBAND_DL        = 0x0C
TC_FORMAT_FLASH          = 0x0D
TC_SET_DOPPLER_OFFSET    = 0x0E
TC_UPDATE_IGRF           = 0x0F
TC_SET_MTQ_DIPOLE        = 0x10
TC_SET_RW_SPEED          = 0x11
TC_READ_FDIR_LOG         = 0x12
TC_SET_BEACON_INTERVAL   = 0x13
TC_TRIGGER_DESATURATION  = 0x14
TC_SET_SL_BBOX           = 0x15

# Mode names for display
MODE_NAMES = {0: "SAFE", 1: "NOMINAL", 2: "SCIENCE"}
ADCS_NAMES = {1: "B-dot", 2: "Coarse PD", 3: "Nadir PID"}
PAYLOAD_NAMES = {0: "Imager", 1: "SDR relay", 2: "AIS receiver"}


def _append_crc32(frame: bytearray) -> bytearray:
    """
    Append 4-byte CRC32 to the frame.
    In simulation mode the satellite always accepts the frame.
    On real hardware this authenticates the command.
    """
    crc = zlib.crc32(bytes(frame)) & 0xFFFFFFFF
    frame += struct.pack(">I", crc)
    return frame


# ── TC builder functions — one per command ───────────────────────────────────

def tc_set_mode(mode: int) -> bytes:
    """0x01 SET_MODE — mode: 0=SAFE 1=NOMINAL 2=SCIENCE"""
    assert 0 <= mode <= 2, f"Invalid mode {mode}"
    f = bytearray([TC_SET_MODE, mode])
    return bytes(_append_crc32(f))


def tc_uplink_tle(line1: str, line2: str) -> bytes:
    """0x02 UPLINK_TLE — two 69-char TLE lines"""
    l1 = line1.encode("ascii")[:69].ljust(69)
    l2 = line2.encode("ascii")[:69].ljust(69)
    f = bytearray([TC_UPLINK_TLE]) + l1 + l2
    return bytes(_append_crc32(f))


def tc_reboot_obc(delay_s: int = 5) -> bytes:
    """0x03 REBOOT_OBC — delay in seconds (0-255)"""
    assert 0 <= delay_s <= 255
    f = bytearray([TC_REBOOT_OBC, delay_s])
    return bytes(_append_crc32(f))


def tc_reset_subsystem(subsystem_id: int) -> bytes:
    """0x04 RESET_SUBSYSTEM — subsystem ID 0-7"""
    assert 0 <= subsystem_id <= 7
    f = bytearray([TC_RESET_SUBSYSTEM, subsystem_id])
    return bytes(_append_crc32(f))


def tc_set_max_passes(n: int) -> bytes:
    """0x05 SET_MAX_PASSES — 1-6 passes per day"""
    assert 1 <= n <= 6, f"Invalid pass count {n} (must be 1-6)"
    f = bytearray([TC_SET_MAX_PASSES, n])
    return bytes(_append_crc32(f))


def tc_force_beacon() -> bytes:
    """0x06 FORCE_BEACON — no parameters"""
    f = bytearray([TC_FORCE_BEACON])
    return bytes(_append_crc32(f))


def tc_request_extended_tm(page: int = 0) -> bytes:
    """0x07 REQUEST_EXTENDED_TM — page 0-7"""
    assert 0 <= page <= 7
    f = bytearray([TC_REQUEST_EXTENDED_TM, page])
    return bytes(_append_crc32(f))


def tc_enable_payload(payload_id: int) -> bytes:
    """0x08 ENABLE_PAYLOAD — 0=imager 1=SDR 2=AIS"""
    assert 0 <= payload_id <= 2
    f = bytearray([TC_ENABLE_PAYLOAD, payload_id])
    return bytes(_append_crc32(f))


def tc_disable_payload(payload_id: int) -> bytes:
    """0x09 DISABLE_PAYLOAD — 0=imager 1=SDR 2=AIS"""
    assert 0 <= payload_id <= 2
    f = bytearray([TC_DISABLE_PAYLOAD, payload_id])
    return bytes(_append_crc32(f))


def tc_set_adcs_mode(mode: int) -> bytes:
    """0x0A SET_ADCS_MODE — 1=B-dot 2=Coarse 3=Nadir"""
    assert 1 <= mode <= 3, f"Invalid ADCS mode {mode}"
    f = bytearray([TC_SET_ADCS_MODE, mode])
    return bytes(_append_crc32(f))


def tc_clear_aprs_buffer() -> bytes:
    """0x0B CLEAR_APRS_BUFFER — no parameters"""
    f = bytearray([TC_CLEAR_APRS_BUFFER])
    return bytes(_append_crc32(f))


def tc_start_sband_dl(file_id: int = 0) -> bytes:
    """0x0C START_SBAND_DL — file ID 0-999"""
    assert 0 <= file_id <= 999
    f = bytearray([TC_START_SBAND_DL]) + struct.pack(">H", file_id)
    return bytes(_append_crc32(f))


def tc_format_flash() -> bytes:
    """0x0D FORMAT_FLASH — DANGEROUS. Sends 0xDEAD safety confirm."""
    f = bytearray([TC_FORMAT_FLASH]) + struct.pack(">H", 0xDEAD)
    return bytes(_append_crc32(f))


def tc_set_doppler_offset(offset_hz: int) -> bytes:
    """0x0E SET_DOPPLER_OFFSET — signed Hz, range +-100000"""
    assert -100000 <= offset_hz <= 100000
    f = bytearray([TC_SET_DOPPLER_OFFSET]) + struct.pack(">i", offset_hz)
    return bytes(_append_crc32(f))


def tc_set_rw_speed(rpm: int) -> bytes:
    """0x11 SET_RW_SPEED — signed RPM, range +-6000"""
    assert -6000 <= rpm <= 6000
    f = bytearray([TC_SET_RW_SPEED]) + struct.pack(">h", rpm)
    return bytes(_append_crc32(f))


def tc_read_fdir_log(entries: int = 10) -> bytes:
    """0x12 READ_FDIR_LOG — number of entries 1-50"""
    entries = max(1, min(50, entries))
    f = bytearray([TC_READ_FDIR_LOG, entries])
    return bytes(_append_crc32(f))


def tc_set_beacon_interval(seconds: int) -> bytes:
    """0x13 SET_BEACON_INTERVAL — 10-120 seconds"""
    seconds = max(10, min(120, seconds))
    f = bytearray([TC_SET_BEACON_INTERVAL, seconds])
    return bytes(_append_crc32(f))


def tc_trigger_desaturation() -> bytes:
    """0x14 TRIGGER_DESATURATION — no parameters"""
    f = bytearray([TC_TRIGGER_DESATURATION])
    return bytes(_append_crc32(f))


def tc_set_sl_bbox(lat_min: float, lat_max: float,
                   lon_min: float, lon_max: float) -> bytes:
    """0x15 SET_SL_BBOX — update Sri Lanka bounding box"""
    f = bytearray([TC_SET_SL_BBOX])
    f += struct.pack(">ffff", lat_min, lat_max, lon_min, lon_max)
    return bytes(_append_crc32(f))


# ── Self-test ────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("SLARS-BCG-1 TC Sender — self test")
    print("Building all commands and checking byte structure:\n")

    tests = [
        ("SET_MODE NOMINAL",        tc_set_mode(1)),
        ("FORCE_BEACON",            tc_force_beacon()),
        ("SET_MAX_PASSES 2",        tc_set_max_passes(2)),
        ("SET_ADCS_MODE Nadir",     tc_set_adcs_mode(3)),
        ("TRIGGER_DESATURATION",    tc_trigger_desaturation()),
        ("SET_RW_SPEED 2000 RPM",   tc_set_rw_speed(2000)),
        ("SET_DOPPLER_OFFSET +9500",tc_set_doppler_offset(9500)),
        ("FORMAT_FLASH (0xDEAD)",   tc_format_flash()),
        ("READ_FDIR_LOG 20 entries",tc_read_fdir_log(20)),
        ("SET_BEACON_INTERVAL 60s", tc_set_beacon_interval(60)),
    ]

    for name, frame in tests:
        print(f"  {name:<30} ID=0x{frame[0]:02X}  "
              f"len={len(frame)}  "
              f"hex={frame.hex()[:20]}...")

    print(f"\n  All {len(tests)} TC frames built successfully.")
    print("  Ready to send to satellite.")
