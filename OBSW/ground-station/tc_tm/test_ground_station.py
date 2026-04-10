#!/usr/bin/env python3
"""
SLARS-BCG-1 — Ground Station Test Suite
test_ground_station.py

Tests the TM decoder and TC builder end-to-end.
Verifies that every TC builds correctly and every
TM field decodes accurately.

Run: python3 test_ground_station.py

SLARS Team — Bandaranayake College, Gampaha
"""

import sys
import os
import struct
import time

sys.path.insert(0, os.path.dirname(__file__))

from tm_decoder import decode_tm_frame, crc16_ccitt
from tc_sender import (
    tc_set_mode, tc_force_beacon, tc_set_max_passes,
    tc_set_adcs_mode, tc_trigger_desaturation,
    tc_reboot_obc, tc_read_fdir_log,
    tc_set_beacon_interval, tc_set_rw_speed,
    tc_enable_payload, tc_disable_payload,
    tc_format_flash, tc_set_doppler_offset,
    tc_clear_aprs_buffer, tc_start_sband_dl
)

passed = 0
total  = 0

def test(name, condition):
    global passed, total
    total += 1
    if condition:
        print(f"  PASS: {name}")
        passed += 1
    else:
        print(f"  FAIL: {name}")


def build_test_frame(soc=0.85, mode=1, adcs_mode=2,
                     pointing=8.5, rate=0.8, rw=1200,
                     bat_temp=22, obc_temp=35,
                     solar=2.0, draw=1.85) -> bytes:
    """Build a TM frame with specified values."""
    frame = bytearray(128)
    hdr = (5 << 26) | (1 << 20) | (9 << 14) | (10 << 8) | 0x01
    struct.pack_into(">I", frame, 0, hdr)
    frame[4] = 0x01
    frame[5] = 0
    struct.pack_into(">H", frame, 6, 3600)
    frame[8] = mode
    struct.pack_into(">H", frame, 9,  int(soc * 10000))
    struct.pack_into(">H", frame, 11, int((6.0 + soc*2.4) * 1000))
    frame[13] = bat_temp + 40
    struct.pack_into(">H", frame, 14, int(solar * 1000))
    struct.pack_into(">H", frame, 16, int(draw  * 1000))
    frame[18] = obc_temp + 40
    frame[19] = adcs_mode
    frame[20] = int(pointing * 2)
    frame[21] = int(rate * 4)
    struct.pack_into(">h", frame, 22, rw)
    struct.pack_into(">I", frame, 24, 47)
    frame[28] = 2
    struct.pack_into(">I", frame, 30, int(time.time()))
    crc = crc16_ccitt(bytes(frame[0:126]))
    struct.pack_into(">H", frame, 126, crc)
    return bytes(frame)


# ════════════════════════════════════════════════════════════════════════════
print("\n*" * 28)
print("*  SLARS-BCG-1 Ground Station Test Suite  *")
print("*  SLARS Team — Bandaranayake College      *")
print("*" * 28)

# ── Test 1: TM decoder basic fields ──────────────────────────────────────────
print("\n[TEST] TM decoder — basic field accuracy")
frame = build_test_frame(soc=0.85, mode=1, adcs_mode=2,
                         pointing=8.5, rate=0.8, rw=1200)
tm = decode_tm_frame(frame)

test("TM decode succeeds",                   tm is not None)
test("CRC valid",                            tm["crc_valid"])
test("System mode = 1 (NOMINAL)",            tm["system_mode"] == 1)
test("Mode name = NOMINAL",                  tm["mode_name"] == "NOMINAL")
test("Battery SoC = 85.00%",                 abs(tm["bat_soc_pct"] - 85.00) < 0.01)
test("ADCS mode = 2 (Coarse PD)",            tm["adcs_mode"] == 2)
test("ADCS name = Coarse PD",                tm["adcs_name"] == "Coarse PD")
test("Pointing error = 8.5 deg (0.5 res)",   abs(tm["pointing_deg"] - 8.5) <= 0.5)
test("Angular rate = 0.8 deg/s (0.25 res)",  abs(tm["rate_degs"]  - 0.8) <= 0.25)
test("RW speed = 1200 RPM",                  tm["rw_rpm"] == 1200)

# ── Test 2: All three system modes ───────────────────────────────────────────
print("\n[TEST] System mode encoding")
for mode_val, mode_name in [(0,"SAFE"),(1,"NOMINAL"),(2,"SCIENCE")]:
    f = build_test_frame(mode=mode_val)
    t = decode_tm_frame(f)
    test(f"Mode {mode_val} ({mode_name}) round-trips", t["mode_name"] == mode_name)

# ── Test 3: Battery edge cases ───────────────────────────────────────────────
print("\n[TEST] Battery SoC edge cases")
for soc, label in [(0.0,"0%"),(0.20,"20% safe threshold"),
                   (0.60,"60% science gate"),(1.0,"100%")]:
    f = build_test_frame(soc=soc)
    t = decode_tm_frame(f)
    test(f"SoC {label} decodes within 0.01%",
         abs(t["bat_soc_pct"] - soc*100) < 0.01)

# ── Test 4: Negative RW speed ────────────────────────────────────────────────
print("\n[TEST] Negative RW speed (signed int16)")
f = build_test_frame(rw=-1800)
t = decode_tm_frame(f)
test("RW -1800 RPM encodes and decodes correctly", t["rw_rpm"] == -1800)

f = build_test_frame(rw=-6000)
t = decode_tm_frame(f)
test("RW -6000 RPM (max negative) round-trips", t["rw_rpm"] == -6000)

# ── Test 5: Temperature edge cases ───────────────────────────────────────────
print("\n[TEST] Temperature encoding (+40 offset)")
f = build_test_frame(bat_temp=-10, obc_temp=70)
t = decode_tm_frame(f)
test("Battery -10C round-trips correctly",  t["bat_temp_c"] == -10)
test("OBC 70C round-trips correctly",       t["obc_temp_c"] == 70)

# ── Test 6: CRC corruption detection ─────────────────────────────────────────
print("\n[TEST] CRC corruption detection")
frame = build_test_frame()
valid = decode_tm_frame(frame)
test("Valid frame: CRC passes", valid["crc_valid"])

bad = bytearray(frame)
bad[9] ^= 0xFF
invalid = decode_tm_frame(bytes(bad))
test("Corrupted byte 9: CRC fails", not invalid["crc_valid"])

bad2 = bytearray(frame)
bad2[126] ^= 0x01
invalid2 = decode_tm_frame(bytes(bad2))
test("Corrupted CRC byte: decode rejects", not invalid2["crc_valid"])

# ── Test 7: TC builder — command IDs ─────────────────────────────────────────
print("\n[TEST] TC builder — command IDs")
test("SET_MODE 0 starts with 0x01",       tc_set_mode(0)[0]           == 0x01)
test("FORCE_BEACON starts with 0x06",     tc_force_beacon()[0]        == 0x06)
test("SET_MAX_PASSES starts with 0x05",   tc_set_max_passes(2)[0]     == 0x05)
test("SET_ADCS_MODE starts with 0x0A",    tc_set_adcs_mode(3)[0]      == 0x0A)
test("TRIGGER_DESAT starts with 0x14",    tc_trigger_desaturation()[0]== 0x14)
test("FORMAT_FLASH starts with 0x0D",     tc_format_flash()[0]        == 0x0D)
test("SET_RW_SPEED starts with 0x11",     tc_set_rw_speed(1000)[0]    == 0x11)
test("CLEAR_APRS_BUFFER starts with 0x0B",tc_clear_aprs_buffer()[0]   == 0x0B)
test("START_SBAND_DL starts with 0x0C",   tc_start_sband_dl(5)[0]     == 0x0C)
test("READ_FDIR_LOG starts with 0x12",    tc_read_fdir_log(10)[0]     == 0x12)

# ── Test 8: TC parameter encoding ────────────────────────────────────────────
print("\n[TEST] TC parameter encoding")
tc = tc_set_max_passes(3)
test("SET_MAX_PASSES 3: param byte = 3", tc[1] == 3)

tc = tc_set_adcs_mode(2)
test("SET_ADCS_MODE 2: param byte = 2", tc[1] == 2)

tc = tc_reboot_obc(10)
test("REBOOT_OBC 10s: param byte = 10", tc[1] == 10)

tc = tc_set_beacon_interval(60)
test("SET_BEACON_INTERVAL 60s: param byte = 60", tc[1] == 60)

tc = tc_format_flash()
confirm = struct.unpack(">H", tc[1:3])[0]
test("FORMAT_FLASH: confirm bytes = 0xDEAD", confirm == 0xDEAD)

tc = tc_set_rw_speed(-500)
rpm = struct.unpack(">h", tc[1:3])[0]
test("SET_RW_SPEED -500: signed int16 encodes correctly", rpm == -500)

tc = tc_set_doppler_offset(-9500)
hz = struct.unpack(">i", tc[1:5])[0]
test("SET_DOPPLER_OFFSET -9500: signed int32 encodes correctly", hz == -9500)

# ── Test 9: NULL and empty frame safety ──────────────────────────────────────
print("\n[TEST] Edge cases and safety")
result = decode_tm_frame(b"")
test("Empty frame: decode returns None", result is None)

result = decode_tm_frame(b"\x00" * 64)
test("64-byte frame: decode returns None (too short)", result is None)

result = decode_tm_frame(b"\x00" * 128)
tm_zero = decode_tm_frame(b"\x00" * 128)
test("All-zero frame: decode does not crash", tm_zero is not None)

# ── Results ───────────────────────────────────────────────────────────────────
print()
print("=" * 48)
print(f"  RESULTS: {passed} / {total} tests passed")
if passed == total:
    print("  ALL TESTS PASSED")
else:
    print(f"  {total - passed} FAILED")
print("=" * 48)
print()

sys.exit(0 if passed == total else 1)
