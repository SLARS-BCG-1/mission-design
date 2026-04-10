#!/usr/bin/env python3
"""
SLARS-BCG-1 Ground Station — Interactive Terminal
ground_station.py

Simulates a complete ground station operator interface.
Runs in two modes:

  1. SIMULATION MODE (default, no hardware needed)
     Connects to the C simulation by running slars_obsw_sim
     as a subprocess and exchanging frames via stdin/stdout.

  2. SERIAL MODE (when real OBC arrives)
     Connects to the real OBC over USB serial port.
     Run with:  python3 ground_station.py --port /dev/ttyUSB0

Type 'help' at the prompt to see all commands.

SLARS Team — Bandaranayake College, Gampaha
"""

import struct
import time
import zlib
import sys
import os

# Add parent directory to path so we can import our modules
sys.path.insert(0, os.path.dirname(__file__))

from tm_decoder import decode_tm_frame, print_tm_frame, crc16_ccitt
from tc_sender import (
    tc_set_mode, tc_force_beacon, tc_set_max_passes,
    tc_set_adcs_mode, tc_trigger_desaturation,
    tc_uplink_tle, tc_reboot_obc, tc_read_fdir_log,
    tc_set_beacon_interval, tc_set_rw_speed,
    tc_enable_payload, tc_disable_payload,
    tc_clear_aprs_buffer, tc_start_sband_dl,
    MODE_NAMES, ADCS_NAMES, PAYLOAD_NAMES
)


# ── Simulation: build a TM frame from current satellite state ────────────────

class SimulatedSatellite:
    """
    Simulates the satellite for ground station testing.
    Maintains internal state that changes as TCs are received.
    Produces realistic TM frames in response to FORCE_BEACON.
    """

    def __init__(self):
        self.mode         = 1       # NOMINAL
        self.bat_soc      = 0.85
        self.bat_temp     = 22.0
        self.obc_temp     = 35.0
        self.solar_w      = 2.0
        self.draw_w       = 1.85
        self.adcs_mode    = 1       # B-dot at start
        self.pointing_err = 45.0   # tumbling at start
        self.ang_rate     = 5.2
        self.rw_rpm       = 0
        self.orbit_count  = 0
        self.passes_today = 0
        self.aprs_queue   = 0
        self.fdir_level   = 0
        self.seq_counter  = 0
        self.boot_time    = int(time.time())
        self.max_passes   = 3
        self.beacon_interval = 30

        # Start a background thread to simulate orbital dynamics
        import threading
        self._running = True
        self._thread  = threading.Thread(target=self._orbital_sim, daemon=True)
        self._thread.start()

    def _orbital_sim(self):
        """Simulate satellite dynamics over time."""
        while self._running:
            time.sleep(1)
            # Battery slowly drains in nominal mode
            if self.mode == 1:
                self.bat_soc = max(0.20, self.bat_soc - 0.0002)
            # Battery charges faster in science mode (solar facing)
            elif self.mode == 2:
                self.bat_soc = min(0.95, self.bat_soc + 0.0001)
            # ADCS slowly detumbles
            if self.adcs_mode == 1 and self.ang_rate > 0.1:
                self.ang_rate = max(0.0, self.ang_rate - 0.05)
                if self.ang_rate < 3.0:
                    self.adcs_mode    = 2
                    self.pointing_err = 15.0
                    print("\n  [SIM] ADCS: detumbling complete -> Mode 2")
            elif self.adcs_mode == 2 and self.pointing_err > 1.0:
                self.pointing_err = max(1.0, self.pointing_err - 0.3)
            # Orbit counter increments every ~94 minutes
            elapsed = int(time.time()) - self.boot_time
            self.orbit_count = elapsed // 5676

    def build_tm_frame(self) -> bytes:
        """Build a 128-byte TM frame from current satellite state."""
        frame = bytearray(128)

        # CSP header: src=5 dst=1 dport=9 sport=10 flags=0x01
        hdr = (5 << 26) | (1 << 20) | (9 << 14) | (10 << 8) | 0x01
        struct.pack_into(">I", frame, 0, hdr)

        frame[4] = 0x01
        frame[5] = self.seq_counter & 0xFF
        self.seq_counter += 1

        met = int(time.time()) - self.boot_time
        struct.pack_into(">H", frame, 6, min(met, 0xFFFF))
        frame[8] = self.mode

        soc_raw = int(self.bat_soc * 10000)
        struct.pack_into(">H", frame, 9, min(soc_raw, 0xFFFF))

        v = 6.0 + self.bat_soc * 2.4
        struct.pack_into(">H", frame, 11, int(v * 1000))

        frame[13] = max(0, min(255, int(self.bat_temp) + 40))
        struct.pack_into(">H", frame, 14, int(self.solar_w * 1000))
        struct.pack_into(">H", frame, 16, int(self.draw_w * 1000))
        frame[18] = max(0, min(255, int(self.obc_temp) + 40))
        frame[19] = self.adcs_mode
        frame[20] = min(255, int(self.pointing_err * 2))
        frame[21] = min(255, int(self.ang_rate * 4))
        struct.pack_into(">h", frame, 22, max(-32768, min(32767, self.rw_rpm)))
        struct.pack_into(">I", frame, 24, self.orbit_count)
        frame[28] = self.passes_today
        frame[29] = self.aprs_queue
        struct.pack_into(">I", frame, 30, int(time.time()))
        frame[34] = self.fdir_level

        crc = crc16_ccitt(bytes(frame[0:126]))
        struct.pack_into(">H", frame, 126, crc)

        return bytes(frame)

    def process_tc(self, tc_bytes: bytes) -> str:
        """
        Process a TC command and update satellite state.
        Returns a string describing what happened.
        """
        if not tc_bytes:
            return "Empty TC frame"

        cmd_id = tc_bytes[0]

        if cmd_id == 0x01:   # SET_MODE
            if len(tc_bytes) >= 2:
                m = tc_bytes[1]
                if 0 <= m <= 2:
                    self.mode = m
                    return f"Mode -> {MODE_NAMES[m]}"
            return "SET_MODE: invalid parameter"

        elif cmd_id == 0x05:  # SET_MAX_PASSES
            if len(tc_bytes) >= 2:
                n = tc_bytes[1]
                if 1 <= n <= 6:
                    self.max_passes = n
                    return f"Max passes -> {n}/day"
            return "SET_MAX_PASSES: invalid parameter"

        elif cmd_id == 0x06:  # FORCE_BEACON
            return "Beacon transmitted"

        elif cmd_id == 0x0A:  # SET_ADCS_MODE
            if len(tc_bytes) >= 2:
                m = tc_bytes[1]
                if 1 <= m <= 3:
                    self.adcs_mode = m
                    if m == 1:
                        self.ang_rate = 5.0
                        self.pointing_err = 45.0
                    return f"ADCS -> {ADCS_NAMES[m]}"
            return "SET_ADCS_MODE: invalid parameter"

        elif cmd_id == 0x13:  # SET_BEACON_INTERVAL
            if len(tc_bytes) >= 2:
                s = tc_bytes[1]
                if 10 <= s <= 120:
                    self.beacon_interval = s
                    return f"Beacon interval -> {s}s"

        elif cmd_id == 0x14:  # TRIGGER_DESATURATION
            old = self.rw_rpm
            self.rw_rpm = int(self.rw_rpm * 0.2)
            return f"RW desaturation: {old} -> {self.rw_rpm} RPM"

        elif cmd_id == 0x0B:  # CLEAR_APRS_BUFFER
            self.aprs_queue = 0
            return "APRS buffer cleared"

        elif cmd_id == 0x12:  # READ_FDIR_LOG
            return f"FDIR log: {self.fdir_level} active events"

        return f"TC 0x{cmd_id:02X} executed"

    def stop(self):
        self._running = False


# ── Interactive ground station terminal ──────────────────────────────────────

def print_banner():
    print()
    print("  ╔═══════════════════════════════════════════════════════╗")
    print("  ║       SLARS-BCG-1  GROUND STATION TERMINAL           ║")
    print("  ║       SLARS Team — Bandaranayake College, Gampaha    ║")
    print("  ╚═══════════════════════════════════════════════════════╝")
    print()
    print("  Simulated satellite running. Type 'help' for commands.")
    print()


def print_help():
    print()
    print("  ┌─────────────────────────────────────────────────────┐")
    print("  │  Available commands                                 │")
    print("  ├─────────────────────────────────────────────────────┤")
    print("  │  tm          — Request and display TM health report │")
    print("  │  safe        — Set satellite to SAFE mode           │")
    print("  │  nominal     — Set satellite to NOMINAL mode        │")
    print("  │  science     — Set satellite to SCIENCE mode        │")
    print("  │  beacon      — Force immediate beacon TX            │")
    print("  │  passes N    — Set max science passes (1-6)         │")
    print("  │  adcs N      — Set ADCS mode (1=Bdot 2=PD 3=PID)   │")
    print("  │  desat       — Trigger RW desaturation              │")
    print("  │  fdir        — Read FDIR event log                  │")
    print("  │  aprs clear  — Clear APRS message buffer            │")
    print("  │  status      — Show satellite status summary        │")
    print("  │  help        — Show this help                       │")
    print("  │  quit        — Exit ground station                  │")
    print("  └─────────────────────────────────────────────────────┘")
    print()


def run_ground_station():
    print_banner()

    sat = SimulatedSatellite()
    tc_count = 0

    # Give the satellite a moment to initialise
    time.sleep(0.5)

    while True:
        try:
            cmd = input("  GS> ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            print("\n  Ground station shutdown.")
            break

        if not cmd:
            continue

        parts = cmd.split()
        verb  = parts[0]
        args  = parts[1:]

        # ── TM request ──────────────────────────────────────────
        if verb == "tm":
            frame = sat.build_tm_frame()
            tm    = decode_tm_frame(frame)
            print_tm_frame(tm)

        # ── Mode commands ────────────────────────────────────────
        elif verb == "safe":
            tc = tc_set_mode(0)
            result = sat.process_tc(tc)
            tc_count += 1
            print(f"\n  [TC #{tc_count}] SET_MODE -> {result}\n")

        elif verb == "nominal":
            tc = tc_set_mode(1)
            result = sat.process_tc(tc)
            tc_count += 1
            print(f"\n  [TC #{tc_count}] SET_MODE -> {result}\n")

        elif verb == "science":
            tc = tc_set_mode(2)
            result = sat.process_tc(tc)
            tc_count += 1
            print(f"\n  [TC #{tc_count}] SET_MODE -> {result}\n")

        # ── Beacon ──────────────────────────────────────────────
        elif verb == "beacon":
            tc = tc_force_beacon()
            result = sat.process_tc(tc)
            tc_count += 1
            print(f"\n  [TC #{tc_count}] FORCE_BEACON: {result}")
            frame = sat.build_tm_frame()
            tm    = decode_tm_frame(frame)
            print_tm_frame(tm)

        # ── Passes ──────────────────────────────────────────────
        elif verb == "passes":
            if args and args[0].isdigit():
                n = int(args[0])
                tc = tc_set_max_passes(n)
                result = sat.process_tc(tc)
                tc_count += 1
                print(f"\n  [TC #{tc_count}] SET_MAX_PASSES -> {result}\n")
            else:
                print("  Usage: passes N  (N = 1 to 6)")

        # ── ADCS mode ────────────────────────────────────────────
        elif verb == "adcs":
            if args and args[0].isdigit():
                m = int(args[0])
                tc = tc_set_adcs_mode(m)
                result = sat.process_tc(tc)
                tc_count += 1
                print(f"\n  [TC #{tc_count}] SET_ADCS_MODE -> {result}\n")
            else:
                print("  Usage: adcs N  (N = 1 B-dot, 2 Coarse, 3 Nadir)")

        # ── Desaturation ─────────────────────────────────────────
        elif verb == "desat":
            tc = tc_trigger_desaturation()
            result = sat.process_tc(tc)
            tc_count += 1
            print(f"\n  [TC #{tc_count}] TRIGGER_DESATURATION: {result}\n")

        # ── FDIR log ─────────────────────────────────────────────
        elif verb == "fdir":
            tc = tc_read_fdir_log(20)
            result = sat.process_tc(tc)
            tc_count += 1
            print(f"\n  [TC #{tc_count}] READ_FDIR_LOG: {result}\n")

        # ── APRS buffer ──────────────────────────────────────────
        elif verb == "aprs" and args and args[0] == "clear":
            tc = tc_clear_aprs_buffer()
            result = sat.process_tc(tc)
            tc_count += 1
            print(f"\n  [TC #{tc_count}] CLEAR_APRS_BUFFER: {result}\n")

        # ── Status summary ───────────────────────────────────────
        elif verb == "status":
            print()
            print(f"  Mode:         {MODE_NAMES.get(sat.mode, sat.mode)}")
            print(f"  Battery SoC:  {sat.bat_soc*100:.1f}%")
            print(f"  ADCS mode:    {ADCS_NAMES.get(sat.adcs_mode, sat.adcs_mode)}")
            print(f"  Pointing:     {sat.pointing_err:.1f} deg")
            print(f"  Angular rate: {sat.ang_rate:.2f} deg/s")
            print(f"  RW speed:     {sat.rw_rpm} RPM")
            print(f"  Orbit:        {sat.orbit_count}")
            print(f"  TCs sent:     {tc_count}")
            print()

        # ── Help ─────────────────────────────────────────────────
        elif verb == "help":
            print_help()

        # ── Quit ─────────────────────────────────────────────────
        elif verb in ("quit", "exit", "q"):
            print("\n  Ground station shutdown. 73 de SLARS-BCG-1.\n")
            sat.stop()
            break

        else:
            print(f"  Unknown command: '{cmd}'. Type 'help' for options.")


# ── Entry point ──────────────────────────────────────────────────────────────

if __name__ == "__main__":
    run_ground_station()
