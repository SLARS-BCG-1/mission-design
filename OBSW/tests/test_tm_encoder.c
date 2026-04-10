/*
 * SLARS-BCG-1 — Unit Tests: TM Frame Encoder
 * Tests every field in the 128-byte housekeeping frame.
 * Verifies encode-then-decode gives back the same values.
 * Verifies CRC detects corrupted frames.
 *
 * Build:
 *   gcc test_tm_encoder.c ../src/protocols/tm_encoder.c \
 *       -I../src/hal -lm -o test_tm -DSIMULATION_MODE && ./test_tm
 *
 * SLARS Team — Bandaranayake College, Gampaha
 */

#include "../src/hal/sim_hal.h"
#include "../src/hal/slars_types.h"
#include <assert.h>
#include <math.h>

/* Minimal stubs for isolated testing */
void fdir_log(const char *fmt, ...) { (void)fmt; }
void fdir_report_task_dead(uint8_t id) { (void)id; }
void fdir_report_power_violation(void) {}
void fdir_report_flash_failure(void) {}
void fdir_report_emergency(void) {}
void fdir_restart_task(uint8_t id) { (void)id; }
void fdir_power_cycle_subsystem(uint8_t id) { (void)id; }
void fdir_transmit_anomaly_report(void) {}
void comms_trigger_sband_downlink(void) {}
void comms_update_doppler_quaternion(const float *q) { (void)q; }
uint8_t comms_uhf_beacon_confirmed(void) { return 1; }
void tc_dispatch(const uint8_t *b, uint8_t l) { (void)b; (void)l; }
void payload_scheduler_set_max_passes(uint8_t n) { (void)n; }
void payload_scheduler_reset_daily(void) {}
void payload_scheduler_update_tle(const char *a, const char *b) {
    (void)a; (void)b;
}
void watchdog_task_kick(uint8_t id) { (void)id; }
uint8_t adcs_get_mode(void)  { return sim_adcs_mode; }
void    adcs_set_mode(uint8_t m) { sim_adcs_mode = m; }
uint8_t power_get_mode(void)     { return sim_system_mode; }
uint8_t power_get_max_passes(void) { return 3; }
void power_notify_uhf_pa(uint8_t a) { (void)a; }
void power_notify_sband_tx(uint8_t a) { (void)a; }
void power_pass_complete(void) {}
void power_reset_daily_passes(void) {}

/* ── Test framework ──────────────────────────────────────────────────────── */
static int tests_run    = 0;
static int tests_passed = 0;

#define CLOSE_ENOUGH(a, b, tol) (fabsf((a)-(b)) <= (tol))

#define TEST(name, condition) do { \
    tests_run++; \
    if (condition) { \
        printf("  PASS: %s\n", name); \
        tests_passed++; \
    } else { \
        printf("  FAIL: %s\n", name); \
    } \
} while(0)

/* ── Test 1: Frame is exactly 128 bytes ─────────────────────────────────── */
void test_frame_length(void) {
    printf("\n[TEST] Frame length\n");
    uint8_t buf[128];
    uint8_t n = tm_encode_housekeeping(buf);
    TEST("Encode returns 128", n == 128);
}

/* ── Test 2: CSP header is correct ──────────────────────────────────────── */
void test_csp_header(void) {
    printf("\n[TEST] CSP header\n");
    uint8_t buf[128] = {0};
    tm_encode_housekeeping(buf);

    /* Reconstruct the 32-bit CSP header */
    uint32_t hdr = ((uint32_t)buf[0] << 24) |
                   ((uint32_t)buf[1] << 16) |
                   ((uint32_t)buf[2] <<  8) |
                    (uint32_t)buf[3];

    uint8_t src      = (hdr >> 26) & 0x3F;
    uint8_t dst      = (hdr >> 20) & 0x3F;
    uint8_t dst_port = (hdr >> 14) & 0x3F;
    uint8_t src_port = (hdr >>  8) & 0x3F;
    uint8_t flags    =  hdr        & 0xFF;

    TEST("CSP src address = 5 (SLARS satellite)",  src      == 5);
    TEST("CSP dst address = 1 (ground station)",   dst      == 1);
    TEST("CSP dst port = 9 (TM port)",             dst_port == 9);
    TEST("CSP src port = 10 (OBC)",                src_port == 10);
    TEST("CSP CRC flag set (0x01)",                flags    == 0x01);
}

/* ── Test 3: Frame type and sequence counter ─────────────────────────────── */
void test_frame_type_and_seq(void) {
    printf("\n[TEST] Frame type and sequence counter\n");
    uint8_t buf1[128], buf2[128], buf3[128];
    uint8_t seq_before = tm_get_seq_counter();
    tm_encode_housekeeping(buf1);
    tm_encode_housekeeping(buf2);
    tm_encode_housekeeping(buf3);

    TEST("Frame type is 0x01 (housekeeping)", buf1[4] == 0x01);
    TEST("Sequence counter increments each frame",
         buf2[5] == buf1[5] + 1);
    TEST("Sequence counter increments again",
         buf3[5] == buf2[5] + 1);
    TEST("tm_get_seq_counter advances",
         tm_get_seq_counter() > seq_before);
}

/* ── Test 4: Battery SoC encoding ───────────────────────────────────────── */
void test_battery_soc(void) {
    printf("\n[TEST] Battery SoC encoding\n");
    uint8_t buf[128];
    tm_decoded_t dec;

    /* Test at 85% SoC */
    sim_battery_soc = 0.85f;
    tm_encode_housekeeping(buf);
    tm_decode_housekeeping(buf, &dec);
    TEST("SoC 85%: encoded and decoded within 0.01%",
         CLOSE_ENOUGH(dec.bat_soc_pct, 85.0f, 0.01f));

    /* Test at 20% SoC (safe mode threshold) */
    sim_battery_soc = 0.20f;
    tm_encode_housekeeping(buf);
    tm_decode_housekeeping(buf, &dec);
    TEST("SoC 20%: exactly at safe mode threshold",
         CLOSE_ENOUGH(dec.bat_soc_pct, 20.0f, 0.01f));

    /* Test at 0% */
    sim_battery_soc = 0.0f;
    tm_encode_housekeeping(buf);
    tm_decode_housekeeping(buf, &dec);
    TEST("SoC 0%: empty battery encodes correctly",
         CLOSE_ENOUGH(dec.bat_soc_pct, 0.0f, 0.01f));

    /* Test at 100% */
    sim_battery_soc = 1.0f;
    tm_encode_housekeeping(buf);
    tm_decode_housekeeping(buf, &dec);
    TEST("SoC 100%: full battery encodes correctly",
         CLOSE_ENOUGH(dec.bat_soc_pct, 100.0f, 0.01f));

    sim_battery_soc = 0.85f; /* restore */
}

/* ── Test 5: Battery voltage encoding ───────────────────────────────────── */
void test_battery_voltage(void) {
    printf("\n[TEST] Battery voltage encoding\n");
    uint8_t buf[128];
    tm_decoded_t dec;

    /* Voltage = 6.0 + (soc * 2.4) */
    sim_battery_soc = 0.75f;  /* -> 6.0 + 1.8 = 7.8V */
    tm_encode_housekeeping(buf);
    tm_decode_housekeeping(buf, &dec);
    TEST("Voltage 7.8V: millivolt encoding within 0.001V",
         CLOSE_ENOUGH(dec.bat_voltage_v, 7.8f, 0.001f));

    sim_battery_soc = 0.0f;  /* -> 6.0V (minimum) */
    tm_encode_housekeeping(buf);
    tm_decode_housekeeping(buf, &dec);
    TEST("Voltage 6.0V: empty battery voltage correct",
         CLOSE_ENOUGH(dec.bat_voltage_v, 6.0f, 0.001f));

    sim_battery_soc = 0.85f; /* restore */
}

/* ── Test 6: Temperature encoding ───────────────────────────────────────── */
void test_temperatures(void) {
    printf("\n[TEST] Temperature encoding (+40 offset)\n");
    uint8_t buf[128];
    tm_decoded_t dec;

    sim_battery_temp = 22.0f;
    tm_encode_housekeeping(buf);
    tm_decode_housekeeping(buf, &dec);
    TEST("Battery 22C: round-trips correctly",
         dec.bat_temp_c == 22);

    sim_battery_temp = -10.0f;
    tm_encode_housekeeping(buf);
    tm_decode_housekeeping(buf, &dec);
    TEST("Battery -10C: negative temperature encodes correctly",
         dec.bat_temp_c == -10);

    sim_obc_temp = 35.0f;
    tm_encode_housekeeping(buf);
    tm_decode_housekeeping(buf, &dec);
    TEST("OBC 35C: round-trips correctly",
         dec.obc_temp_c == 35);

    sim_battery_temp = 22.0f; /* restore */
}

/* ── Test 7: ADCS fields ─────────────────────────────────────────────────── */
void test_adcs_fields(void) {
    printf("\n[TEST] ADCS field encoding\n");
    uint8_t buf[128];
    tm_decoded_t dec;

    sim_adcs_mode    = 3;    /* Nadir PID */
    sim_pointing_err = 4.5f;
    sim_angular_rate = 0.8f;
    sim_rw_speed     = 2500;

    tm_encode_housekeeping(buf);
    tm_decode_housekeeping(buf, &dec);

    TEST("ADCS mode 3 (Nadir PID) encodes correctly",
         dec.adcs_mode == 3);
    TEST("Pointing error 4.5 deg: within 0.5 deg resolution",
         CLOSE_ENOUGH(dec.pointing_deg, 4.5f, 0.5f));
    TEST("Angular rate 0.8 deg/s: within 0.25 deg/s resolution",
         CLOSE_ENOUGH(dec.rate_degs, 0.8f, 0.25f));
    TEST("RW speed 2500 RPM: exact int16 round-trip",
         dec.rw_rpm == 2500);

    /* Test negative RW speed */
    sim_rw_speed = -1800;
    tm_encode_housekeeping(buf);
    tm_decode_housekeeping(buf, &dec);
    TEST("RW speed -1800 RPM: negative RPM encodes correctly",
         dec.rw_rpm == -1800);

    /* Restore */
    sim_adcs_mode    = 1;
    sim_pointing_err = 45.0f;
    sim_angular_rate = 5.2f;
    sim_rw_speed     = 0;
}

/* ── Test 8: System mode ─────────────────────────────────────────────────── */
void test_system_mode(void) {
    printf("\n[TEST] System mode encoding\n");
    uint8_t buf[128];
    tm_decoded_t dec;

    sim_system_mode = 0;  /* SAFE */
    tm_encode_housekeeping(buf);
    tm_decode_housekeeping(buf, &dec);
    TEST("SAFE mode (0) encodes correctly",   dec.system_mode == 0);

    sim_system_mode = 1;  /* NOMINAL */
    tm_encode_housekeeping(buf);
    tm_decode_housekeeping(buf, &dec);
    TEST("NOMINAL mode (1) encodes correctly", dec.system_mode == 1);

    sim_system_mode = 2;  /* SCIENCE */
    tm_encode_housekeeping(buf);
    tm_decode_housekeeping(buf, &dec);
    TEST("SCIENCE mode (2) encodes correctly", dec.system_mode == 2);

    sim_system_mode = 0;
}

/* ── Test 9: CRC16 integrity ─────────────────────────────────────────────── */
void test_crc_integrity(void) {
    printf("\n[TEST] CRC16-CCITT integrity\n");
    uint8_t buf[128];
    tm_decoded_t dec;

    tm_encode_housekeeping(buf);

    /* Valid frame should decode successfully */
    uint8_t ok = tm_decode_housekeeping(buf, &dec);
    TEST("Valid frame: CRC passes", ok == 1);
    TEST("Valid frame: crc_valid flag set", dec.crc_valid == 1);

    /* Corrupt byte 10 (SoC high byte) — CRC must catch this */
    buf[10] ^= 0xFF;
    ok = tm_decode_housekeeping(buf, &dec);
    TEST("Corrupted frame: CRC fails and decode returns 0", ok == 0);

    /* Corrupt the CRC itself — still invalid */
    tm_encode_housekeeping(buf);
    buf[126] ^= 0x01;
    ok = tm_decode_housekeeping(buf, &dec);
    TEST("Corrupted CRC byte: decode returns 0", ok == 0);

    /* Corrupt last data byte (byte 125) */
    tm_encode_housekeeping(buf);
    buf[125] = 0xFF;
    ok = tm_decode_housekeeping(buf, &dec);
    TEST("Corrupted byte 125: CRC catches it", ok == 0);
}

/* ── Test 10: NULL pointer safety ────────────────────────────────────────── */
void test_null_safety(void) {
    printf("\n[TEST] NULL pointer safety\n");
    uint8_t buf[128];
    tm_decoded_t dec;

    uint8_t r1 = tm_encode_housekeeping(NULL);
    TEST("Encode NULL buf returns 0 without crash", r1 == 0);

    tm_encode_housekeeping(buf);
    uint8_t r2 = tm_decode_housekeeping(NULL, &dec);
    TEST("Decode NULL buf returns 0 without crash", r2 == 0);

    uint8_t r3 = tm_decode_housekeeping(buf, NULL);
    TEST("Decode NULL out returns 0 without crash", r3 == 0);
}

/* ── Test 11: Frame counter ──────────────────────────────────────────────── */
void test_frame_counter(void) {
    printf("\n[TEST] Frame counter\n");
    uint32_t before = tm_get_frames_encoded();
    uint8_t buf[128];
    tm_encode_housekeeping(buf);
    tm_encode_housekeeping(buf);
    tm_encode_housekeeping(buf);
    TEST("Frame counter increments by 3",
         tm_get_frames_encoded() == before + 3);
}

/* ── Test 12: Print a decoded frame (visual check) ───────────────────────── */
void test_print_frame(void) {
    printf("\n[TEST] Decoded frame contents (visual check)\n");
    sim_battery_soc  = 0.82f;
    sim_battery_temp = 24.0f;
    sim_obc_temp     = 38.0f;
    sim_adcs_mode    = 2;
    sim_pointing_err = 8.5f;
    sim_angular_rate = 0.3f;
    sim_rw_speed     = 1200;
    sim_system_mode  = 1;
    sim_total_draw   = 1.85f;
    sim_solar_power  = 2.0f;

    uint8_t buf[128];
    tm_decoded_t dec;
    tm_encode_housekeeping(buf);
    tm_decode_housekeeping(buf, &dec);

    printf("\n  === Decoded TM Frame ===\n");
    printf("  Frame type:     0x%02X\n",  dec.frame_type);
    printf("  Seq counter:    %d\n",       dec.seq_counter);
    printf("  MET:            %d s\n",     dec.met_seconds);
    printf("  System mode:    %d (%s)\n",  dec.system_mode,
           dec.system_mode==0?"SAFE":dec.system_mode==1?"NOMINAL":"SCIENCE");
    printf("  Battery SoC:    %.2f%%\n",   dec.bat_soc_pct);
    printf("  Battery V:      %.3f V\n",   dec.bat_voltage_v);
    printf("  Battery temp:   %d C\n",     dec.bat_temp_c);
    printf("  Solar power:    %.3f W\n",   dec.solar_w);
    printf("  Total draw:     %.3f W\n",   dec.draw_w);
    printf("  OBC temp:       %d C\n",     dec.obc_temp_c);
    printf("  ADCS mode:      %d\n",       dec.adcs_mode);
    printf("  Pointing err:   %.1f deg\n", dec.pointing_deg);
    printf("  Angular rate:   %.2f deg/s\n",dec.rate_degs);
    printf("  RW speed:       %d RPM\n",   dec.rw_rpm);
    printf("  Orbit count:    %lu\n",       (unsigned long)dec.orbit_count);
    printf("  Passes today:   %d\n",        dec.passes_today);
    printf("  APRS queue:     %d\n",        dec.aprs_queue);
    printf("  CRC valid:      %s\n",        dec.crc_valid ? "YES" : "NO");
    printf("  ========================\n\n");

    TEST("Full frame decode: CRC valid", dec.crc_valid == 1);
    TEST("Full frame decode: SoC 82% within tolerance",
         CLOSE_ENOUGH(dec.bat_soc_pct, 82.0f, 0.01f));
    TEST("Full frame decode: NOMINAL mode correct",
         dec.system_mode == 1);
}

/* ── main ────────────────────────────────────────────────────────────────── */
int main(void) {
    printf("*******************************************************\n");
    printf("*   SLARS-BCG-1 TM Encoder Unit Test Suite            *\n");
    printf("*   SLARS Team — Bandaranayake College, Gampaha       *\n");
    printf("*******************************************************\n");

    test_frame_length();
    test_csp_header();
    test_frame_type_and_seq();
    test_battery_soc();
    test_battery_voltage();
    test_temperatures();
    test_adcs_fields();
    test_system_mode();
    test_crc_integrity();
    test_null_safety();
    test_frame_counter();
    test_print_frame();

    printf("=======================================================\n");
    printf("  RESULTS: %d / %d tests passed\n", tests_passed, tests_run);
    if (tests_passed == tests_run)
        printf("  ALL TESTS PASSED\n");
    else
        printf("  %d FAILED\n", tests_run - tests_passed);
    printf("=======================================================\n\n");
    return (tests_passed == tests_run) ? 0 : 1;
}
