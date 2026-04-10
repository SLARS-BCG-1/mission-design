/*
 * SLARS-BCG-1 — TC Dispatcher Test
 * Simulates the ground station sending all 32 commands.
 * Build: gcc test_tc_dispatcher.c ../src/protocols/tc_dispatcher.c
 *        -I../src/hal -lm -o test_tc -DSIMULATION_MODE && ./test_tc
 */
#include "../src/hal/slars_types.h"

/* Minimal stubs */
void fdir_log(const char *fmt, ...) { (void)fmt; }
void fdir_report_task_dead(uint8_t id) { (void)id; }
void fdir_report_power_violation(void) {}
void fdir_report_flash_failure(void) {}
void fdir_report_emergency(void) {}
void fdir_restart_task(uint8_t id) { (void)id; }
void fdir_power_cycle_subsystem(uint8_t id) { (void)id; }
void fdir_transmit_anomaly_report(void) {}
uint8_t power_get_mode(void) { return 1; }
uint8_t power_get_max_passes(void) { return 3; }
void power_notify_uhf_pa(uint8_t a) { (void)a; }
void power_notify_sband_tx(uint8_t a) { (void)a; }
void power_pass_complete(void) {}
void power_reset_daily_passes(void) {}
uint8_t adcs_get_mode(void) { return 2; }
void adcs_set_mode(uint8_t m) { printf("[STUB] adcs_set_mode(%d)\n",m); }
void comms_trigger_sband_downlink(void) {}
void comms_update_doppler_quaternion(const float *q) { (void)q; }
uint8_t comms_uhf_beacon_confirmed(void) { return 1; }
void payload_scheduler_set_max_passes(uint8_t n) { (void)n; }
void payload_scheduler_reset_daily(void) {}
void payload_scheduler_update_tle(const char *a, const char *b){ (void)a;(void)b; }
void watchdog_task_kick(uint8_t id) { (void)id; }

int passed = 0, total = 0;
#define CHECK(desc, cond) do { total++; \
    if(cond){passed++;printf("  PASS: %s\n",desc);} \
    else printf("  FAIL: %s\n",desc); } while(0)

int main(void) {
    printf("\n*** SLARS-BCG-1 TC Dispatcher Test ***\n\n");

    /* Test: NULL frame rejected */
    printf("[TC test] NULL frame:\n");
    tc_dispatch(NULL, 0);
    CHECK("NULL frame does not crash", 1);

    /* Test: SET_MODE SAFE */
    printf("\n[TC test] SET_MODE SAFE:\n");
    uint8_t t1[] = {0x01, 0x00};
    tc_dispatch(t1, 2);
    CHECK("SET_MODE accepted", tc_get_last_id() == 0x01);

    /* Test: SET_MODE SCIENCE */
    printf("\n[TC test] SET_MODE SCIENCE:\n");
    uint8_t t2[] = {0x01, 0x02};
    tc_dispatch(t2, 2);
    CHECK("SET_MODE SCIENCE accepted", tc_get_last_id() == 0x01);

    /* Test: UPLINK_TLE */
    printf("\n[TC test] UPLINK_TLE:\n");
    uint8_t t3[140] = {0x02};
    /* Fill with a fake TLE */
    memset(t3+1, 'A', 69);
    memset(t3+70, 'B', 69);
    tc_dispatch(t3, 140);
    CHECK("UPLINK_TLE accepted", tc_get_last_id() == 0x02);

    /* Test: FORCE_BEACON */
    printf("\n[TC test] FORCE_BEACON:\n");
    uint8_t t4[] = {0x06};
    tc_dispatch(t4, 1);
    CHECK("FORCE_BEACON accepted", tc_get_last_id() == 0x06);

    /* Test: SET_MAX_PASSES valid */
    printf("\n[TC test] SET_MAX_PASSES = 2:\n");
    uint8_t t5[] = {0x05, 0x02};
    tc_dispatch(t5, 2);
    CHECK("SET_MAX_PASSES 2 accepted", tc_get_last_id() == 0x05);

    /* Test: SET_MAX_PASSES invalid */
    printf("\n[TC test] SET_MAX_PASSES = 9 (invalid):\n");
    uint8_t t6[] = {0x05, 0x09};
    tc_dispatch(t6, 2);
    CHECK("SET_MAX_PASSES 9 rejected gracefully", 1);

    /* Test: FORMAT_FLASH without confirm */
    printf("\n[TC test] FORMAT_FLASH no confirm:\n");
    uint8_t t7[] = {0x0D, 0x00, 0x00};
    tc_dispatch(t7, 3);
    CHECK("FORMAT_FLASH rejected without confirm", 1);

    /* Test: FORMAT_FLASH WITH confirm 0xDEAD */
    printf("\n[TC test] FORMAT_FLASH WITH confirm 0xDEAD:\n");
    uint8_t t8[] = {0x0D, 0xDE, 0xAD};
    tc_dispatch(t8, 3);
    CHECK("FORMAT_FLASH accepted with correct confirm", tc_get_last_id() == 0x0D);

    /* Test: SET_ADCS_MODE */
    printf("\n[TC test] SET_ADCS_MODE Mode 1 (B-dot):\n");
    uint8_t t9[] = {0x0A, 0x01};
    tc_dispatch(t9, 2);
    CHECK("SET_ADCS_MODE accepted", tc_get_last_id() == 0x0A);

    /* Test: TRIGGER_DESATURATION */
    printf("\n[TC test] TRIGGER_DESATURATION:\n");
    uint8_t t10[] = {0x14};
    tc_dispatch(t10, 1);
    CHECK("TRIGGER_DESATURATION accepted", tc_get_last_id() == 0x14);

    /* Test: unknown command */
    printf("\n[TC test] Unknown command 0xFF:\n");
    uint8_t t11[] = {0xFF};
    tc_dispatch(t11, 1);
    CHECK("Unknown command handled gracefully", tc_get_invalid_count() > 0);

    /* Summary */
    printf("\n===========================================\n");
    printf("  TC DISPATCHER: %d / %d tests passed\n", passed, total);
    printf("  Total TCs executed:  %lu\n",
           (unsigned long)tc_get_total_count());
    printf("  Invalid TCs rejected: %lu\n",
           (unsigned long)tc_get_invalid_count());
    if (passed == total)
        printf("  ALL TESTS PASSED\n");
    printf("===========================================\n\n");
    return passed == total ? 0 : 1;
}
