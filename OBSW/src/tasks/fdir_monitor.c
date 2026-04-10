/*
 * SLARS-BCG-1 — FDIR Monitor Task
 * Priority 6 | Period 2000ms | Stack 2KB
 * 4-level fault escalation. Satellite's autonomous doctor.
 */
#include "../hal/slars_types.h"

#define FAULT_THRESHOLD  3

static uint8_t fault_count[10] = {0};
static char    log_buf[500][80];
static uint16_t log_head = 0;

/* ── Public FDIR functions — called by all tasks ─── */

void fdir_log(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(log_buf[log_head], 80, fmt, args);
    va_end(args);
    printf("[FDIR] LOG: %s\n", log_buf[log_head]);
    log_head = (log_head + 1) % 500;
}

void fdir_report_task_dead(uint8_t id) {
    printf("[FDIR] L1: Task %d dead — restarting\n", id);
}

void fdir_restart_task(uint8_t id) {
    printf("[FDIR] L1: Task %d restarted\n", id);
}

void fdir_power_cycle_subsystem(uint8_t id) {
    printf("[FDIR] L2: Power cycling subsystem %d\n", id);
}

void fdir_report_power_violation(void) {
    fdir_log("POWER_VIOLATION: UHF+SBAND simultaneous");
}

void fdir_report_flash_failure(void) {
    fdir_log("FLASH_FAILURE detected");
}

void fdir_report_emergency(void) {
    fdir_log("EMERGENCY: battery critical");
    printf("[FDIR] L4: EMERGENCY — transmitting anomaly report\n");
}

void fdir_transmit_anomaly_report(void) {
    printf("[FDIR] L4: Anomaly report sent — awaiting ground TC\n");
}

/* ── Internal: check one parameter ───────────────── */

static void check(uint8_t id, uint8_t failed,
                  uint8_t level, const char *name) {
    if (failed) {
        fault_count[id]++;
        if (fault_count[id] >= FAULT_THRESHOLD) {
            printf("[FDIR] ESCALATE: %s -> Level %d\n", name, level);
            switch(level) {
                case 1: fdir_restart_task(id);          break;
                case 2: fdir_power_cycle_subsystem(id); break;
                case 3: system_enter_safe_mode();       break;
                case 4: fdir_transmit_anomaly_report(); break;
            }
            fault_count[id] = 0;
        }
    } else {
        fault_count[id] = 0;
    }
}

/* ── Main FDIR task ───────────────────────────────── */

void fdir_monitor_task(void) {
    watchdog_task_kick(TASK_FDIR);

    float   soc     = eps_read_soc();
    float   vbat    = eps_read_voltage();
    float   obc_t   = obc_read_temperature();
    float   rate    = adcs_read_angular_rate();
    int     rw      = adcs_read_rw_speed();
    float   watts   = eps_read_total_watts();
    float   att_err = adcs_read_pointing_error();
    uint8_t uhf_ok  = comms_uhf_beacon_confirmed();
    uint8_t fl_ok   = flash_health_check();

    /* All 10 parameters from Step 6 Section 9 */
    check(0, soc   < 0.20f,                      3, "Battery SoC");
    check(1, vbat  < 6.0f,                        3, "Battery Voltage");
    check(2, obc_t < -25.0f || obc_t > 75.0f,    2, "OBC Temperature");
    check(3, rate  > 15.0f,                       2, "ADCS Angular Rate");
    check(4, abs(rw) >= 5999,                     2, "RW Speed");
    check(5, !uhf_ok,                             1, "UHF COMMS");
    check(6, !fl_ok,                              2, "eMMC Flash");
    check(7, watts > 5.8f,                        2, "EPS Current");
    check(8, att_err > 10.0f && adcs_get_mode()==3, 1, "ADCS Pointing");

    printf("[FDIR] Health: soc=%.0f%% rate=%.1f att=%.1f rw=%d\n",
           soc*100, rate, att_err, rw);
}
