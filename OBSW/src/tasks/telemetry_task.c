/*
 * SLARS-BCG-1 — Telemetry Task
 * Priority 4 | Period 10000ms | Stack 2KB
 * Assembles full housekeeping TM. Logs to flash. Resets daily counters.
 */
#include "../hal/slars_types.h"

typedef struct {
    float    bat_soc, bat_voltage, bat_temp;
    float    solar_power, total_draw;
    float    obc_temp;
    uint8_t  system_mode;
    uint32_t orbit_count, uptime_s;
    uint8_t  adcs_mode;
    float    pointing_error, angular_rate;
    int16_t  rw_speed;
    uint8_t  passes_today;
    uint8_t  aprs_queue;
    uint32_t unix_time;
} tm_hk_t;

static tm_hk_t tm;
static uint32_t tm_count = 0;

void telemetry_task(void) {
    watchdog_task_kick(TASK_TELEMETRY);

    tm.bat_soc        = eps_read_soc();
    tm.bat_voltage    = eps_read_voltage();
    tm.bat_temp       = eps_read_battery_temp();
    tm.solar_power    = eps_read_solar_power();
    tm.total_draw     = eps_read_total_watts();
    tm.obc_temp       = obc_read_temperature();
    tm.system_mode    = power_get_mode();
    tm.orbit_count    = orbit_get_count();
    tm.uptime_s       = (uint32_t)time(NULL);
    tm.adcs_mode      = adcs_get_mode();
    tm.pointing_error = adcs_read_pointing_error();
    tm.angular_rate   = adcs_read_angular_rate();
    tm.rw_speed       = (int16_t)adcs_read_rw_speed();
    tm.aprs_queue     = aprs_get_queue_length();
    tm.unix_time      = rtc_get_unix_time();

    flash_write_tm_frame(&tm, sizeof(tm));
    tm_count++;

    printf("[TM]   ── Frame #%lu ─────────────────────────\n",
           (unsigned long)tm_count);
    printf("[TM]   Bat: soc=%.0f%%  V=%.2fV  T=%.1fC\n",
           tm.bat_soc*100, tm.bat_voltage, tm.bat_temp);
    printf("[TM]   Power: solar=%.2fW  draw=%.2fW\n",
           tm.solar_power, tm.total_draw);
    printf("[TM]   Mode: %s  Orbit=%lu  Up=%lus\n",
           tm.system_mode==0?"SAFE":
           tm.system_mode==1?"NOMINAL":"SCIENCE",
           (unsigned long)tm.orbit_count,
           (unsigned long)tm.uptime_s);
    printf("[TM]   ADCS: mode=%d  err=%.1fdeg  rate=%.2fdeg/s\n",
           tm.adcs_mode, tm.pointing_error, tm.angular_rate);
    printf("[TM]   RW: %d RPM  APRS queue: %d msgs\n",
           tm.rw_speed, tm.aprs_queue);
    printf("[TM]   ────────────────────────────────────────\n");

    /* Reset daily counters at UTC midnight */
    uint32_t utc = rtc_get_unix_time();
    if ((utc % 86400) < 10) {
        power_reset_daily_passes();
        payload_scheduler_reset_daily();
        printf("[TM]   Daily counters reset\n");
    }
}
