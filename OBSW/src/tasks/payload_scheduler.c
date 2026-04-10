/*
 * SLARS-BCG-1 — Payload Scheduler Task
 * Priority 5 | Period 1000ms | Stack 4KB
 *
 * Sri Lanka bounding box (Step 6):
 *   Lat: 5.9N to 9.9N
 *   Lon: 79.7E to 81.9E
 *
 * All 4 science gates must pass before imaging:
 *   1. Over Sri Lanka footprint
 *   2. SoC > 60%
 *   3. ADCS pointing error < 5 degrees
 *   4. passes_today < max_passes
 */
#include "../hal/slars_types.h"

#define SL_LAT_MIN   5.9f
#define SL_LAT_MAX   9.9f
#define SL_LON_MIN  79.7f
#define SL_LON_MAX  81.9f
#define PREAMBLE_S  120

static uint8_t science_active    = 0;
static uint8_t passes_today      = 0;
static uint8_t max_passes_sched  = 3;

static char tle_l1[70] =
    "1 25544U 98067A   21001.00000000  .00000000  00000-0  00000-0 0  9999";
static char tle_l2[70] =
    "2 25544  10.0000  80.0000 0001000   0.0000   0.0000 15.24000000000000";

void payload_scheduler_set_max_passes(uint8_t n) { max_passes_sched = n; }
void payload_scheduler_reset_daily(void)          { passes_today = 0;    }
void payload_scheduler_update_tle(const char *l1, const char *l2) {
    strncpy(tle_l1, l1, 69);
    strncpy(tle_l2, l2, 69);
    printf("[SCHED] TLE updated\n");
}

static uint8_t over_sri_lanka(float lat, float lon) {
    return (lat >= SL_LAT_MIN && lat <= SL_LAT_MAX &&
            lon >= SL_LON_MIN && lon <= SL_LON_MAX);
}

void payload_scheduler_task(void) {
    watchdog_task_kick(TASK_PAYLOAD_SCHED);

    double lat, lon, alt;
    double t_now = (double)rtc_get_unix_time();
    sgp4_propagate(tle_l1, tle_l2, t_now, &lat, &lon, &alt);

    double t_entry = sgp4_time_to_footprint(
        tle_l1, tle_l2,
        SL_LAT_MIN, SL_LAT_MAX,
        SL_LON_MIN, SL_LON_MAX,
        t_now);

    /* Pre-activate ADCS Mode 3 two minutes before pass */
    if (t_entry > 0 && t_entry < PREAMBLE_S) {
        if (adcs_get_mode() != ADCS_MODE_3_NADIR) {
            adcs_set_mode(ADCS_MODE_3_NADIR);
            printf("[SCHED] ADCS Mode 3 activated — T-%.0fs\n", t_entry);
        }
    }

    /* Science window — check all 4 gates */
    if (over_sri_lanka((float)lat, (float)lon)) {
        float   att_err  = adcs_read_pointing_error();
        float   soc      = eps_read_soc();
        uint8_t ok_soc   = (soc > 0.60f);
        uint8_t ok_att   = (att_err < 5.0f);
        uint8_t ok_pass  = (passes_today < max_passes_sched);

        if (ok_soc && ok_att && ok_pass && !science_active) {
            imager_enable();
            sdr_enable_tx();
            science_active = 1;
            printf("[SCHED] SCIENCE WINDOW OPEN lat=%.1f lon=%.1f\n",
                   lat, lon);
            printf("[SCHED]   soc=%.0f%% att=%.1f deg pass=%d/%d\n",
                   soc*100, att_err, passes_today+1, max_passes_sched);

        } else if (!science_active) {
            printf("[SCHED] Gate FAILED over SL:"
                   " soc=%s att=%s pass=%s\n",
                   ok_soc?"OK":"LOW", ok_att?"OK":"POOR",
                   ok_pass?"OK":"MAX");
        }

    } else if (science_active) {
        /* Footprint exit — close science window */
        imager_disable();
        sdr_disable_tx();
        science_active = 0;
        passes_today++;
        comms_trigger_sband_downlink();
        adcs_set_mode(ADCS_MODE_2_COARSE);
        printf("[SCHED] SCIENCE WINDOW CLOSED — pass %d done\n",
               passes_today);
    }

    ais_log_update();

    printf("[SCHED] lat=%.2f lon=%.2f T-next=%.0fs passes=%d/%d\n",
           lat, lon, t_entry, passes_today, max_passes_sched);
}
