/*
 * SLARS-BCG-1 — Flash Manager Task
 * Priority 3 | Period 5000ms | Stack 4KB
 * Manages 512 MB eMMC. LittleFS wear levelling.
 *
 * Storage (Step 6 Section 11):
 *   400 MB science imagery
 *    50 MB AIS vessel data
 *    50 MB housekeeping log
 *    12 MB anomaly files
 */
#include "../hal/slars_types.h"

#define SCIENCE_MAX_MB  400
#define AIS_MAX_MB       50
#define HK_MAX_MB        50

static uint32_t science_mb  = 0;
static uint32_t ais_mb      = 0;
static uint32_t hk_frames   = 0;
static uint32_t purge_count = 0;

void flash_manager_task(void) {
    watchdog_task_kick(TASK_FLASH_MANAGER);

    uint8_t health = flash_health_check();
    if (!health) {
        fdir_log("EMMC_HEALTH: failure");
        fdir_report_flash_failure();
    }

    /* Simulate data accumulation */
    science_mb += 4;   /* ~3.5 MB per pass, ~4 MB with overhead */
    hk_frames++;

    float usage = (float)(science_mb + ais_mb) /
                  (float)(SCIENCE_MAX_MB + AIS_MAX_MB) * 100.0f;

    if (usage > 90.0f) {
        purge_count++;
        science_mb = (uint32_t)(science_mb * 0.7f);
        printf("[FLASH] Storage >90%% — purging old files #%lu\n",
               (unsigned long)purge_count);
        fdir_log("FLASH: purging, usage=%.0f%%", usage);
    }

    if (science_mb > SCIENCE_MAX_MB) science_mb = SCIENCE_MAX_MB;

    printf("[FLASH] Science=%luMB/%dMB  HK=%lu frames  Usage=%.1f%%\n",
           (unsigned long)science_mb, SCIENCE_MAX_MB,
           (unsigned long)hk_frames, usage);
}
