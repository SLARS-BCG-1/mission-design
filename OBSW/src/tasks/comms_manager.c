/*
 * SLARS-BCG-1 — COMMS Manager Task
 * Priority 5 | Period 500ms | Stack 4KB
 * UHF beacon every 30s. S-band CFDP downlink. TC receive.
 * HARD RULE: UHF PA and S-band TX never simultaneous.
 */
#include "../hal/slars_types.h"

#define BEACON_INTERVAL_S  30

static uint32_t last_beacon_time = 0;
static uint8_t  sband_pending    = 0;
static uint32_t beacon_count     = 0;
static uint8_t  uhf_active       = 0;

uint8_t comms_uhf_beacon_confirmed(void) {
    uint32_t now = (uint32_t)time(NULL);
    return (now - last_beacon_time) < 35;
}

void comms_trigger_sband_downlink(void) {
    sband_pending = 1;
}

void comms_update_doppler_quaternion(const float q[4]) {
    (void)q;
}

void comms_manager_task(void) {
    watchdog_task_kick(TASK_COMMS_MANAGER);
    uint32_t now = (uint32_t)time(NULL);

    /* Beacon every 30 seconds — ALWAYS even in safe mode */
    if (now - last_beacon_time >= BEACON_INTERVAL_S) {
    /* NEW — use the proper TM encoder */
        uint8_t frame[128];
        tm_encode_housekeeping(frame);
        power_notify_uhf_pa(1);
        uhf_active = 1;
        uhf_transmit(frame, 128);
        uhf_active = 0;
        power_notify_uhf_pa(0);

        last_beacon_time = now;
        beacon_count++;
        printf("[COMMS] Beacon #%lu TX — soc=%.0f%% mode=%d\n",
               (unsigned long)beacon_count,
               eps_read_soc()*100, power_get_mode());
    }

    /* S-band CFDP downlink after each science pass */
    if (sband_pending && !uhf_active) {
        int32_t dop = (int32_t)(adcs_get_radial_velocity_ms()
                      / 299792458.0f * 2401000000.0f);
        sband_set_frequency_offset(dop);
        power_notify_sband_tx(1);
        cfdp_send_pending_files();
        power_notify_sband_tx(0);
        sband_pending = 0;
        printf("[COMMS] S-band CFDP complete. Doppler=%d Hz\n", dop);
    }

    /* Check for incoming TC */
    uint8_t buf[64];
    uint8_t n = uhf_receive(buf, 64);
    if (n > 0) tc_dispatch(buf, n);
}
