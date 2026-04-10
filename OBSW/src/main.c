/*
 * SLARS-BCG-1 Flight Software — Main Entry Point
 * Bandaranayake College, Gampaha, Sri Lanka
 *
 * SIMULATION BUILD — runs all 8 tasks on your laptop.
 * On real OBC: FreeRTOS scheduler runs tasks concurrently.
 *
 * Boot sequence (Step 6 Section 12):
 *   Stage 1: Hardware init
 *   Stage 2: All 8 tasks initialised
 *   Stage 3: Subsystem health check
 *   Stage 4: TLE + IGRF load
 *   Stage 5: Mode determination
 *   Stage 6: Nominal operation begins
 */

#include "hal/slars_types.h"
#include <unistd.h>

int main(void) {

    /* ── BOOT BANNER ─────────────────────────────────── */
    printf("\n");
    printf("*******************************************************\n");
    printf("*       SLARS-BCG-1 FLIGHT SOFTWARE v1.0             *\n");
    printf("*   Bandaranayake College, Gampaha, Sri Lanka         *\n");
    printf("*                                                     *\n");
    printf("*   MISSION: Flood mapping | Emergency relay | AIS   *\n");
    printf("*   ORBIT:   500 km LEO | 10 deg | 5-6 passes/day    *\n");
    printf("*   PURPOSE: Protect Sri Lanka from disasters         *\n");
    printf("*******************************************************\n\n");

    /* ── STAGE 1: Hardware init ──────────────────────── */
    printf("[BOOT] Stage 1: Hardware init...\n");
    hal_system_init();
    printf("[BOOT] Stage 1 complete\n\n");

    /* ── STAGE 2: Task init ──────────────────────────── */
    printf("[BOOT] Stage 2: Initialising 8 tasks...\n");
    printf("[BOOT]   Watchdog     P8 100ms\n");
    printf("[BOOT]   ADCS         P7 100ms\n");
    printf("[BOOT]   FDIR Monitor P6 2000ms\n");
    printf("[BOOT]   Power Mgr    P6 1000ms\n");
    printf("[BOOT]   COMMS Mgr    P5 500ms\n");
    printf("[BOOT]   Payload Sched P5 1000ms\n");
    printf("[BOOT]   Telemetry    P4 10000ms\n");
    printf("[BOOT]   Flash Mgr    P3 5000ms\n");
    printf("[BOOT] Stage 2 complete\n\n");

    /* ── STAGE 3: Health check ───────────────────────── */
    printf("[BOOT] Stage 3: Subsystem health check...\n");
    printf("[BOOT]   EPS:  soc=%.0f%%  vbat=%.1fV\n",
           eps_read_soc()*100, eps_read_voltage());
    printf("[BOOT]   OBC:  temp=%.1fC\n", obc_read_temperature());
    printf("[BOOT]   ADCS: rate=%.1fdeg/s\n",
           adcs_read_angular_rate());
    printf("[BOOT] Stage 3 complete\n\n");

    /* ── STAGE 4: TLE load ───────────────────────────── */
    printf("[BOOT] Stage 4: TLE + IGRF loaded\n");
    printf("[BOOT]   TLE: simulation placeholder\n");
    printf("[BOOT]   IGRF-13: 500KB loaded\n");
    printf("[BOOT] Stage 4 complete\n\n");

    /* ── STAGE 5: Mode determination ─────────────────── */
    printf("[BOOT] Stage 5: Mode determination...\n");
    printf("[BOOT]   Battery SoC: %.0f%%\n", eps_read_soc()*100);
    printf("[BOOT]   Starting mode: %s\n",
           eps_read_soc() < 0.20f ? "SAFE" : "NOMINAL");
    printf("[BOOT] Stage 5 complete\n\n");

    /* ── STAGE 6: Nominal operation ──────────────────── */
    printf("[BOOT] Stage 6: SLARS-BCG-1 OPERATIONAL\n");
    printf("[BOOT] Satellite is now running.\n\n");
    printf("=======================================================\n\n");

    /* Print watchdog banner once */
    watchdog_task();

    /* ── MAIN SIMULATION LOOP ────────────────────────── */
    /*
     * Each iteration = 100ms of satellite time.
     * Tasks run at their proper periods:
     *   Cycle 1   = 100ms  (ADCS, WDT)
     *   Cycle 5   = 500ms  (COMMS)
     *   Cycle 10  = 1000ms (Power, Payload)
     *   Cycle 20  = 2000ms (FDIR)
     *   Cycle 50  = 5000ms (Flash)
     *   Cycle 100 = 10000ms (Telemetry)
     */
    int cycle = 0;

    for (;;) {
        cycle++;

        /* ADCS — every 100ms (highest priority after WDT) */
        adcs_controller_task();

        /* COMMS — every 500ms */
        if (cycle % 5 == 0)
            comms_manager_task();

        /* Power manager — every 1000ms */
        if (cycle % 10 == 0)
            power_manager_task();

        /* Payload scheduler — every 1000ms */
        if (cycle % 10 == 0)
            payload_scheduler_task();

        /* FDIR monitor — every 2000ms */
        if (cycle % 20 == 0)
            fdir_monitor_task();

        /* Flash manager — every 5000ms */
        if (cycle % 50 == 0)
            flash_manager_task();

        /* Telemetry — every 10000ms */
        if (cycle % 100 == 0)
            telemetry_task();

        /* Simulate battery drain and solar charging */
        if (cycle % 30 == 0)
            sim_drain_battery(0.001f);
        if (cycle % 20 == 0 && eps_read_soc() < 0.95f)
            sim_charge_battery(0.002f);

        /* 100ms per cycle */
        usleep(100000);
    }

    return 0;
}
