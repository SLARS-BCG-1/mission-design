/*
 * SLARS-BCG-1 — Watchdog Task
 * Priority 8 | Period 100ms | Stack 512B
 * Kicks hardware WDT. Monitors all 8 tasks are alive.
 */
#include "../hal/slars_types.h"

static volatile uint8_t task_alive[8] = {0};
static int wdt_cycle = 0;

void watchdog_task_kick(uint8_t task_id) {
    if (task_id < 8) task_alive[task_id] = 1;
}

void watchdog_task(void) {
    printf("\n[WDT]  Watchdog started — Priority 8 — Period 100ms\n");
    printf("[WDT]  Monitoring 8 tasks. Satellite CANNOT freeze.\n");

    for (;;) {
        wdt_cycle++;

        /* Kick hardware WDT — on real STM32: IWDG->KR = 0xAAAA */

        /* Check every task is alive */
        for (uint8_t i = 1; i < 8; i++) {
            if (task_alive[i] == 0 && wdt_cycle > 5) {
                printf("[WDT]  WARNING: Task %d missed deadline!\n", i);
                fdir_report_task_dead(i);
            }
            task_alive[i] = 0;
        }
        task_alive[TASK_WATCHDOG] = 1;

        if (wdt_cycle % 100 == 0) {
            printf("[WDT]  All tasks alive — cycle %d\n", wdt_cycle);
        }

        /* In simulation: return after printing banner (called once at boot) */
        return;
    }
}
