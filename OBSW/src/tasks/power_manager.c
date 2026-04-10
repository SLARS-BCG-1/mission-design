/*
 * SLARS-BCG-1 — Power Manager Task
 * Priority 6 | Period 1000ms | Stack 2KB
 * Guardian of the battery. All Step 2 power rules enforced here.
 */
#include "../hal/slars_types.h"

#define SOC_SAFE_ENTRY    0.20f
#define SOC_SAFE_EXIT     0.70f
#define SOC_SCIENCE_GATE  0.60f
#define VOLTAGE_CUTOFF    6.0f
#define PEAK_LOAD_W       5.8f
#define ORBITAL_PERIOD_S  5676.0f

static uint8_t  current_mode    = MODE_SAFE;
static uint8_t  uhf_pa_active   = 0;
static uint8_t  sband_tx_active = 0;
static uint8_t  passes_today    = 0;
static uint8_t  max_passes      = 3;
static float    safe_entry_time = 0.0f;
static float    sim_time        = 0.0f;

/* ── Public API ───────────────────────────────────── */
uint8_t power_get_mode(void)               { return current_mode;  }
uint8_t power_get_max_passes(void)         { return max_passes;    }
void    power_notify_uhf_pa(uint8_t a)    { uhf_pa_active = a;    }
void    power_notify_sband_tx(uint8_t a)  { sband_tx_active = a;  }
void    power_pass_complete(void)          { passes_today++;       }
void    power_reset_daily_passes(void)     { passes_today = 0;     }

/* ── Main task ────────────────────────────────────── */
void power_manager_task(void) {
    watchdog_task_kick(TASK_POWER_MANAGER);
    sim_time += 1.0f;

    float soc   = eps_read_soc();
    float vbat  = eps_read_voltage();
    float watts = eps_read_total_watts();

    /* Safe mode entry */
    if (soc < SOC_SAFE_ENTRY || vbat < VOLTAGE_CUTOFF) {
        if (current_mode != MODE_SAFE) {
            current_mode    = MODE_SAFE;
            safe_entry_time = sim_time;
            max_passes      = 0;
            payload_disable_all();
            adcs_set_minimum_power();
            fdir_log("SAFE_ENTRY: soc=%.2f vbat=%.2f", soc, vbat);
            printf("[PWR]  SAFE MODE ENTERED: soc=%.0f%% vbat=%.1fV\n",
                   soc*100, vbat);
        }

    /* Safe mode exit — must wait 2 orbits */
    } else if (current_mode == MODE_SAFE && soc > SOC_SAFE_EXIT) {
        float orbits = (sim_time - safe_entry_time) / ORBITAL_PERIOD_S;
        if (orbits >= 2.0f) {
            current_mode = MODE_NOMINAL;
            max_passes   = 3;
            printf("[PWR]  SAFE MODE EXIT: soc=%.0f%%\n", soc*100);
        }

    /* Nominal -> Science */
    } else if (current_mode == MODE_NOMINAL && soc > SOC_SCIENCE_GATE) {
        current_mode = MODE_SCIENCE;
        printf("[PWR]  SCIENCE MODE: soc=%.0f%%\n", soc*100);

    /* Science -> Nominal */
    } else if (current_mode == MODE_SCIENCE && soc < SOC_SCIENCE_GATE) {
        current_mode = MODE_NOMINAL;
        printf("[PWR]  NOMINAL MODE: soc dropped below 60%%\n");
    }

    /* EOL reduction — Step 2 Section 6 */
    max_passes = (soc < SOC_SCIENCE_GATE) ? 2 : 3;
    payload_scheduler_set_max_passes(max_passes);

    /* HARD RULE: UHF PA and S-band TX never simultaneous — Step 4 */
    if (uhf_pa_active && sband_tx_active) {
        sband_tx_disable();
        fdir_log("POWER_VIOLATION: UHF+SBAND simultaneous blocked");
        printf("[PWR]  POWER VIOLATION BLOCKED!\n");
    }

    /* Peak load guard — Step 2: 6.0W EPS cap */
    if (watts > PEAK_LOAD_W) {
        payload_disable_lowest_priority();
        printf("[PWR]  PEAK LOAD: %.1fW — shedding payload\n", watts);
    }

    /* Emergency */
    if (soc < 0.10f) {
        payload_disable_all();
        fdir_log("EMERGENCY_POWER: soc=%.2f", soc);
        printf("[PWR]  EMERGENCY: soc=%.0f%% CRITICAL!\n", soc*100);
    }

    system_set_mode(current_mode);

    printf("[PWR]  soc=%.0f%% vbat=%.1fV mode=%s watts=%.1fW\n",
           soc*100, vbat,
           current_mode==0?"SAFE":current_mode==1?"NOMINAL":"SCIENCE",
           watts);
}
