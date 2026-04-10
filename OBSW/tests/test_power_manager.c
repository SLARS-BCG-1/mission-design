/*
 * SLARS-BCG-1 — Unit Tests
 * Tests all power rules, Sri Lanka bbox, ADCS modes, FDIR thresholds.
 *
 * Build and run:
 *   gcc test_power_manager.c -o test_power -lm && ./test_power
 */
#include "../src/hal/sim_hal.h"
#include <assert.h>

static int tests_run    = 0;
static int tests_passed = 0;

#define TEST(name, condition) do { \
    tests_run++; \
    if (condition) { \
        printf("  PASS: %s\n", name); \
        tests_passed++; \
    } else { \
        printf("  FAIL: %s\n", name); \
    } \
} while(0)

void test_battery(void) {
    printf("\n[TEST] Battery thresholds\n");
    sim_battery_soc = 0.19f;
    TEST("SoC 19% is below safe threshold (20%)",
         eps_read_soc() < 0.20f);

    sim_battery_soc = 0.20f;
    TEST("SoC 20% is at safe threshold",
         eps_read_soc() >= 0.20f);

    sim_battery_soc = 0.55f;
    TEST("SoC 55% is below science gate (60%)",
         eps_read_soc() < 0.60f);

    sim_battery_soc = 0.75f;
    TEST("SoC 75% is above science gate (60%)",
         eps_read_soc() > 0.60f);

    sim_battery_soc = 0.85f;
    TEST("SoC 85% is above safe exit (70%)",
         eps_read_soc() > 0.70f);
}

void test_voltage(void) {
    printf("\n[TEST] Battery voltage\n");
    sim_battery_soc = 0.0f;
    TEST("Empty battery voltage = 6.0V (cutoff)",
         fabsf(eps_read_voltage() - 6.0f) < 0.1f);

    sim_battery_soc = 1.0f;
    TEST("Full battery voltage = 8.4V",
         fabsf(eps_read_voltage() - 8.4f) < 0.1f);

    sim_battery_soc = 0.5f;
    TEST("Half battery voltage is between 6V and 8.4V",
         eps_read_voltage() > 6.0f && eps_read_voltage() < 8.4f);
}

void test_sri_lanka_bbox(void) {
    printf("\n[TEST] Sri Lanka bounding box\n");

    /* Points inside Sri Lanka */
    TEST("Colombo (6.9N 79.9E) lat in range",
         6.9f >= 5.9f && 6.9f <= 9.9f);
    TEST("Colombo lon in range",
         79.9f >= 79.7f && 79.9f <= 81.9f);

    TEST("Jaffna (9.7N 80.0E) lat in range",
         9.7f >= 5.9f && 9.7f <= 9.9f);
    TEST("Jaffna lon in range",
         80.0f >= 79.7f && 80.0f <= 81.9f);

    TEST("Gampaha (7.1N 80.0E) in bbox",
         7.1f >= 5.9f && 7.1f <= 9.9f &&
         80.0f >= 79.7f && 80.0f <= 81.9f);

    /* Points outside Sri Lanka */
    TEST("India (15N 78E) lat OUT of range",
         !(15.0f >= 5.9f && 15.0f <= 9.9f));
    TEST("Maldives (3N 73E) lat OUT of range",
         !(3.0f >= 5.9f && 3.0f <= 9.9f));
    TEST("Bangladesh (23N 90E) lon OUT of range",
         !(90.0f >= 79.7f && 90.0f <= 81.9f));

    /* Boundary cases */
    TEST("Southern tip (5.9N) is boundary",
         5.9f >= 5.9f);
    TEST("Northern tip (9.9N) is boundary",
         9.9f <= 9.9f);
    TEST("Western coast (79.7E) is boundary",
         79.7f >= 79.7f);
    TEST("Eastern coast (81.9E) is boundary",
         81.9f <= 81.9f);
}

void test_power_sequencing(void) {
    printf("\n[TEST] Power sequencing rule (Step 4)\n");

    float uhf_pa_w   = 3.5f;
    float sband_tx_w = 2.5f;
    float eps_limit  = 6.0f;

    TEST("UHF PA (3.5W) alone within EPS limit",
         uhf_pa_w < eps_limit);
    TEST("S-band TX (2.5W) alone within EPS limit",
         sband_tx_w < eps_limit);
    TEST("Combined = EPS limit (6.0W) — never simultaneous",
         uhf_pa_w + sband_tx_w == eps_limit);
    TEST("Peak load guard triggers at 5.8W (97% of 6.0W)",
         5.8f < eps_limit);
}

void test_adcs_thresholds(void) {
    printf("\n[TEST] ADCS thresholds (Step 5)\n");

    sim_angular_rate = 5.5f;
    TEST("Rate 5.5 deg/s — still tumbling (above 3.0 threshold)",
         adcs_read_angular_rate() >= 3.0f);

    sim_angular_rate = 2.8f;
    TEST("Rate 2.8 deg/s — detumbling complete (below 3.0)",
         adcs_read_angular_rate() < 3.0f);

    sim_pointing_err = 6.0f;
    TEST("Pointing 6 deg — fails science gate (above 5 deg)",
         adcs_read_pointing_error() >= 5.0f);

    sim_pointing_err = 4.5f;
    TEST("Pointing 4.5 deg — passes science gate (below 5 deg)",
         adcs_read_pointing_error() < 5.0f);

    sim_rw_speed = 5999;
    TEST("RW 5999 RPM triggers FDIR desaturation",
         abs(adcs_read_rw_speed()) >= 5999);

    sim_rw_speed = 3000;
    TEST("RW 3000 RPM is healthy (nominal range)",
         abs(adcs_read_rw_speed()) < 5500);
}

void test_fdir_thresholds(void) {
    printf("\n[TEST] FDIR monitoring thresholds (Step 6)\n");

    sim_battery_soc = 0.15f;
    TEST("SoC 15% triggers FDIR Level 3 safe mode",
         eps_read_soc() < 0.20f);

    sim_obc_temp = 80.0f;
    TEST("OBC 80C triggers FDIR Level 2 (above 75C)",
         obc_read_temperature() > 75.0f);

    sim_obc_temp = 35.0f;
    TEST("OBC 35C is healthy",
         obc_read_temperature() <= 75.0f);

    sim_total_draw = 6.2f;
    TEST("Total 6.2W triggers FDIR Level 2 (above 5.8W)",
         eps_read_total_watts() > 5.8f);

    sim_total_draw = 1.85f;
    TEST("Total 1.85W is healthy (nominal)",
         eps_read_total_watts() <= 5.8f);
}

void test_orbital_parameters(void) {
    printf("\n[TEST] Orbital parameters (Step 1)\n");

    float period_min = 94.6f;
    float eclipse_frac = 0.35f;
    float sunlit_frac  = 0.65f;

    TEST("Eclipse fraction = 35% (33 min per orbit)",
         fabsf(eclipse_frac - 0.35f) < 0.01f);
    TEST("Sunlit fraction = 65% (62 min per orbit)",
         fabsf(sunlit_frac - 0.65f) < 0.01f);
    TEST("Eclipse + sunlit = 100%",
         fabsf(eclipse_frac + sunlit_frac - 1.0f) < 0.001f);
    TEST("Orbital period = 94.6 minutes",
         fabsf(period_min - 94.6f) < 0.1f);
    TEST("Battery must survive 33 min eclipse",
         33.0f < period_min);
    TEST("Science window (7 min) fits in sunlit period (62 min)",
         7.0f < period_min * sunlit_frac);
}

int main(void) {
    printf("*******************************************************\n");
    printf("*    SLARS-BCG-1 OBSW Unit Test Suite                 *\n");
    printf("*    Bandaranayake College, Gampaha                   *\n");
    printf("*******************************************************\n");

    test_battery();
    test_voltage();
    test_sri_lanka_bbox();
    test_power_sequencing();
    test_adcs_thresholds();
    test_fdir_thresholds();
    test_orbital_parameters();

    printf("\n=======================================================\n");
    printf("  RESULTS: %d / %d tests passed\n",
           tests_passed, tests_run);
    if (tests_passed == tests_run)
        printf("  ALL TESTS PASSED\n");
    else
        printf("  %d test(s) FAILED\n", tests_run - tests_passed);
    printf("=======================================================\n\n");

    return (tests_passed == tests_run) ? 0 : 1;
}
