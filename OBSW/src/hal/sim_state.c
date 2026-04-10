/*
 * SLARS-BCG-1 — Simulation State Variables
 * sim_state.c
 *
 * All simulation state lives here as non-static globals.
 * Every .c file that includes sim_hal.h shares these values.
 * Test files write directly to these to inject scenarios.
 *
 * SLARS Team — Bandaranayake College, Gampaha
 */

#include <stdint.h>

float    sim_battery_soc     = 0.85f;
float    sim_battery_temp    = 22.0f;
float    sim_solar_power     = 2.0f;
float    sim_total_draw      = 1.85f;
float    sim_obc_temp        = 35.0f;
uint32_t sim_orbit_count     = 0;
uint8_t  sim_adcs_mode       = 1;
float    sim_angular_rate    = 5.2f;
float    sim_pointing_err    = 45.0f;
int      sim_rw_speed        = 0;
float    sim_magnetic_B[3]   = {25e-6f, 5e-6f, -40e-6f};
uint8_t  sim_uhf_ok          = 1;
uint32_t sim_beacon_count    = 0;
uint8_t  sim_emmc_healthy    = 1;
uint8_t  sim_aprs_queue      = 0;
uint8_t  sim_system_mode     = 0;
