/*
 * SLARS-BCG-1 — Simulation Hardware Abstraction Layer
 * sim_hal.h
 *
 * State variables are extern — they live in sim_state.c.
 * All .c files and test files share the same simulation state.
 *
 * SLARS Team — Bandaranayake College, Gampaha
 */

#ifndef SIM_HAL_H
#define SIM_HAL_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <stdarg.h>

/* ══════════════════════════════════════════════════════
 * SHARED SIMULATION STATE — defined in sim_state.c
 * extern means: the variable exists, but it is defined
 * in sim_state.c. All files share the same one copy.
 * ══════════════════════════════════════════════════════ */

extern float    sim_battery_soc;
extern float    sim_battery_temp;
extern float    sim_solar_power;
extern float    sim_total_draw;
extern float    sim_obc_temp;
extern uint32_t sim_orbit_count;
extern uint8_t  sim_adcs_mode;
extern float    sim_angular_rate;
extern float    sim_pointing_err;
extern int      sim_rw_speed;
extern float    sim_magnetic_B[3];
extern uint8_t  sim_uhf_ok;
extern uint32_t sim_beacon_count;
extern uint8_t  sim_emmc_healthy;
extern uint8_t  sim_aprs_queue;
extern uint8_t  sim_system_mode;

/* ══════════════════════════════════════════════════════
 * EPS FUNCTIONS
 * ══════════════════════════════════════════════════════ */

static inline float eps_read_soc(void) {
    return sim_battery_soc;
}
static inline float eps_read_voltage(void) {
    return 6.0f + (sim_battery_soc * 2.4f);
}
static inline float eps_read_total_watts(void) {
    return sim_total_draw;
}
static inline float eps_read_solar_power(void) {
    return sim_solar_power;
}
static inline float eps_read_battery_temp(void) {
    return sim_battery_temp;
}

/* ══════════════════════════════════════════════════════
 * OBC FUNCTIONS
 * ══════════════════════════════════════════════════════ */

static inline float obc_read_temperature(void) {
    return sim_obc_temp;
}
static inline uint32_t orbit_get_count(void) {
    return sim_orbit_count;
}
static inline uint32_t rtc_get_unix_time(void) {
    return (uint32_t)time(NULL);
}

/* ══════════════════════════════════════════════════════
 * ADCS SENSOR FUNCTIONS
 * ══════════════════════════════════════════════════════ */

static inline void magnetometer_read(float B[3]) {
    B[0] = sim_magnetic_B[0] + ((float)(rand()%100-50)) * 1e-9f;
    B[1] = sim_magnetic_B[1] + ((float)(rand()%100-50)) * 1e-9f;
    B[2] = sim_magnetic_B[2] + ((float)(rand()%100-50)) * 1e-9f;
}
static inline void gyroscope_read(float rate[3]) {
    rate[0] = sim_angular_rate * 0.6f;
    rate[1] = sim_angular_rate * 0.5f;
    rate[2] = sim_angular_rate * 0.4f;
}
static inline float adcs_read_angular_rate(void)   { return sim_angular_rate; }
static inline float adcs_read_pointing_error(void) { return sim_pointing_err; }
static inline int   adcs_read_rw_speed(void)       { return sim_rw_speed; }
static inline float adcs_get_radial_velocity_ms(void) { return 3250.0f; }

/* ══════════════════════════════════════════════════════
 * ADCS ACTUATOR FUNCTIONS
 * ══════════════════════════════════════════════════════ */

static inline void magnetorquer_set_dipole(float mx, float my, float mz) {
    (void)mx; (void)my; (void)mz;
    sim_angular_rate *= 0.998f;
    if (sim_angular_rate < 0.05f) sim_angular_rate = 0.0f;
}
static inline void magnetorquer_set_torque(float tx, float ty) {
    (void)tx; (void)ty;
    if (sim_pointing_err > 1.0f) sim_pointing_err *= 0.99f;
}
static inline void reaction_wheel_set_torque(float torque) {
    sim_rw_speed += (int)(torque * 1000.0f);
    if (sim_rw_speed >  6000) sim_rw_speed =  6000;
    if (sim_rw_speed < -6000) sim_rw_speed = -6000;
    if (sim_pointing_err > 0.5f) sim_pointing_err *= 0.995f;
}
static inline int reaction_wheel_get_speed(void) { return sim_rw_speed; }
static inline void adcs_begin_desaturation(void) {
    printf("[HAL]  RW desaturation: %d RPM -> reducing\n", sim_rw_speed);
    sim_rw_speed = (int)(sim_rw_speed * 0.2f);
}

/* ══════════════════════════════════════════════════════
 * EKF STUBS
 * ══════════════════════════════════════════════════════ */

static inline void ekf_update(void) {
    if (sim_angular_rate < 3.0f && sim_adcs_mode >= 2)
        if (sim_pointing_err > 2.0f) sim_pointing_err -= 0.05f;
}
static inline void ekf_get_attitude_error(float e[3]) {
    e[0] = sim_pointing_err * 0.6f;
    e[1] = sim_pointing_err * 0.5f;
    e[2] = sim_pointing_err * 0.4f;
}
static inline void ekf_get_nadir_error(float e[3]) {
    e[0] = sim_pointing_err * 0.5f;
    e[1] = sim_pointing_err * 0.4f;
    e[2] = sim_pointing_err * 0.3f;
}
static inline float ekf_get_pointing_error(void)        { return sim_pointing_err; }
static inline float ekf_get_angular_rate_magnitude(void){ return sim_angular_rate; }
static inline void  ekf_get_quaternion(float q[4]) {
    q[0]=1.0f; q[1]=0.0f; q[2]=0.0f; q[3]=0.0f;
}

/* ══════════════════════════════════════════════════════
 * COMMS HARDWARE FUNCTIONS
 * ══════════════════════════════════════════════════════ */

static inline void uhf_transmit(const uint8_t *data, size_t len) {
    (void)data; (void)len;
    sim_beacon_count++;
}
static inline uint8_t uhf_receive(uint8_t *buf, size_t maxlen) {
    (void)buf; (void)maxlen;
    return 0;
}
static inline uint8_t uhf_pa_is_active(void) { return 0; }
static inline void sband_set_frequency_offset(int32_t hz) {
    printf("[HAL]  S-band Doppler offset: %d Hz\n", hz);
}
static inline void cfdp_send_pending_files(void) {
    printf("[HAL]  CFDP: transmitting science files at 128 kbps\n");
}
static inline uint8_t cfdp_is_delivered(const char *f) { (void)f; return 0; }

/* ══════════════════════════════════════════════════════
 * PAYLOAD HARDWARE FUNCTIONS
 * ══════════════════════════════════════════════════════ */

static inline void imager_enable(void) {
    printf("[HAL]  IMAGER: ON\n");
    sim_total_draw += 1.2f;
}
static inline void imager_disable(void) {
    printf("[HAL]  IMAGER: OFF\n");
    if (sim_total_draw > 1.2f) sim_total_draw -= 1.2f;
}
static inline void sdr_enable_tx(void) {
    printf("[HAL]  SDR RELAY: TX ON\n");
    sim_total_draw += 0.8f;
}
static inline void sdr_disable_tx(void) {
    printf("[HAL]  SDR RELAY: TX OFF\n");
    if (sim_total_draw > 0.8f) sim_total_draw -= 0.8f;
}
static inline void    sband_tx_disable(void)      { printf("[HAL]  S-BAND TX: OFF\n"); }
static inline void    ais_log_update(void)         { }
static inline uint8_t aprs_get_queue_length(void)  { return sim_aprs_queue; }

/* ══════════════════════════════════════════════════════
 * FLASH HARDWARE STUBS
 * ══════════════════════════════════════════════════════ */

static inline void flash_write_tm_frame(const void *d, size_t l) {
    (void)d; (void)l;
}
static inline void flash_write_science_frame(const uint8_t *d, size_t l) {
    (void)d;
    printf("[HAL]  FLASH: Wrote %zu bytes\n", l);
}
static inline uint8_t flash_health_check(void) { return sim_emmc_healthy; }

/* ══════════════════════════════════════════════════════
 * SYSTEM FUNCTIONS
 * ══════════════════════════════════════════════════════ */

static inline void payload_disable_all(void) {
    printf("[HAL]  PAYLOAD: All OFF\n");
    sim_total_draw = 1.25f;
}
static inline void payload_disable_lowest_priority(void) {
    printf("[HAL]  PAYLOAD: Lowest priority OFF\n");
}
static inline void adcs_set_minimum_power(void) {
    printf("[HAL]  ADCS: Minimum power\n");
}
static inline void system_set_mode(uint8_t mode) {
    if (mode != sim_system_mode) {
        const char *n[] = {"SAFE","NOMINAL","SCIENCE"};
        printf("[HAL]  MODE: %s -> %s\n",
               n[sim_system_mode<3?sim_system_mode:0],
               n[mode<3?mode:0]);
        sim_system_mode = mode;
    }
}
static inline void system_enter_safe_mode(void) { system_set_mode(0); }
static inline void hal_system_init(void) {
    srand((unsigned)time(NULL));
    printf("[HAL]  Simulation HAL initialized\n");
}
static inline void purge_oldest_files(void) { }

/* ══════════════════════════════════════════════════════
 * SGP4 ORBIT PROPAGATOR STUBS
 * ══════════════════════════════════════════════════════ */

static inline void sgp4_propagate(const void *l1, const void *l2,
                                   double t, double *lat,
                                   double *lon, double *alt) {
    (void)l1; (void)l2;
    double tm = fmod(t, 94.6*60.0);
    *lat = 10.0 * sin(2.0*3.14159*tm/(94.6*60.0));
    *lon = fmod(80.3 + (tm/60.0)*4.0, 360.0) - 180.0;
    *alt = 500.0;
}
static inline double sgp4_time_to_footprint(
    const void *l1, const void *l2,
    float lat_min, float lat_max,
    float lon_min, float lon_max,
    double t_now) {
    (void)l1;(void)l2;(void)lat_min;(void)lat_max;(void)lon_min;(void)lon_max;
    double p = 94.6*60.0;
    double r = p - fmod(t_now, p);
    return r > p/2 ? r - p/2 : r;
}

/* ══════════════════════════════════════════════════════
 * CRC16
 * ══════════════════════════════════════════════════════ */

static inline uint16_t crc16_compute(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc<<1)^0x1021 : crc<<1;
    }
    return crc;
}

/* ══════════════════════════════════════════════════════
 * SIMULATION SCENARIO CONTROLS
 * ══════════════════════════════════════════════════════ */

static inline void sim_drain_battery(float d) {
    sim_battery_soc -= d;
    if (sim_battery_soc < 0.0f) sim_battery_soc = 0.0f;
}
static inline void sim_charge_battery(float d) {
    sim_battery_soc += d;
    if (sim_battery_soc > 1.0f) sim_battery_soc = 1.0f;
}
static inline void sim_set_tumbling(float rate) {
    sim_angular_rate = rate;
    sim_pointing_err = 90.0f;
    sim_adcs_mode    = 1;
}
static inline void sim_set_low_battery(void) {
    sim_battery_soc = 0.15f;
    sim_solar_power = 0.0f;
}

#endif /* SIM_HAL_H */
