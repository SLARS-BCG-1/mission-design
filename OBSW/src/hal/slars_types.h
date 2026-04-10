/*
 * SLARS-BCG-1 — Common types and shared declarations
 * slars_types.h
 */

#ifndef SLARS_TYPES_H
#define SLARS_TYPES_H

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "sim_hal.h"

/* ── System modes ──────────────────────────────────── */
#define MODE_SAFE     0
#define MODE_NOMINAL  1
#define MODE_SCIENCE  2

/* ── ADCS modes ────────────────────────────────────── */
#define ADCS_MODE_1_BDOT    1
#define ADCS_MODE_2_COARSE  2
#define ADCS_MODE_3_NADIR   3

/* ── Task IDs for watchdog ─────────────────────────── */
#define TASK_WATCHDOG       0
#define TASK_ADCS           1
#define TASK_FDIR           2
#define TASK_POWER_MANAGER  3
#define TASK_COMMS_MANAGER  4
#define TASK_PAYLOAD_SCHED  5
#define TASK_TELEMETRY      6
#define TASK_FLASH_MANAGER  7

/* ══════════════════════════════════════════════════════
 * FUNCTION DECLARATIONS
 * Each task declares only what OTHER tasks need to call.
 * ══════════════════════════════════════════════════════ */

/* Watchdog */
void    watchdog_task_kick(uint8_t task_id);

/* FDIR */
void    fdir_log(const char *fmt, ...);
void    fdir_report_task_dead(uint8_t task_id);
void    fdir_report_power_violation(void);
void    fdir_report_flash_failure(void);
void    fdir_report_emergency(void);
void    fdir_restart_task(uint8_t task_id);
void    fdir_power_cycle_subsystem(uint8_t id);
void    fdir_transmit_anomaly_report(void);

/* Power manager */
uint8_t power_get_mode(void);
uint8_t power_get_max_passes(void);
void    power_notify_uhf_pa(uint8_t active);
void    power_notify_sband_tx(uint8_t active);
void    power_pass_complete(void);
void    power_reset_daily_passes(void);

/* ADCS controller */
uint8_t adcs_get_mode(void);
void    adcs_set_mode(uint8_t mode);

/* COMMS manager */
void    comms_trigger_sband_downlink(void);
void    comms_update_doppler_quaternion(const float q[4]);
uint8_t comms_uhf_beacon_confirmed(void);
void     tc_dispatch(const uint8_t *buf, uint8_t len);
uint32_t tc_get_total_count(void);
uint32_t tc_get_invalid_count(void);
uint32_t tc_get_last_id(void);

/* Payload scheduler */
void    payload_scheduler_set_max_passes(uint8_t n);
void    payload_scheduler_reset_daily(void);
void    payload_scheduler_update_tle(const char *l1, const char *l2);

/* Telemetry */
void    telemetry_task(void);

/* Flash manager */
void    flash_manager_task(void);

/* Task entry points */
void    watchdog_task(void);
void    adcs_controller_task(void);
void    fdir_monitor_task(void);
void    power_manager_task(void);
void    comms_manager_task(void);
void    payload_scheduler_task(void);

#endif /* SLARS_TYPES_H */
/* ── TM decoded struct — used by decoder and ground station ─────────────── */
typedef struct {
    uint8_t  frame_type;       /* 0x01 = housekeeping                        */
    uint8_t  seq_counter;      /* 0-255 wrapping sequence number             */
    uint16_t met_seconds;      /* Mission elapsed time in seconds            */
    uint8_t  system_mode;      /* 0=SAFE 1=NOMINAL 2=SCIENCE                 */
    float    bat_soc_pct;      /* Battery SoC in percent (0.00-100.00)       */
    float    bat_voltage_v;    /* Battery voltage in Volts                   */
    int8_t   bat_temp_c;       /* Battery temperature in degrees C           */
    float    solar_w;          /* Solar panel power in Watts                 */
    float    draw_w;           /* Total satellite draw in Watts              */
    int8_t   obc_temp_c;       /* OBC board temperature in degrees C         */
    uint8_t  adcs_mode;        /* 1=B-dot 2=Coarse 3=Nadir                  */
    float    pointing_deg;     /* Pointing error in degrees                  */
    float    rate_degs;        /* Angular rate in deg/s                      */
    int16_t  rw_rpm;           /* Reaction wheel speed in RPM               */
    uint32_t orbit_count;      /* Total orbits since launch                  */
    uint8_t  passes_today;     /* Science passes completed today             */
    uint8_t  aprs_queue;       /* APRS messages waiting to relay             */
    uint32_t unix_time;        /* UTC Unix timestamp                         */
    uint8_t  fdir_level;       /* Current FDIR escalation level              */
    uint8_t  crc_valid;        /* 1 = CRC passed, 0 = frame corrupt          */
} tm_decoded_t;

/* TM encoder function declarations */
uint8_t  tm_encode_housekeeping(uint8_t *buf);
uint8_t  tm_decode_housekeeping(const uint8_t *buf, tm_decoded_t *out);
uint32_t tm_get_frames_encoded(void);
uint8_t  tm_get_seq_counter(void);
