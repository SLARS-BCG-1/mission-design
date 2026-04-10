/*
 * SLARS-BCG-1 — EKF header
 * ekf.h
 *
 * SLARS Team — Bandaranayake College, Gampaha
 */

#ifndef EKF_H
#define EKF_H

#include "quat.h"

/*
 * EKF state vector: [qw, qx, qy, qz, bx, by, bz]
 *   q = attitude quaternion (4 components)
 *   b = gyroscope bias in rad/s (3 components)
 *
 * The gyroscope measures angular rate + bias + noise.
 * The EKF estimates both the attitude and the bias simultaneously.
 * Correcting for bias makes the attitude estimate much more accurate.
 */
typedef struct {
    Quat  q;            /* Current attitude estimate */
    float bias[3];      /* Gyroscope bias estimate (rad/s) */
    float P[7][7];      /* Error covariance matrix (7x7) */
    float Q_att;        /* Process noise — attitude */
    float Q_bias;       /* Process noise — gyro bias */
    float R_mag;        /* Measurement noise — magnetometer */
    float dt;           /* Time step in seconds (0.1 for 100ms) */
    int   initialised;  /* 1 after first magnetometer init */
} EKF_State;

/* Initialise the EKF */
void ekf_init(EKF_State *ekf, float dt_seconds);

/*
 * Propagate: integrate gyroscope rates forward by dt
 * Call this every 100ms with fresh gyro readings.
 * gyro_rads = angular rate in rad/s [x, y, z]
 */
void ekf_propagate(EKF_State *ekf, const float gyro_rads[3]);

/*
 * Update: correct the attitude estimate using magnetometer
 * Call this after propagate when magnetometer data is available.
 * mag_body  = measured B vector in body frame (Tesla)
 * mag_ref   = reference B vector in inertial frame (Tesla) from IGRF
 */
void ekf_update_magnetometer(EKF_State *ekf,
                               const float mag_body[3],
                               const float mag_ref[3]);

/* Get current attitude estimate */
void  ekf_get_attitude(const EKF_State *ekf, Quat *q_out);

/* Get pointing error vs nadir in degrees */
float ekf_get_pointing_error_deg(const EKF_State *ekf,
                                   float pos_x,
                                   float pos_y,
                                   float pos_z);

/* Get angular rate magnitude in deg/s (bias-corrected) */
float ekf_get_rate_degs(const EKF_State *ekf,
                          const float gyro_rads[3]);

/* Get Euler angles for telemetry (degrees) */
void  ekf_get_euler_deg(const EKF_State *ekf,
                          float *roll, float *pitch, float *yaw);

/* Reset EKF to identity attitude */
void  ekf_reset(EKF_State *ekf);

#endif /* EKF_H */
