/*
 * SLARS-BCG-1 Flight Software
 * File: quat.c
 *
 * Quaternion mathematics for attitude representation.
 * A quaternion q = [w, x, y, z] where w is the scalar part.
 * Unit quaternion represents a pure rotation — no scaling.
 *
 * Why quaternions instead of Euler angles?
 *   - No gimbal lock (singularity when pitch = +/-90 deg)
 *   - Smooth interpolation between orientations
 *   - Efficient multiplication (4 multiplies vs 9 for rotation matrix)
 *   - Used by every professional satellite ADCS
 *
 * SLARS Team — Bandaranayake College, Gampaha
 */

#include <math.h>
#include <string.h>
#include "quat.h"

/* ── Initialise identity quaternion (no rotation) ────────────────────────── */
void quat_identity(Quat *q) {
    q->w = 1.0f;
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
}

/* ── Copy ────────────────────────────────────────────────────────────────── */
void quat_copy(Quat *dst, const Quat *src) {
    dst->w = src->w;
    dst->x = src->x;
    dst->y = src->y;
    dst->z = src->z;
}

/* ── Normalise — make the quaternion unit length ─────────────────────────── */
/*
 * A quaternion must stay unit length (norm = 1.0) to represent
 * a pure rotation. Floating point errors accumulate over time
 * and slowly stretch the quaternion away from unit length.
 * Normalise after every integration step to correct this.
 */
void quat_normalise(Quat *q) {
    float norm = sqrtf(q->w*q->w + q->x*q->x +
                       q->y*q->y + q->z*q->z);
    if (norm < 1e-10f) {
        /* Degenerate quaternion — reset to identity */
        quat_identity(q);
        return;
    }
    float inv = 1.0f / norm;
    q->w *= inv;
    q->x *= inv;
    q->y *= inv;
    q->z *= inv;
}

/* ── Norm (magnitude) ────────────────────────────────────────────────────── */
float quat_norm(const Quat *q) {
    return sqrtf(q->w*q->w + q->x*q->x +
                 q->y*q->y + q->z*q->z);
}

/* ── Conjugate (inverse for unit quaternion) ─────────────────────────────── */
/*
 * The conjugate of q = [w, x, y, z] is q* = [w, -x, -y, -z].
 * For a unit quaternion, q* = q^(-1) — it undoes the rotation.
 */
void quat_conjugate(Quat *out, const Quat *q) {
    out->w =  q->w;
    out->x = -q->x;
    out->y = -q->y;
    out->z = -q->z;
}

/* ── Multiply two quaternions ────────────────────────────────────────────── */
/*
 * q_out = q1 * q2
 * This composes two rotations — first apply q2, then q1.
 * Note: quaternion multiplication is NOT commutative (q1*q2 != q2*q1).
 */
void quat_multiply(Quat *out, const Quat *q1, const Quat *q2) {
    /* Hamilton product */
    out->w = q1->w*q2->w - q1->x*q2->x - q1->y*q2->y - q1->z*q2->z;
    out->x = q1->w*q2->x + q1->x*q2->w + q1->y*q2->z - q1->z*q2->y;
    out->y = q1->w*q2->y - q1->x*q2->z + q1->y*q2->w + q1->z*q2->x;
    out->z = q1->w*q2->z + q1->x*q2->y - q1->y*q2->x + q1->z*q2->w;
}

/* ── Rotate a 3D vector by a quaternion ──────────────────────────────────── */
/*
 * v_out = q * v * q*
 * Rotates vector v by the rotation represented by quaternion q.
 * This is the core operation for rotating sensor readings
 * from body frame to inertial frame and back.
 */
void quat_rotate_vector(float out[3], const Quat *q, const float v[3]) {
    /* Convert v to pure quaternion: p = [0, vx, vy, vz] */
    Quat p, q_conj, tmp;
    p.w = 0.0f;
    p.x = v[0];
    p.y = v[1];
    p.z = v[2];

    /* tmp = q * p */
    quat_multiply(&tmp, q, &p);

    /* out = tmp * q* */
    quat_conjugate(&q_conj, q);
    Quat result;
    quat_multiply(&result, &tmp, &q_conj);

    out[0] = result.x;
    out[1] = result.y;
    out[2] = result.z;
}

/* ── Convert quaternion to Euler angles (roll, pitch, yaw) ──────────────── */
/*
 * Returns angles in degrees.
 * Convention: ZYX (yaw-pitch-roll) — standard aerospace convention.
 * Singularity occurs at pitch = +/-90 deg (gimbal lock).
 * Use only for display/telemetry — never for control calculations.
 */
void quat_to_euler_deg(const Quat *q,
                        float *roll_deg,
                        float *pitch_deg,
                        float *yaw_deg) {
    /* Roll (x-axis rotation) */
    float sinr_cosp = 2.0f*(q->w*q->x + q->y*q->z);
    float cosr_cosp = 1.0f - 2.0f*(q->x*q->x + q->y*q->y);
    *roll_deg = atan2f(sinr_cosp, cosr_cosp) * (180.0f / 3.14159265f);

    /* Pitch (y-axis rotation) */
    float sinp = 2.0f*(q->w*q->y - q->z*q->x);
    if (fabsf(sinp) >= 1.0f)
        *pitch_deg = copysignf(90.0f, sinp);
    else
        *pitch_deg = asinf(sinp) * (180.0f / 3.14159265f);

    /* Yaw (z-axis rotation) */
    float siny_cosp = 2.0f*(q->w*q->z + q->x*q->y);
    float cosy_cosp = 1.0f - 2.0f*(q->y*q->y + q->z*q->z);
    *yaw_deg = atan2f(siny_cosp, cosy_cosp) * (180.0f / 3.14159265f);
}

/* ── Error quaternion between two attitudes ──────────────────────────────── */
/*
 * Computes q_err = q_desired* * q_current
 * The result represents the rotation needed to go from
 * current attitude to desired attitude.
 * The vector part [x,y,z] of q_err is proportional to the
 * pointing error — this is what the PD/PID controller uses.
 */
void quat_error(Quat *q_err, const Quat *q_desired, const Quat *q_current) {
    Quat q_des_conj;
    quat_conjugate(&q_des_conj, q_desired);
    quat_multiply(q_err, &q_des_conj, q_current);
    quat_normalise(q_err);
}

/* ── Pointing error magnitude in degrees ────────────────────────────────── */
/*
 * Extracts the angular magnitude of the attitude error.
 * The angle is 2*acos(|w|) for a unit quaternion.
 * Clamp w to [-1, 1] to avoid acos domain errors from
 * floating point values just outside this range.
 */
float quat_pointing_error_deg(const Quat *q_err) {
    float w = q_err->w;
    if (w >  1.0f) w =  1.0f;
    if (w < -1.0f) w = -1.0f;
    return 2.0f * acosf(fabsf(w)) * (180.0f / 3.14159265f);
}

/* ── Build nadir-pointing quaternion ─────────────────────────────────────── */
/*
 * For nadir pointing, the satellite -Z axis must point toward Earth.
 * Given the satellite position vector (from Earth centre),
 * compute the quaternion that aligns -Z with the nadir direction.
 * This is the reference attitude for Mode 3 PID.
 *
 * In simulation: returns a simple identity quaternion.
 * On real hardware: use position from SGP4 and orbital mechanics.
 */
void quat_nadir_reference(Quat *q_nadir,
                           float pos_x, float pos_y, float pos_z) {
    /* Nadir direction = -position / |position| */
    float mag = sqrtf(pos_x*pos_x + pos_y*pos_y + pos_z*pos_z);
    if (mag < 1.0f) {
        quat_identity(q_nadir);
        return;
    }
    float nx = -pos_x / mag;
    float ny = -pos_y / mag;
    float nz = -pos_z / mag;

    /*
     * Axis-angle to quaternion:
     * Find rotation from [0,0,-1] (body -Z) to nadir direction.
     * Axis = cross([0,0,-1], nadir), angle = acos(dot([0,0,-1], nadir))
     */
    float dot   = -nz;   /* dot([0,0,-1], [nx,ny,nz]) = -nz */
    float ax    = -ny;   /* cross([0,0,-1], [nx,ny,nz]) */
    float ay    =  nx;
    float az    =  0.0f;

    float axis_mag = sqrtf(ax*ax + ay*ay + az*az);
    if (axis_mag < 1e-6f) {
        /* Already aligned or anti-aligned */
        if (dot > 0.0f)
            quat_identity(q_nadir);
        else {
            /* 180 deg rotation around X */
            q_nadir->w = 0.0f;
            q_nadir->x = 1.0f;
            q_nadir->y = 0.0f;
            q_nadir->z = 0.0f;
        }
        return;
    }

    float angle = acosf(dot > 1.0f ? 1.0f : dot < -1.0f ? -1.0f : dot);
    float s     = sinf(angle / 2.0f);
    q_nadir->w  = cosf(angle / 2.0f);
    q_nadir->x  = (ax / axis_mag) * s;
    q_nadir->y  = (ay / axis_mag) * s;
    q_nadir->z  = (az / axis_mag) * s;
    quat_normalise(q_nadir);
}
