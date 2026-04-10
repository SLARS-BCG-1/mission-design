/*
 * SLARS-BCG-1 — Quaternion header
 * quat.h
 *
 * SLARS Team — Bandaranayake College, Gampaha
 */

#ifndef QUAT_H
#define QUAT_H

/* Quaternion type: w is the scalar, x/y/z is the vector part */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quat;

/* Initialise and copy */
void  quat_identity(Quat *q);
void  quat_copy(Quat *dst, const Quat *src);

/* Core operations */
void  quat_normalise(Quat *q);
float quat_norm(const Quat *q);
void  quat_conjugate(Quat *out, const Quat *q);
void  quat_multiply(Quat *out, const Quat *q1, const Quat *q2);

/* Rotation and conversion */
void  quat_rotate_vector(float out[3], const Quat *q, const float v[3]);
void  quat_to_euler_deg(const Quat *q,
                         float *roll_deg,
                         float *pitch_deg,
                         float *yaw_deg);

/* ADCS helpers */
void  quat_error(Quat *q_err,
                  const Quat *q_desired,
                  const Quat *q_current);
float quat_pointing_error_deg(const Quat *q_err);
void  quat_nadir_reference(Quat *q_nadir,
                             float pos_x,
                             float pos_y,
                             float pos_z);

#endif /* QUAT_H */
