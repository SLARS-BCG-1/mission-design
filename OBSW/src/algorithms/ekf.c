/*
 * SLARS-BCG-1 Flight Software
 * File: ekf.c
 *
 * Extended Kalman Filter for satellite attitude determination.
 *
 * UPDATE ALGORITHM:
 *   Rotate body-frame magnetometer reading to inertial frame using
 *   current q estimate, compare with IGRF reference, use the
 *   difference to correct q via left-multiply.
 *
 * SLARS Team -- Bandaranayake College, Gampaha
 */

#include <math.h>
#include <string.h>
#include "ekf.h"
#include "quat.h"

#define PI 3.14159265358979f

static float vec3_norm(const float v[3]) {
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}
static void vec3_normalise(float out[3], const float v[3]) {
    float n = vec3_norm(v);
    if (n < 1e-10f) { out[0]=0; out[1]=0; out[2]=0; return; }
    out[0]=v[0]/n; out[1]=v[1]/n; out[2]=v[2]/n;
}
static void vec3_cross(float out[3], const float a[3], const float b[3]) {
    out[0]=a[1]*b[2]-a[2]*b[1];
    out[1]=a[2]*b[0]-a[0]*b[2];
    out[2]=a[0]*b[1]-a[1]*b[0];
}

void ekf_init(EKF_State *ekf, float dt_seconds) {
    memset(ekf, 0, sizeof(EKF_State));
    quat_identity(&ekf->q);
    ekf->bias[0]=0.0f; ekf->bias[1]=0.0f; ekf->bias[2]=0.0f;
    ekf->dt=dt_seconds;
    ekf->Q_att=1e-6f; ekf->Q_bias=1e-8f; ekf->R_mag=0.01f;
    ekf->initialised=0;
    for (int i=0;i<7;i++) for (int j=0;j<7;j++) ekf->P[i][j]=0.0f;
    ekf->P[0][0]=0.1f; ekf->P[1][1]=0.1f;
    ekf->P[2][2]=0.1f; ekf->P[3][3]=0.1f;
    ekf->P[4][4]=1e-4f; ekf->P[5][5]=1e-4f; ekf->P[6][6]=1e-4f;
}

void ekf_propagate(EKF_State *ekf, const float gyro_rads[3]) {
    float wx=gyro_rads[0]-ekf->bias[0];
    float wy=gyro_rads[1]-ekf->bias[1];
    float wz=gyro_rads[2]-ekf->bias[2];
    float dt=ekf->dt;
    float mag=sqrtf(wx*wx+wy*wy+wz*wz);
    Quat dq;
    if (mag < 1e-8f) {
        dq.w=1.0f; dq.x=0.0f; dq.y=0.0f; dq.z=0.0f;
    } else {
        float ha=mag*dt*0.5f;
        float s=sinf(ha)/mag;
        dq.w=cosf(ha); dq.x=wx*s; dq.y=wy*s; dq.z=wz*s;
    }
    Quat qn; quat_multiply(&qn,&ekf->q,&dq);
    quat_normalise(&qn); quat_copy(&ekf->q,&qn);
    for (int i=0;i<4;i++) ekf->P[i][i]+=ekf->Q_att;
    for (int i=4;i<7;i++) ekf->P[i][i]+=ekf->Q_bias;
}

/*
 * Magnetometer update -- CORRECT ALGORITHM
 *
 * Step 1: rotate body-frame measurement into inertial frame:
 *           b_inertial = q * mag_body * q_conj
 * Step 2: find the angle/axis between b_inertial and mag_ref
 *           error_axis  = cross(b_inertial_norm, mag_ref_norm)
 *           error_angle = acos(dot(b_inertial_norm, mag_ref_norm))
 * Step 3: build correction quaternion and left-multiply q
 *           q_new = dq_corr * q
 */
void ekf_update_magnetometer(EKF_State *ekf,
                               const float mag_body[3],
                               const float mag_ref[3]) {
    if (!ekf->initialised) {
        /* Init: rotate q so mag_body aligns with mag_ref */
        float bn[3], rn[3];
        vec3_normalise(bn, mag_body);
        vec3_normalise(rn, mag_ref);
        float axis[3]; vec3_cross(axis, bn, rn);
        float am=vec3_norm(axis);
        if (am > 1e-6f) {
            float dot=bn[0]*rn[0]+bn[1]*rn[1]+bn[2]*rn[2];
            if(dot>1)dot=1; if(dot<-1)dot=-1;
            float angle=acosf(dot);
            float s=sinf(angle*0.5f)/am;
            ekf->q.w=cosf(angle*0.5f);
            ekf->q.x=axis[0]*s; ekf->q.y=axis[1]*s; ekf->q.z=axis[2]*s;
            quat_normalise(&ekf->q);
        }
        ekf->initialised=1;
        return;
    }

    /* Rotate body measurement to inertial frame */
    float b_inertial[3];
    quat_rotate_vector(b_inertial, &ekf->q, mag_body);

    float bn[3], rn[3];
    vec3_normalise(bn, b_inertial);
    vec3_normalise(rn, mag_ref);

    /* Error axis and angle */
    float error_axis[3];
    vec3_cross(error_axis, bn, rn);
    float am=vec3_norm(error_axis);

    float dot=bn[0]*rn[0]+bn[1]*rn[1]+bn[2]*rn[2];
    if(dot>1)dot=1; if(dot<-1)dot=-1;
    float error_angle=acosf(dot);

    if (am < 1e-8f || error_angle < 1e-7f) return;

    /* Kalman gain */
    float att_cov=(ekf->P[0][0]+ekf->P[1][1]+
                   ekf->P[2][2]+ekf->P[3][3])*0.25f;
    float K=att_cov/(att_cov+ekf->R_mag+1e-10f);

    /* Correction quaternion -- left-multiply (inertial frame) */
    float corr=K*error_angle;
    float s=sinf(corr*0.5f)/am;
    Quat dq;
    dq.w=cosf(corr*0.5f);
    dq.x=error_axis[0]*s; dq.y=error_axis[1]*s; dq.z=error_axis[2]*s;

    Quat qu; quat_multiply(&qu,&dq,&ekf->q);
    quat_normalise(&qu); quat_copy(&ekf->q,&qu);

    float scale=1.0f-K;
    if(scale<0.01f) scale=0.01f;
    for (int i=0;i<4;i++) ekf->P[i][i]*=scale;
}

void ekf_get_attitude(const EKF_State *ekf, Quat *q_out) {
    quat_copy(q_out,&ekf->q);
}

float ekf_get_pointing_error_deg(const EKF_State *ekf,
                                   float px, float py, float pz) {
    Quat qn; quat_nadir_reference(&qn,px,py,pz);
    Quat qe; quat_error(&qe,&qn,&ekf->q);
    return quat_pointing_error_deg(&qe);
}

float ekf_get_rate_degs(const EKF_State *ekf, const float gyro_rads[3]) {
    float wx=gyro_rads[0]-ekf->bias[0];
    float wy=gyro_rads[1]-ekf->bias[1];
    float wz=gyro_rads[2]-ekf->bias[2];
    return sqrtf(wx*wx+wy*wy+wz*wz)*(180.0f/PI);
}

void ekf_get_euler_deg(const EKF_State *ekf,
                         float *roll, float *pitch, float *yaw) {
    quat_to_euler_deg(&ekf->q,roll,pitch,yaw);
}

void ekf_reset(EKF_State *ekf) {
    float dt=ekf->dt;
    ekf_init(ekf,dt);
}
