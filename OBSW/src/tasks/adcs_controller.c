/*
 * SLARS-BCG-1 — ADCS Controller Task
 * Priority 7 | Period 100ms | Stack 4KB
 *
 * Mode 1 B-dot:  m = -k * dB/dt
 * Mode 2 PD:     T = -Kp*e_att - Kd*e_rate
 * Mode 3 PID:    T = -Kp*e - Ki*integral(e) - Kd*de/dt
 *
 * All gains from Step 5 Section 6.
 */
#include "../hal/slars_types.h"
#include "../algorithms/ekf.h"
#include "../algorithms/quat.h"

#define K_BDOT   2e4f
#define KP_PD    5e-5f
#define KD_PD    3e-4f
#define KP_PID   8e-5f
#define KI_PID   2e-6f
#define KD_PID   5e-4f
#define I_CLAMP  0.5f
#define DT       0.1f

static uint8_t adcs_mode_state = ADCS_MODE_1_BDOT;
static float   integral[3]    = {0};
static float   prev_B[3]      = {0};
static float   prev_err[3]    = {0};
static int     adcs_cycle     = 0;
static EKF_State ekf;
static uint8_t   ekf_ready = 0;

uint8_t adcs_get_mode(void) { return adcs_mode_state; }

void adcs_set_mode(uint8_t m) {
    if (m != adcs_mode_state) {
        integral[0]=integral[1]=integral[2]=0;
        prev_err[0]=prev_err[1]=prev_err[2]=0;
        const char *names[]={"","B-dot","Coarse PD","Nadir PID"};
        printf("[ADCS] Mode: %s -> %s\n",
               names[adcs_mode_state<4?adcs_mode_state:0],
               names[m<4?m:0]);
        adcs_mode_state = m;
        sim_adcs_mode   = m;
    }
}

static void mode1_bdot(void) {
    float B[3];
    magnetometer_read(B);
    float m_cmd[3];
    m_cmd[0] = -K_BDOT * (B[0]-prev_B[0]) / DT;
    m_cmd[1] = -K_BDOT * (B[1]-prev_B[1]) / DT;
    m_cmd[2] = -K_BDOT * (B[2]-prev_B[2]) / DT;
    magnetorquer_set_dipole(m_cmd[0], m_cmd[1], m_cmd[2]);
    prev_B[0]=B[0]; prev_B[1]=B[1]; prev_B[2]=B[2];

    float rate = adcs_read_angular_rate();
    if (rate < 3.0f) {
        printf("[ADCS] Detumbling complete! rate=%.2f deg/s -> Mode 2\n",
               rate);
        adcs_set_mode(ADCS_MODE_2_COARSE);
    }
}

static void mode2_pd(void) {
    float e[3], er[3];
    ekf_get_attitude_error(e);
    gyroscope_read(er);
    float T[3];
    for (int i=0;i<3;i++)
        T[i] = -KP_PD*e[i] - KD_PD*er[i];
    magnetorquer_set_torque(T[0], T[1]);
    reaction_wheel_set_torque(T[2]);
}

static void mode3_pid(void) {
    float e[3], er[3];
    ekf_get_nadir_error(e);
    gyroscope_read(er);
    float T[3];
    for (int i=0;i<3;i++) {
        integral[i] += e[i]*DT;
        if (integral[i]>I_CLAMP)  integral[i]= I_CLAMP;
        if (integral[i]<-I_CLAMP) integral[i]=-I_CLAMP;
        float de = (e[i]-prev_err[i])/DT;
        T[i] = -KP_PID*e[i] - KI_PID*integral[i] - KD_PID*de;
        prev_err[i]=e[i];
    }
    magnetorquer_set_torque(T[0], T[1]);
    reaction_wheel_set_torque(T[2]);

    int rw = adcs_read_rw_speed();
    if (abs(rw) > 4500) {
        printf("[ADCS] RW desaturation: %d RPM\n", rw);
        fdir_log("RW_DESAT: %d RPM", rw);
        adcs_begin_desaturation();
    }
}

void adcs_controller_task(void) {
    watchdog_task_kick(TASK_ADCS);

    /* Initialise EKF on first run */
    if (!ekf_ready) {
        ekf_init(&ekf, 0.1f);   /* 100ms step */
        ekf_ready = 1;
        printf("[ADCS] EKF initialised\n");
    }

    /* Read sensors */
    float gyro[3], mag[3];
    gyroscope_read(gyro);
    magnetometer_read(mag);

    /* IGRF reference field for Sri Lanka region (simplified) */
    /* Real implementation uses full IGRF-13 coefficients */
    /* At 500 km over Sri Lanka: approx 25e-6, 5e-6, -40e-6 Tesla */
    float mag_ref[3] = {25e-6f, 5e-6f, -40e-6f};

    /* Run EKF */
    ekf_propagate(&ekf, gyro);
    ekf_update_magnetometer(&ekf, mag, mag_ref);

    /* Get EKF estimates — replace simulation variable reads */
    float pointing_err = ekf_get_pointing_error_deg(&ekf, 0.0f, 6871000.0f, 0.0f);
    float ang_rate     = ekf_get_rate_degs(&ekf, gyro);

    adcs_cycle++;

    /* Execute Control Logic based on state */
    switch(adcs_mode_state) {
        case ADCS_MODE_1_BDOT:   mode1_bdot(); break;
        case ADCS_MODE_2_COARSE: mode2_pd();   break;
        case ADCS_MODE_3_NADIR:  mode3_pid();  break;
    }

    /* Send quaternion to COMMS at 1 Hz (every 10 cycles of 100ms) */
    static int hz_ctr = 0;
    if (++hz_ctr >= 10) {
        hz_ctr = 0;
        float q[4];
        ekf_get_quaternion(q);
        comms_update_doppler_quaternion(q);
    }

    /* Telemetry output at 0.2 Hz */
    if (adcs_cycle % 50 == 0) {
        const char *mn[] = {"", "B-dot", "Coarse", "Nadir"};
        printf("[ADCS] Mode=%s rate=%.2f deg/s err=%.1f deg RW=%d RPM\n",
               mn[adcs_mode_state < 4 ? adcs_mode_state : 0],
               ang_rate,
               pointing_err,
               adcs_read_rw_speed());
    }
}
