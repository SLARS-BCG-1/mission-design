/*
 * SLARS-BCG-1 — EKF Unit Tests
 * test_ekf.c
 *
 * Build:
 *   gcc test_ekf.c \
 *       ../src/algorithms/ekf.c \
 *       ../src/algorithms/quat.c \
 *       -I../src/algorithms -lm -o test_ekf
 *   ./test_ekf
 *
 * SLARS Team — Bandaranayake College, Gampaha
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "../src/algorithms/ekf.h"
#include "../src/algorithms/quat.h"

static int passed = 0, total = 0;

#define CLOSE(a,b,tol) (fabsf((a)-(b)) <= (tol))

#define TEST(desc, cond) do { \
    total++; \
    if(cond){ passed++; printf("  PASS: %s\n", desc); } \
    else         printf("  FAIL: %s  (got condition false)\n", desc); \
} while(0)

/* ── Quaternion tests ────────────────────────────────────────────────────── */
void test_quat_identity(void) {
    printf("\n[TEST] Quaternion identity\n");
    Quat q;
    quat_identity(&q);
    TEST("Identity: w=1", CLOSE(q.w, 1.0f, 1e-6f));
    TEST("Identity: x=0", CLOSE(q.x, 0.0f, 1e-6f));
    TEST("Identity: y=0", CLOSE(q.y, 0.0f, 1e-6f));
    TEST("Identity: z=0", CLOSE(q.z, 0.0f, 1e-6f));
}

void test_quat_normalise(void) {
    printf("\n[TEST] Quaternion normalise\n");
    Quat q;
    q.w = 2.0f; q.x = 2.0f; q.y = 2.0f; q.z = 2.0f;
    quat_normalise(&q);
    float norm = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    TEST("Normalised quaternion has unit norm", CLOSE(norm, 1.0f, 1e-5f));

    /* Zero quaternion should return identity */
    q.w = 0.0f; q.x = 0.0f; q.y = 0.0f; q.z = 0.0f;
    quat_normalise(&q);
    TEST("Zero quaternion resets to identity", CLOSE(q.w, 1.0f, 1e-6f));
}

void test_quat_multiply(void) {
    printf("\n[TEST] Quaternion multiplication\n");
    Quat q, identity, result;
    quat_identity(&identity);
    /* 90 deg rotation around Z */
    float s = sinf(3.14159f/4.0f);
    float c = cosf(3.14159f/4.0f);
    q.w = c; q.x = 0.0f; q.y = 0.0f; q.z = s;

    /* q * identity = q */
    quat_multiply(&result, &q, &identity);
    TEST("q * identity = q (w)", CLOSE(result.w, q.w, 1e-5f));
    TEST("q * identity = q (z)", CLOSE(result.z, q.z, 1e-5f));

    /* identity * q = q */
    quat_multiply(&result, &identity, &q);
    TEST("identity * q = q (w)", CLOSE(result.w, q.w, 1e-5f));
}

void test_quat_conjugate(void) {
    printf("\n[TEST] Quaternion conjugate\n");
    Quat q, q_conj, result;
    q.w = 0.7071f; q.x = 0.0f; q.y = 0.7071f; q.z = 0.0f;
    quat_normalise(&q);
    quat_conjugate(&q_conj, &q);

    /* q * q* should equal identity */
    quat_multiply(&result, &q, &q_conj);
    TEST("q * q_conj = identity (w~1)", CLOSE(result.w, 1.0f, 1e-5f));
    TEST("q * q_conj = identity (x~0)", CLOSE(result.x, 0.0f, 1e-5f));
    TEST("q * q_conj = identity (y~0)", CLOSE(result.y, 0.0f, 1e-5f));
}

void test_quat_rotate_vector(void) {
    printf("\n[TEST] Quaternion rotate vector\n");

    /* 90 deg rotation around Z — X axis should become Y axis */
    Quat q;
    q.w = cosf(3.14159f/4.0f);
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = sinf(3.14159f/4.0f);
    quat_normalise(&q);

    float v_in[3]  = {1.0f, 0.0f, 0.0f};   /* X axis */
    float v_out[3] = {0.0f, 0.0f, 0.0f};
    quat_rotate_vector(v_out, &q, v_in);

    TEST("Rotate X by 90 deg around Z: result x ~ 0", CLOSE(v_out[0],  0.0f, 1e-4f));
    TEST("Rotate X by 90 deg around Z: result y ~ 1", CLOSE(v_out[1],  1.0f, 1e-4f));
    TEST("Rotate X by 90 deg around Z: result z ~ 0", CLOSE(v_out[2],  0.0f, 1e-4f));
}

void test_quat_pointing_error(void) {
    printf("\n[TEST] Quaternion pointing error\n");
    Quat q_des, q_cur, q_err;

    /* Same attitude — error should be 0 */
    quat_identity(&q_des);
    quat_identity(&q_cur);
    quat_error(&q_err, &q_des, &q_cur);
    float err = quat_pointing_error_deg(&q_err);
    TEST("Identical attitudes: pointing error = 0 deg", CLOSE(err, 0.0f, 0.01f));

    /* 45 deg rotation around Z */
    q_cur.w = cosf(3.14159f/8.0f);
    q_cur.x = 0.0f;
    q_cur.y = 0.0f;
    q_cur.z = sinf(3.14159f/8.0f);
    quat_normalise(&q_cur);
    quat_error(&q_err, &q_des, &q_cur);
    err = quat_pointing_error_deg(&q_err);
    TEST("45 deg rotation: pointing error ~ 45 deg", CLOSE(err, 45.0f, 0.5f));
}

/* ── EKF tests ───────────────────────────────────────────────────────────── */
void test_ekf_init(void) {
    printf("\n[TEST] EKF initialisation\n");
    EKF_State ekf;
    ekf_init(&ekf, 0.1f);

    TEST("EKF init: q is identity (w=1)",  CLOSE(ekf.q.w, 1.0f, 1e-6f));
    TEST("EKF init: bias x = 0",           CLOSE(ekf.bias[0], 0.0f, 1e-9f));
    TEST("EKF init: bias y = 0",           CLOSE(ekf.bias[1], 0.0f, 1e-9f));
    TEST("EKF init: dt = 0.1",             CLOSE(ekf.dt, 0.1f, 1e-6f));
    TEST("EKF init: not yet initialised",  ekf.initialised == 0);
}

void test_ekf_propagate_zero_rate(void) {
    printf("\n[TEST] EKF propagate — zero angular rate\n");
    EKF_State ekf;
    ekf_init(&ekf, 0.1f);

    float zero_rate[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 100; i++)
        ekf_propagate(&ekf, zero_rate);

    /* Zero rate: attitude should stay at identity */
    TEST("Zero rate: q.w stays near 1", CLOSE(ekf.q.w, 1.0f, 0.001f));
    TEST("Zero rate: q.x stays near 0", CLOSE(ekf.q.x, 0.0f, 0.001f));
    TEST("Zero rate: q.y stays near 0", CLOSE(ekf.q.y, 0.0f, 0.001f));
    TEST("Zero rate: quaternion stays unit norm",
         CLOSE(quat_norm(&ekf.q), 1.0f, 1e-4f));
}

void test_ekf_propagate_rotation(void) {
    printf("\n[TEST] EKF propagate — constant rotation rate\n");
    EKF_State ekf;
    ekf_init(&ekf, 0.1f);

    /*
     * Rotate at 1 deg/s around Z axis.
     * After 90 seconds, should be at ~90 deg.
     */
    float rate_rads[3] = {0.0f, 0.0f, 1.0f * 3.14159f/180.0f};

    for (int i = 0; i < 900; i++)   /* 90 seconds at 100ms steps */
        ekf_propagate(&ekf, rate_rads);

    float roll, pitch, yaw;
    ekf_get_euler_deg(&ekf, &roll, &pitch, &yaw);

    TEST("After 90s at 1 deg/s: yaw ~ 90 deg", CLOSE(yaw, 90.0f, 2.0f));
    TEST("Quaternion stays unit norm after 900 steps",
         CLOSE(quat_norm(&ekf.q), 1.0f, 1e-3f));
}

void test_ekf_update_magnetometer(void) {
    printf("\n[TEST] EKF magnetometer update\n");
    EKF_State ekf;
    ekf_init(&ekf, 0.1f);

    /*
     * Reference field pointing in Y direction (inertial frame)
     * Measured field also in Y direction (body frame)
     * => satellite is aligned with inertial frame
     */
    float mag_ref[3]  = {0.0f, 1.0f, 0.0f};
    float mag_body[3] = {0.0f, 1.0f, 0.0f};

    ekf_update_magnetometer(&ekf, mag_body, mag_ref);

    TEST("EKF initialised after first mag update", ekf.initialised == 1);
    TEST("After aligned update: q stays unit norm",
         CLOSE(quat_norm(&ekf.q), 1.0f, 1e-3f));
}

void test_ekf_convergence(void) {
    printf("\n[TEST] EKF convergence -- rotation error corrected\n");
    EKF_State ekf;
    ekf_init(&ekf, 0.1f);

    float angle_rad = 20.0f * 3.14159f / 180.0f;
    ekf.q.w = cosf(angle_rad / 2.0f);
    ekf.q.x = 0.0f;
    ekf.q.y = 0.0f;
    ekf.q.z = sinf(angle_rad / 2.0f);
    quat_normalise(&ekf.q);
    ekf.initialised = 1;

    ekf.P[0][0]=1.0f; ekf.P[1][1]=1.0f;
    ekf.P[2][2]=1.0f; ekf.P[3][3]=1.0f;
    ekf.R_mag = 0.01f;

    float mag_ref[3]   = {1.0f, 0.0f, 0.0f};
    float mag_body[3]  = {1.0f, 0.0f, 0.0f};
    float zero_rate[3] = {0.0f, 0.0f, 0.0f};

    Quat q_identity;
    quat_identity(&q_identity);
    Quat q_err;
    quat_error(&q_err, &q_identity, &ekf.q);
    float err_before = quat_pointing_error_deg(&q_err);

    for (int i = 0; i < 100; i++) {
        ekf_propagate(&ekf, zero_rate);
        ekf_update_magnetometer(&ekf, mag_body, mag_ref);
    }

    quat_error(&q_err, &q_identity, &ekf.q);
    float err_after = quat_pointing_error_deg(&q_err);

    printf("    Before: %.4f deg  ->  After: %.4f deg\n",
           err_before, err_after);

    TEST("EKF converges: final error < initial error",
         err_after < err_before);
    TEST("Quaternion stays unit norm during convergence",
         CLOSE(quat_norm(&ekf.q), 1.0f, 1e-3f));
}

void test_ekf_rate_correction(void) {
    printf("\n[TEST] EKF rate with bias\n");
    EKF_State ekf;
    ekf_init(&ekf, 0.1f);

    /* Set a known bias */
    ekf.bias[0] = 0.01f;    /* 0.01 rad/s ~ 0.57 deg/s bias on X */

    /* Gyro reports 0.02 rad/s on X — true rate = 0.02 - 0.01 = 0.01 rad/s */
    float gyro[3] = {0.02f, 0.0f, 0.0f};
    float corrected_rate = ekf_get_rate_degs(&ekf, gyro);

    /* Expected corrected rate: 0.01 rad/s * 180/pi ~ 0.573 deg/s */
    TEST("Bias-corrected rate is half the raw rate",
         CLOSE(corrected_rate, 0.01f * 180.0f/3.14159f, 0.01f));
}

void test_ekf_reset(void) {
    printf("\n[TEST] EKF reset\n");
    EKF_State ekf;
    ekf_init(&ekf, 0.1f);

    /* Corrupt the state */
    ekf.q.w = 0.1f; ekf.q.x = 0.9f;
    ekf.bias[0] = 1.0f;
    ekf.initialised = 1;

    ekf_reset(&ekf);

    TEST("After reset: q is identity", CLOSE(ekf.q.w, 1.0f, 1e-6f));
    TEST("After reset: bias is zero",  CLOSE(ekf.bias[0], 0.0f, 1e-9f));
    TEST("After reset: not initialised", ekf.initialised == 0);
}

/* ── main ────────────────────────────────────────────────────────────────── */
int main(void) {
    printf("*******************************************************\n");
    printf("*   SLARS-BCG-1 EKF Unit Test Suite                   *\n");
    printf("*   SLARS Team — Bandaranayake College, Gampaha       *\n");
    printf("*******************************************************\n");

    test_quat_identity();
    test_quat_normalise();
    test_quat_multiply();
    test_quat_conjugate();
    test_quat_rotate_vector();
    test_quat_pointing_error();
    test_ekf_init();
    test_ekf_propagate_zero_rate();
    test_ekf_propagate_rotation();
    test_ekf_update_magnetometer();
    test_ekf_convergence();
    test_ekf_rate_correction();
    test_ekf_reset();

    printf("\n=======================================================\n");
    printf("  RESULTS: %d / %d tests passed\n", passed, total);
    if (passed == total)
        printf("  ALL TESTS PASSED\n");
    else
        printf("  %d FAILED\n", total - passed);
    printf("=======================================================\n\n");

    return (passed == total) ? 0 : 1;
}
