#include <unity.h>
#include "mathx.h"
#include "config.h"

const float EPS = 0.00001f;

void setUp(void) {}
void tearDown(void) {}

static void assertVec3(Vec3 expected, Vec3 actual) {
    TEST_ASSERT_FLOAT_WITHIN(EPS, expected.x, actual.x);
    TEST_ASSERT_FLOAT_WITHIN(EPS, expected.y, actual.y);
    TEST_ASSERT_FLOAT_WITHIN(EPS, expected.z, actual.z);
}

void test_vecAdd(void) {
    Vec3 a = { 1, 2, 3 };
    Vec3 b = { 4, 5, 6 };
    Vec3 expected = { 5, 7, 9 };
    assertVec3(expected, vecAdd(a, b));
}

void test_vecSub(void) {
    Vec3 a = { 5, 7, 9 };
    Vec3 b = { 4, 5, 6 };
    Vec3 expected = { 1, 2, 3 };
    assertVec3(expected, vecSub(a, b));
}

void test_vecScale(void) {
    Vec3 v = { 1, -2, 3 };
    Vec3 expected = { 2, -4, 6 };
    assertVec3(expected, vecScale(v, 2.0f));
}

void test_vecLength_3_4_0_is_5(void) {
    Vec3 v = { 3, 4, 0 };
    TEST_ASSERT_FLOAT_WITHIN(EPS, 5.0f, vecLength(v));
}

void test_vecLength_zero_vector(void) {
    Vec3 v = { 0, 0, 0 };
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, vecLength(v));
}

void test_vecDot(void) {
    Vec3 a = { 1, 2, 3 };
    Vec3 b = { 4, 5, 6 };
    TEST_ASSERT_FLOAT_WITHIN(EPS, 32.0f, vecDot(a, b));
}

void test_vecDot_perpendicular_is_0(void) {
    Vec3 a = { 1, 0, 0 };
    Vec3 b = { 0, 1, 0 };
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, vecDot(a, b));
}

void test_degToRad_180_is_pi(void) {
    TEST_ASSERT_FLOAT_WITHIN(EPS, 3.14159265f, degToRad(180.0f));
}

void test_radToDeg_pi_is_180(void) {
    TEST_ASSERT_FLOAT_WITHIN(EPS, 180.0f, radToDeg(MATHX_PI));
}

void test_deg_rad_round_trip(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -37.5f, radToDeg(degToRad(-37.5f)));
}

void test_clamp_above_max(void) {
    TEST_ASSERT_FLOAT_WITHIN(EPS, 10.0f, clamp(15.0f, 0.0f, 10.0f));
}

void test_clamp_below_min(void) {
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, clamp(-5.0f, 0.0f, 10.0f));
}

void test_clamp_inside_range(void) {
    TEST_ASSERT_FLOAT_WITHIN(EPS, 5.0f, clamp(5.0f, 0.0f, 10.0f));
}

void test_rotateZ_x_by_90_gives_y(void) {
    Vec3 v = { 1, 0, 0 };
    Vec3 expected = { 0, 1, 0 };
    assertVec3(expected, rotateZ(v, degToRad(90.0f)));
}

void test_rotateZ_x_by_minus_90_gives_minus_y(void) {
    Vec3 v = { 1, 0, 0 };
    Vec3 expected = { 0, -1, 0 };
    assertVec3(expected, rotateZ(v, degToRad(-90.0f)));
}

void test_rotateZ_by_180(void) {
    Vec3 v = { 2, 3, 0 };
    Vec3 expected = { -2, -3, 0 };
    assertVec3(expected, rotateZ(v, degToRad(180.0f)));
}

void test_rotateZ_keeps_z_and_length(void) {
    Vec3 v = { 3, 4, 7 };
    Vec3 r = rotateZ(v, degToRad(33.0f));
    TEST_ASSERT_FLOAT_WITHIN(EPS, 7.0f, r.z);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, vecLength(v), vecLength(r));
}

void test_rotateZ_leg_D_by_30_points_like_leg_C(void) {
    const LegConfig &c = LEGS[2];
    const LegConfig &d = LEGS[3];
    Vec3 coxaD = { d.p2.x - d.p1.x, d.p2.y - d.p1.y, d.p2.z - d.p1.z };
    Vec3 coxaC = { c.p2.x - c.p1.x, c.p2.y - c.p1.y, c.p2.z - c.p1.z };
    Vec3 r = rotateZ(coxaD, degToRad(30.0f));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, coxaC.x, r.x);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, coxaC.y, r.y);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, coxaC.z, r.z);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_vecAdd);
    RUN_TEST(test_vecSub);
    RUN_TEST(test_vecScale);
    RUN_TEST(test_vecLength_3_4_0_is_5);
    RUN_TEST(test_vecLength_zero_vector);
    RUN_TEST(test_vecDot);
    RUN_TEST(test_vecDot_perpendicular_is_0);
    RUN_TEST(test_degToRad_180_is_pi);
    RUN_TEST(test_radToDeg_pi_is_180);
    RUN_TEST(test_deg_rad_round_trip);
    RUN_TEST(test_clamp_above_max);
    RUN_TEST(test_clamp_below_min);
    RUN_TEST(test_clamp_inside_range);
    RUN_TEST(test_rotateZ_x_by_90_gives_y);
    RUN_TEST(test_rotateZ_x_by_minus_90_gives_minus_y);
    RUN_TEST(test_rotateZ_by_180);
    RUN_TEST(test_rotateZ_keeps_z_and_length);
    RUN_TEST(test_rotateZ_leg_D_by_30_points_like_leg_C);
    return UNITY_END();
}
