#include <math.h>
#include <stdint.h>
#include <unity.h>
#include "leg.h"

const float TOL_DEG = 0.1f;

void setUp(void) {}
void tearDown(void) {}

static JointAngles makeAngles(float coxa, float femur, float tibia) {
    JointAngles a = { coxa, femur, tibia };
    return a;
}

static void assertSafe(const Leg &leg, JointAngles a, const char *message) {
    TEST_ASSERT_FLOAT_IS_DETERMINATE_MESSAGE(a.coxa, message);
    TEST_ASSERT_FLOAT_IS_DETERMINATE_MESSAGE(a.femur, message);
    TEST_ASSERT_FLOAT_IS_DETERMINATE_MESSAGE(a.tibia, message);
    TEST_ASSERT_TRUE_MESSAGE(a.coxa >= leg.coxaLimits.minDeg && a.coxa <= leg.coxaLimits.maxDeg, message);
    TEST_ASSERT_TRUE_MESSAGE(a.femur >= leg.femurLimits.minDeg && a.femur <= leg.femurLimits.maxDeg, message);
    TEST_ASSERT_TRUE_MESSAGE(a.tibia >= leg.tibiaLimits.minDeg && a.tibia <= leg.tibiaLimits.maxDeg, message);
}

void test_point_1000mm_away_is_out_of_reach(void) {
    Leg a = makeLeg(LEGS[0]);
    Vec3 far = { -1000, 0, 0 };
    JointAngles angles;
    TEST_ASSERT_EQUAL_INT(IK_OUT_OF_REACH, legIK(a, far, angles));
    assertSafe(a, angles, "1000 mm");
}

void test_nan_and_infinity_input_give_safe_zero_pose(void) {
    Leg a = makeLeg(LEGS[0]);
    Vec3 bad[6] = {
        { NAN, 0, 0 }, { 0, NAN, 0 }, { 0, 0, NAN },
        { INFINITY, 0, 0 }, { 0, -INFINITY, 0 }, { NAN, INFINITY, -INFINITY },
    };
    for (int k = 0; k < 6; k++) {
        JointAngles angles;
        TEST_ASSERT_EQUAL_INT(IK_OUT_OF_REACH, legIK(a, bad[k], angles));
        assertSafe(a, angles, "bad input");
        TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, 0.0f, angles.coxa);
        TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, 0.0f, angles.femur);
        TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, 0.0f, angles.tibia);
    }
}

void test_degenerate_geometry_gives_no_nan(void) {
    Leg weird = makeLeg(LEGS[3]);
    weird.tibiaLength = weird.femurLength;
    JointAngles angles;
    TEST_ASSERT_EQUAL_INT(IK_OUT_OF_REACH, legIK(weird, LEGS[3].p2, angles));
    assertSafe(weird, angles, "degenerate");
}

void test_coxa_past_limit_is_cut(void) {
    Leg d = makeLeg(LEGS[3]);
    Vec3 zeroFoot = bodyToLeg(d, LEGS[3].p4);
    JointAngles angles;

    Vec3 left = legToBody(d, rotateZ(zeroFoot, degToRad(80.0f)));
    TEST_ASSERT_EQUAL_INT(IK_LIMIT_HIT, legIK(d, left, angles));
    TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, 50.0f, angles.coxa);
    TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, 0.0f, angles.femur);
    TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, 0.0f, angles.tibia);

    Vec3 right = legToBody(d, rotateZ(zeroFoot, degToRad(-80.0f)));
    TEST_ASSERT_EQUAL_INT(IK_LIMIT_HIT, legIK(d, right, angles));
    TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, -50.0f, angles.coxa);
}

void test_femur_past_limit_is_cut(void) {
    Leg d = makeLeg(LEGS[3]);
    Vec3 target = legFK(d, makeAngles(0, 100, 0));
    JointAngles angles;
    TEST_ASSERT_EQUAL_INT(IK_LIMIT_HIT, legIK(d, target, angles));
    TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, 90.0f, angles.femur);
    TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, 0.0f, angles.tibia);
}

void test_tibia_past_limit_is_cut(void) {
    Leg d = makeLeg(LEGS[3]);
    Vec3 target = legFK(d, makeAngles(0, 0, -130));
    JointAngles angles;
    TEST_ASSERT_EQUAL_INT(IK_LIMIT_HIT, legIK(d, target, angles));
    TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, 0.0f, angles.femur);
    TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, -110.0f, angles.tibia);
}

void test_just_inside_limits_is_ok(void) {
    Leg d = makeLeg(LEGS[3]);
    JointAngles angles;
    TEST_ASSERT_EQUAL_INT(IK_OK, legIK(d, legFK(d, makeAngles(49, 89, -109)), angles));
    TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, 49.0f, angles.coxa);
    TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, 89.0f, angles.femur);
    TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, -109.0f, angles.tibia);
}

void test_out_of_reach_wins_over_limit_hit(void) {
    Leg d = makeLeg(LEGS[3]);
    Vec3 farLeftLeg = { 1000 * cosf(degToRad(80.0f)), 1000 * sinf(degToRad(80.0f)), 0 };
    JointAngles angles;
    TEST_ASSERT_EQUAL_INT(IK_OUT_OF_REACH, legIK(d, legToBody(d, farLeftLeg), angles));
    TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, 50.0f, angles.coxa);
    assertSafe(d, angles, "far + limit");
}

static uint32_t rngState = 777;

static float randomFloat(float minValue, float maxValue) {
    rngState = rngState * 1103515245u + 12345u;
    uint32_t bits = (rngState >> 16) & 0x7FFF;
    return minValue + (maxValue - minValue) * (bits / 32767.0f);
}

void test_random_and_extreme_points_are_always_safe(void) {
    rngState = 777;
    Vec3 extreme[5] = {
        { 1e30f, 1e30f, 1e30f }, { -1e30f, 0, 1e30f }, { 0, 0, -1e30f },
        { 1e-30f, -1e-30f, 0 }, { -0.0f, -0.0f, -0.0f },
    };
    for (int i = 0; i < LEG_COUNT; i++) {
        Leg leg = makeLeg(LEGS[i]);
        JointAngles angles;
        for (int k = 0; k < 2000; k++) {
            Vec3 p = { randomFloat(-1500, 1500), randomFloat(-1500, 1500), randomFloat(-1500, 1500) };
            IkStatus status = legIK(leg, p, angles);
            TEST_ASSERT_TRUE(status == IK_OK || status == IK_OUT_OF_REACH || status == IK_LIMIT_HIT);
            assertSafe(leg, angles, LEGS[i].name);
        }
        for (int k = 0; k < 5; k++) {
            legIK(leg, extreme[k], angles);
            assertSafe(leg, angles, "extreme");
        }
    }
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_point_1000mm_away_is_out_of_reach);
    RUN_TEST(test_nan_and_infinity_input_give_safe_zero_pose);
    RUN_TEST(test_degenerate_geometry_gives_no_nan);
    RUN_TEST(test_coxa_past_limit_is_cut);
    RUN_TEST(test_femur_past_limit_is_cut);
    RUN_TEST(test_tibia_past_limit_is_cut);
    RUN_TEST(test_just_inside_limits_is_ok);
    RUN_TEST(test_out_of_reach_wins_over_limit_hit);
    RUN_TEST(test_random_and_extreme_points_are_always_safe);
    return UNITY_END();
}
