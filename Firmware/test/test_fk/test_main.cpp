#include <stdint.h>
#include <stdio.h>
#include <unity.h>
#include "leg.h"

void setUp(void) {}
void tearDown(void) {}

static void assertVec3(Vec3 expected, Vec3 actual, float tol, const char *message) {
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(tol, expected.x, actual.x, message);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(tol, expected.y, actual.y, message);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(tol, expected.z, actual.z, message);
}

static JointAngles makeAngles(float coxa, float femur, float tibia) {
    JointAngles a = { coxa, femur, tibia };
    return a;
}

void test_fk_zero_is_P4_for_legs_A_and_D(void) {
    Leg a = makeLeg(LEGS[0]);
    Leg d = makeLeg(LEGS[3]);
    assertVec3(LEGS[0].p4, legFK(a, makeAngles(0, 0, 0)), 0.01f, "A");
    assertVec3(LEGS[3].p4, legFK(d, makeAngles(0, 0, 0)), 0.01f, "D");
}

void test_fk_zero_is_ideal_zero_foot_for_all_legs(void) {
    Leg a = makeLeg(LEGS[0]);
    Vec3 zeroFootLegFrame = bodyToLeg(a, LEGS[0].p4);
    for (int i = 0; i < LEG_COUNT; i++) {
        Leg leg = makeLeg(LEGS[i]);
        Vec3 ideal = legToBody(leg, zeroFootLegFrame);
        assertVec3(ideal, legFK(leg, makeAngles(0, 0, 0)), 0.01f, LEGS[i].name);
    }
}

void test_leg_B_cad_P4_is_about_1mm_from_fk_zero(void) {
    Leg b = makeLeg(LEGS[1]);
    Vec3 diff = vecSub(legFK(b, makeAngles(0, 0, 0)), LEGS[1].p4);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.02f, vecLength(diff));
}

void test_fk_leg_D_coxa_plus_90(void) {
    Leg d = makeLeg(LEGS[3]);
    Vec3 expected = { 96, 309, 20 };
    assertVec3(expected, legFK(d, makeAngles(90, 0, 0)), 0.01f, "D coxa 90");
}

void test_fk_leg_D_femur_plus_90(void) {
    Leg d = makeLeg(LEGS[3]);
    Vec3 expected = { 96 + 45 + 40, 0, 60 + 264 };
    assertVec3(expected, legFK(d, makeAngles(0, 90, 0)), 0.01f, "D femur 90");
}

void test_fk_leg_D_straight_knee(void) {
    Leg d = makeLeg(LEGS[3]);
    float openToStraight = 180.0f - d.kneeZeroDeg;
    Vec3 expected = { 96 + 45 + 128 + 141.7604f, 0, 60 };
    assertVec3(expected, legFK(d, makeAngles(0, 0, openToStraight)), 0.01f, "D straight");
}

static uint32_t rngState = 12345;

static float randomFloat(float minValue, float maxValue) {
    rngState = rngState * 1103515245u + 12345u;
    uint32_t bits = (rngState >> 16) & 0x7FFF;
    return minValue + (maxValue - minValue) * (bits / 32767.0f);
}

const int POINTS_PER_LEG = 300;
const float ROUND_TRIP_TOL_MM = 0.5f;

void test_fk_of_ik_returns_the_point_for_random_points(void) {
    rngState = 12345;
    for (int i = 0; i < LEG_COUNT; i++) {
        Leg leg = makeLeg(LEGS[i]);
        int found = 0;
        int attempts = 0;
        float maxError = 0.0f;

        while (found < POINTS_PER_LEG && attempts < 100000) {
            attempts++;
            Vec3 pLeg = { randomFloat(-150, 350), randomFloat(-350, 350), randomFloat(-300, 250) };
            Vec3 p = legToBody(leg, pLeg);

            JointAngles angles;
            if (legIK(leg, p, angles) != IK_OK) {
                continue;
            }
            found++;

            float error = vecLength(vecSub(legFK(leg, angles), p));
            if (error > maxError) {
                maxError = error;
            }
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(ROUND_TRIP_TOL_MM, 0.0f, error, LEGS[i].name);
        }
        TEST_ASSERT_EQUAL_INT_MESSAGE(POINTS_PER_LEG, found, LEGS[i].name);

        char message[80];
        snprintf(message, sizeof(message), "leg %s: %d points, max error %.6f mm",
                 LEGS[i].name, found, maxError);
        TEST_MESSAGE(message);
    }
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_fk_zero_is_P4_for_legs_A_and_D);
    RUN_TEST(test_fk_zero_is_ideal_zero_foot_for_all_legs);
    RUN_TEST(test_leg_B_cad_P4_is_about_1mm_from_fk_zero);
    RUN_TEST(test_fk_leg_D_coxa_plus_90);
    RUN_TEST(test_fk_leg_D_femur_plus_90);
    RUN_TEST(test_fk_leg_D_straight_knee);
    RUN_TEST(test_fk_of_ik_returns_the_point_for_random_points);
    return UNITY_END();
}
