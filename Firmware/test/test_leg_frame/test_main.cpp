#include <unity.h>
#include "leg.h"

const float TOL = 0.001f;

void setUp(void) {}
void tearDown(void) {}

static void assertVec3(Vec3 expected, Vec3 actual, const char *message) {
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL, expected.x, actual.x, message);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL, expected.y, actual.y, message);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL, expected.z, actual.z, message);
}

void test_leg_A_foot_body_to_leg(void) {
    Leg a = makeLeg(LEGS[0]);
    Vec3 footBody = { -405, 0, 20 };
    Vec3 expected = { 309, 0, -40 };
    assertVec3(expected, bodyToLeg(a, footBody), "A foot");
}

void test_leg_A_foot_leg_to_body(void) {
    Leg a = makeLeg(LEGS[0]);
    Vec3 footLeg = { 309, 0, -40 };
    Vec3 expected = { -405, 0, 20 };
    assertVec3(expected, legToBody(a, footLeg), "A foot back");
}

void test_leg_D_foot_is_only_shifted(void) {
    Leg d = makeLeg(LEGS[3]);
    Vec3 footBody = { 405, 0, 20 };
    Vec3 expected = { 309, 0, -40 };
    assertVec3(expected, bodyToLeg(d, footBody), "D foot");
}

void test_all_legs_mount_is_origin_and_foot_on_x_axis(void) {
    Vec3 origin = { 0, 0, 0 };
    for (int i = 0; i < LEG_COUNT; i++) {
        Leg leg = makeLeg(LEGS[i]);
        assertVec3(origin, bodyToLeg(leg, LEGS[i].p1), LEGS[i].name);

        Vec3 foot = bodyToLeg(leg, LEGS[i].p4);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL, 0.0f, foot.y, LEGS[i].name);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL, -40.0f, foot.z, LEGS[i].name);
        TEST_ASSERT_TRUE_MESSAGE(foot.x > 300.0f, LEGS[i].name);
    }
}

void test_round_trip_all_legs(void) {
    const int POINT_COUNT = 4;
    Vec3 points[POINT_COUNT] = {
        { 0, 0, 0 },
        { 123.4f, -56.7f, 8.9f },
        { -305, 0, 0 },
        { 400, 300, -50 },
    };
    for (int i = 0; i < LEG_COUNT; i++) {
        Leg leg = makeLeg(LEGS[i]);
        for (int k = 0; k < POINT_COUNT; k++) {
            Vec3 p = points[k];
            assertVec3(p, legToBody(leg, bodyToLeg(leg, p)), LEGS[i].name);
            assertVec3(p, bodyToLeg(leg, legToBody(leg, p)), LEGS[i].name);
        }
    }
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_leg_A_foot_body_to_leg);
    RUN_TEST(test_leg_A_foot_leg_to_body);
    RUN_TEST(test_leg_D_foot_is_only_shifted);
    RUN_TEST(test_all_legs_mount_is_origin_and_foot_on_x_axis);
    RUN_TEST(test_round_trip_all_legs);
    return UNITY_END();
}
