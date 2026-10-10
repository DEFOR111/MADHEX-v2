#include <unity.h>
#include "leg.h"

const float TOL_DEG = 0.1f;

void setUp(void) {}
void tearDown(void) {}

static void assertAngles(float coxa, float femur, float tibia, JointAngles actual, const char *message) {
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_DEG, coxa, actual.coxa, message);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_DEG, femur, actual.femur, message);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_DEG, tibia, actual.tibia, message);
}

void test_leg_A_zero_pose_P4_gives_zero_angles(void) {
    Leg a = makeLeg(LEGS[0]);
    JointAngles angles;
    TEST_ASSERT_EQUAL_INT(IK_OK, legIK(a, LEGS[0].p4, angles));
    assertAngles(0, 0, 0, angles, "A");
}

void test_all_legs_ideal_zero_pose_gives_zero_angles(void) {
    Leg a = makeLeg(LEGS[0]);
    Vec3 zeroFootLegFrame = bodyToLeg(a, LEGS[0].p4);
    for (int i = 0; i < LEG_COUNT; i++) {
        Leg leg = makeLeg(LEGS[i]);
        Vec3 foot = legToBody(leg, zeroFootLegFrame);
        JointAngles angles;
        TEST_ASSERT_EQUAL_INT_MESSAGE(IK_OK, legIK(leg, foot, angles), LEGS[i].name);
        assertAngles(0, 0, 0, angles, LEGS[i].name);
    }
}

void test_leg_A_standing_foot(void) {
    Leg a = makeLeg(LEGS[0]);
    JointAngles angles;
    TEST_ASSERT_EQUAL_INT(IK_OK, legIK(a, STAND_FOOT_A, angles));
    assertAngles(0.0f, 33.1f, -83.1f, angles, "A standing");
}

void test_leg_B_cad_P4_shows_rounding_effect(void) {
    Leg b = makeLeg(LEGS[1]);
    JointAngles angles;
    TEST_ASSERT_EQUAL_INT(IK_OK, legIK(b, LEGS[1].p4, angles));
    assertAngles(0.0f, 1.43f, -2.78f, angles, "B CAD P4");
}

void test_coxa_follows_target_turn(void) {
    Leg d = makeLeg(LEGS[3]);
    Vec3 zeroFoot = bodyToLeg(d, LEGS[3].p4);
    Vec3 turnedFoot = legToBody(d, rotateZ(zeroFoot, degToRad(20.0f)));
    JointAngles angles;
    TEST_ASSERT_EQUAL_INT(IK_OK, legIK(d, turnedFoot, angles));
    assertAngles(20.0f, 0.0f, 0.0f, angles, "D turned");
}

void test_too_far_is_not_reachable(void) {
    Leg a = makeLeg(LEGS[0]);
    Vec3 farAway = { -1000, 0, 60 };
    JointAngles angles;
    TEST_ASSERT_EQUAL_INT(IK_OUT_OF_REACH, legIK(a, farAway, angles));
    TEST_ASSERT_FLOAT_IS_NOT_NAN(angles.coxa);
    TEST_ASSERT_FLOAT_IS_NOT_NAN(angles.femur);
    TEST_ASSERT_FLOAT_IS_NOT_NAN(angles.tibia);
}

void test_target_on_femur_axis_is_not_reachable(void) {
    Leg a = makeLeg(LEGS[0]);
    JointAngles angles;
    TEST_ASSERT_EQUAL_INT(IK_OUT_OF_REACH, legIK(a, LEGS[0].p2, angles));
    TEST_ASSERT_FLOAT_IS_NOT_NAN(angles.coxa);
    TEST_ASSERT_FLOAT_IS_NOT_NAN(angles.femur);
    TEST_ASSERT_FLOAT_IS_NOT_NAN(angles.tibia);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_leg_A_zero_pose_P4_gives_zero_angles);
    RUN_TEST(test_all_legs_ideal_zero_pose_gives_zero_angles);
    RUN_TEST(test_leg_A_standing_foot);
    RUN_TEST(test_leg_B_cad_P4_shows_rounding_effect);
    RUN_TEST(test_coxa_follows_target_turn);
    RUN_TEST(test_too_far_is_not_reachable);
    RUN_TEST(test_target_on_femur_axis_is_not_reachable);
    return UNITY_END();
}
