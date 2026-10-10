#include <math.h>
#include <unity.h>
#include "body.h"

const float TOL_DEG = 0.1f;
const float TOL_MM = 0.01f;

void setUp(void) {}
void tearDown(void) {}

void test_body_has_legs_A_to_F_in_order(void) {
    Body body = makeBody();
    const char *names[LEG_COUNT] = { "A", "B", "C", "D", "E", "F" };
    for (int i = 0; i < LEG_COUNT; i++) {
        TEST_ASSERT_EQUAL_STRING(names[i], body.legs[i].name);
    }
}

void test_cad_stance_radius_is_209(void) {
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, 209.0f, cadStanceRadius());
}

void test_neutral_foot_of_leg_A_is_cad_standing_foot(void) {
    Body body = makeBody();
    Vec3 foot = neutralFoot(body.legs[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, STAND_FOOT_A.x, foot.x);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, STAND_FOOT_A.y, foot.y);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, STAND_FOOT_A.z, foot.z);
}

void test_all_neutral_feet_on_floor_at_stance_radius(void) {
    Body body = makeBody();
    for (int i = 0; i < LEG_COUNT; i++) {
        Vec3 foot = neutralFoot(body.legs[i]);
        Vec3 fromMount = vecSub(foot, body.legs[i].mount);
        float horizontal = sqrtf(fromMount.x * fromMount.x + fromMount.y * fromMount.y);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_MM, 0.0f, foot.z, LEGS[i].name);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_MM, 209.0f, horizontal, LEGS[i].name);
    }
}

void test_stance_all_legs_ok_and_same_angles(void) {
    Body body = makeBody();
    Vec3 feet[LEG_COUNT];
    JointAngles angles[LEG_COUNT];
    IkStatus status[LEG_COUNT];
    for (int i = 0; i < LEG_COUNT; i++) {
        feet[i] = neutralFoot(body.legs[i]);
    }
    TEST_ASSERT_TRUE(bodyIK(body, feet, angles, status));
    for (int i = 0; i < LEG_COUNT; i++) {
        TEST_ASSERT_EQUAL_INT_MESSAGE(IK_OK, status[i], LEGS[i].name);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_DEG, 0.0f, angles[i].coxa, LEGS[i].name);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_DEG, 33.1f, angles[i].femur, LEGS[i].name);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_DEG, -83.1f, angles[i].tibia, LEGS[i].name);
    }
}

void test_left_and_right_legs_are_mirrored(void) {
    Body body = makeBody();
    const int RIGHT[3] = { 0, 1, 5 };
    const int LEFT[3] = { 3, 2, 4 };
    Vec3 shift = { 15, -25, 20 };

    for (int k = 0; k < 3; k++) {
        const Leg &right = body.legs[RIGHT[k]];
        const Leg &left = body.legs[LEFT[k]];
        Vec3 rightFoot = vecAdd(neutralFoot(right), shift);
        Vec3 leftFoot = { -rightFoot.x, rightFoot.y, rightFoot.z };

        JointAngles r;
        JointAngles l;
        TEST_ASSERT_EQUAL_INT_MESSAGE(IK_OK, legIK(right, rightFoot, r), right.name);
        TEST_ASSERT_EQUAL_INT_MESSAGE(IK_OK, legIK(left, leftFoot, l), left.name);
        TEST_ASSERT_TRUE_MESSAGE(fabsf(r.coxa) > 1.0f, "coxa must be non-zero for this test");
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_DEG, -r.coxa, l.coxa, left.name);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_DEG, r.femur, l.femur, left.name);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_DEG, r.tibia, l.tibia, left.name);
    }
}

void test_body_ik_reports_the_bad_leg(void) {
    Body body = makeBody();
    Vec3 feet[LEG_COUNT];
    JointAngles angles[LEG_COUNT];
    IkStatus status[LEG_COUNT];
    for (int i = 0; i < LEG_COUNT; i++) {
        feet[i] = neutralFoot(body.legs[i]);
    }
    feet[2] = footOnLegLine(body.legs[2], 1000.0f, 0.0f);
    TEST_ASSERT_FALSE(bodyIK(body, feet, angles, status));
    for (int i = 0; i < LEG_COUNT; i++) {
        IkStatus expected = (i == 2) ? IK_OUT_OF_REACH : IK_OK;
        TEST_ASSERT_EQUAL_INT_MESSAGE(expected, status[i], LEGS[i].name);
    }
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_body_has_legs_A_to_F_in_order);
    RUN_TEST(test_cad_stance_radius_is_209);
    RUN_TEST(test_neutral_foot_of_leg_A_is_cad_standing_foot);
    RUN_TEST(test_all_neutral_feet_on_floor_at_stance_radius);
    RUN_TEST(test_stance_all_legs_ok_and_same_angles);
    RUN_TEST(test_left_and_right_legs_are_mirrored);
    RUN_TEST(test_body_ik_reports_the_bad_leg);
    return UNITY_END();
}
