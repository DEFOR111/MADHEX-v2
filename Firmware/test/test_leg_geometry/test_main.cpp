#include <unity.h>
#include "leg.h"

const float TOL = 0.05f;

const float SAME = 0.00001f;

void setUp(void) {}
void tearDown(void) {}

void test_leg_A_lengths(void) {
    Leg a = makeLeg(LEGS[0]);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL, 45.0f, a.coxaLength, "coxa");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL, 128.0f, a.femurLength, "femur");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL, 141.8f, a.tibiaLength, "tibia");
}

void test_leg_A_direction_is_180(void) {
    Leg a = makeLeg(LEGS[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 180.0f, a.directionDeg);
}

void test_leg_A_zero_pose_angles(void) {
    Leg a = makeLeg(LEGS[0]);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL, 0.0f, a.femurZeroDeg, "femur horizontal");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL, -16.4f, a.tibiaZeroDeg, "tibia 16.4 deg down");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL, 163.6f, a.kneeZeroDeg, "knee inner angle");
}

void test_leg_A_mount_is_P1(void) {
    Leg a = makeLeg(LEGS[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -96.0f, a.mount.x);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, a.mount.y);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 60.0f, a.mount.z);
}

void test_leg_B_measured_lengths(void) {
    LegLengths b = measureLegLengths(LEGS[1]);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL, 44.3f, b.coxa, "coxa");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL, 127.0f, b.femur, "femur");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL, 142.4f, b.tibia, "tibia");
}

void test_leg_B_uses_leg_A_lengths_and_zero_angles(void) {
    Leg a = makeLeg(LEGS[0]);
    Leg b = makeLeg(LEGS[1]);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(SAME, a.coxaLength, b.coxaLength, "coxa");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(SAME, a.femurLength, b.femurLength, "femur");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(SAME, a.tibiaLength, b.tibiaLength, "tibia");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(SAME, a.femurZeroDeg, b.femurZeroDeg, "femur zero");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(SAME, a.tibiaZeroDeg, b.tibiaZeroDeg, "tibia zero");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(SAME, a.kneeZeroDeg, b.kneeZeroDeg, "knee zero");
}

void test_leg_B_direction_is_150_1(void) {
    Leg b = makeLeg(LEGS[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 150.1f, b.directionDeg);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -88.0f, b.mount.x);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 81.5f, b.mount.y);
}

void test_all_leg_directions(void) {
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL, 29.9f, makeLeg(LEGS[2]).directionDeg, "C left rear");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL, 0.0f, makeLeg(LEGS[3]).directionDeg, "D left middle");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL, -29.9f, makeLeg(LEGS[4]).directionDeg, "E left front");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL, -150.1f, makeLeg(LEGS[5]).directionDeg, "F right front");
}

void test_length_difference_leg_A_vs_itself_is_0(void) {
    TEST_ASSERT_FLOAT_WITHIN(SAME, 0.0f, maxLengthDifferenceMm(LEGS[0], LEGS[0]));
}

void test_length_difference_leg_B_vs_A(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.987f, maxLengthDifferenceMm(LEGS[1], LEGS[0]));
}

void test_no_leg_exceeds_warning_limit(void) {
    for (int i = 0; i < LEG_COUNT; i++) {
        float diff = maxLengthDifferenceMm(LEGS[i], LEGS[REFERENCE_LEG]);
        TEST_ASSERT_TRUE_MESSAGE(diff <= LENGTH_WARNING_MM, LEGS[i].name);
    }
}

void test_length_difference_detects_bad_leg(void) {
    LegConfig bad = LEGS[0];
    bad.p3.x = bad.p3.x - 2.0f;
    float diff = maxLengthDifferenceMm(bad, LEGS[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.0f, diff);
    TEST_ASSERT_TRUE(diff > LENGTH_WARNING_MM);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_leg_A_lengths);
    RUN_TEST(test_leg_A_direction_is_180);
    RUN_TEST(test_leg_A_zero_pose_angles);
    RUN_TEST(test_leg_A_mount_is_P1);
    RUN_TEST(test_leg_B_measured_lengths);
    RUN_TEST(test_leg_B_uses_leg_A_lengths_and_zero_angles);
    RUN_TEST(test_leg_B_direction_is_150_1);
    RUN_TEST(test_all_leg_directions);
    RUN_TEST(test_length_difference_leg_A_vs_itself_is_0);
    RUN_TEST(test_length_difference_leg_B_vs_A);
    RUN_TEST(test_no_leg_exceeds_warning_limit);
    RUN_TEST(test_length_difference_detects_bad_leg);
    return UNITY_END();
}
