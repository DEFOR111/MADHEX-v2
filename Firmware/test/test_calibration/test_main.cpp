#include <math.h>
#include <unity.h>
#include "calibration.h"
#include "servo_packet.h"

void setUp(void) {}
void tearDown(void) {}

void test_defaults_trim_0_direction_plus_1(void) {
    Calibration c = makeCalibration();
    for (int leg = 0; leg < LEG_COUNT; leg++) {
        for (int joint = 0; joint < JOINT_COUNT; joint++) {
            TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, c.trimDeg[leg][joint]);
            TEST_ASSERT_EQUAL_INT(1, c.direction[leg][joint]);
        }
    }
    TEST_ASSERT_TRUE(calibrationIsValid(c));
}

void test_zero_pose_default_is_500(void) {
    Calibration c = makeCalibration();
    for (int leg = 0; leg < LEG_COUNT; leg++) {
        for (int joint = 0; joint < JOINT_COUNT; joint++) {
            TEST_ASSERT_EQUAL_INT(500, calibratedServoPosition(c, leg, joint, 0.0f));
        }
    }
}

void test_trim_changes_one_servo(void) {
    Calibration c = makeCalibration();
    int legA = 0;
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 2.0f, calibrationAddTrim(c, legA, JOINT_FEMUR, 2.0f));
    TEST_ASSERT_EQUAL_INT(508, calibratedServoPosition(c, legA, JOINT_FEMUR, 0.0f));
    TEST_ASSERT_EQUAL_INT(500, calibratedServoPosition(c, legA, JOINT_COXA, 0.0f));
    TEST_ASSERT_EQUAL_INT(500, calibratedServoPosition(c, 1, JOINT_FEMUR, 0.0f));

    calibrationAddTrim(c, legA, JOINT_FEMUR, -0.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.5f, c.trimDeg[legA][JOINT_FEMUR]);
}

void test_trim_is_cut_to_limit(void) {
    Calibration c = makeCalibration();
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 30.0f, calibrationAddTrim(c, 2, JOINT_TIBIA, 50.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -30.0f, calibrationAddTrim(c, 2, JOINT_TIBIA, -100.0f));
}

void test_direction_and_trim_formula(void) {
    Calibration c = makeCalibration();
    int legD = 3;
    calibrationAddTrim(c, legD, JOINT_COXA, 3.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 13.0f, calibratedServoAngle(c, legD, JOINT_COXA, 10.0f));

    TEST_ASSERT_EQUAL_INT(-1, calibrationFlipDirection(c, legD, JOINT_COXA));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -13.0f, calibratedServoAngle(c, legD, JOINT_COXA, 10.0f));
    TEST_ASSERT_EQUAL_INT(446, calibratedServoPosition(c, legD, JOINT_COXA, 10.0f));

    TEST_ASSERT_EQUAL_INT(1, calibrationFlipDirection(c, legD, JOINT_COXA));
}

void test_trim_is_in_joint_space(void) {
    Calibration c = makeCalibration();
    int legC = 2;
    calibrationFlipDirection(c, legC, JOINT_FEMUR);
    calibrationAddTrim(c, legC, JOINT_FEMUR, 2.4f);
    int position = calibratedServoPosition(c, legC, JOINT_FEMUR, 0.0f);
    TEST_ASSERT_EQUAL_INT(490, position);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.4f, servoPositionToAngle(position) * c.direction[legC][JOINT_FEMUR]);
}

void test_invalid_data_is_detected(void) {
    Calibration c = makeCalibration();
    c.trimDeg[4][1] = NAN;
    TEST_ASSERT_FALSE(calibrationIsValid(c));

    c = makeCalibration();
    c.trimDeg[0][0] = 31.0f;
    TEST_ASSERT_FALSE(calibrationIsValid(c));

    c = makeCalibration();
    c.direction[5][2] = 0;
    TEST_ASSERT_FALSE(calibrationIsValid(c));
}

void test_parse_leg_and_joint(void) {
    int leg = -1;
    int joint = -1;
    TEST_ASSERT_TRUE(parseLeg("A", leg));
    TEST_ASSERT_EQUAL_INT(0, leg);
    TEST_ASSERT_TRUE(parseLeg("f", leg));
    TEST_ASSERT_EQUAL_INT(5, leg);
    TEST_ASSERT_FALSE(parseLeg("G", leg));
    TEST_ASSERT_FALSE(parseLeg("AB", leg));

    TEST_ASSERT_TRUE(parseJoint("1", joint));
    TEST_ASSERT_EQUAL_INT(JOINT_COXA, joint);
    TEST_ASSERT_TRUE(parseJoint("femur", joint));
    TEST_ASSERT_EQUAL_INT(JOINT_FEMUR, joint);
    TEST_ASSERT_TRUE(parseJoint("3", joint));
    TEST_ASSERT_EQUAL_INT(JOINT_TIBIA, joint);
    TEST_ASSERT_FALSE(parseJoint("0", joint));
    TEST_ASSERT_FALSE(parseJoint("4", joint));
    TEST_ASSERT_EQUAL_STRING("tibia", jointName(JOINT_TIBIA));
}

void test_servo_ids_and_buses(void) {
    bool used[19] = { false };
    for (int leg = 0; leg < LEG_COUNT; leg++) {
        for (int joint = 0; joint < JOINT_COUNT; joint++) {
            int id = SERVO_ID[leg][joint];
            TEST_ASSERT_EQUAL_INT(3 * leg + joint + 1, id);
            TEST_ASSERT_FALSE(used[id]);
            used[id] = true;
        }
    }
    int expectedBus[LEG_COUNT] = { 1, 1, 2, 2, 2, 1 };
    for (int leg = 0; leg < LEG_COUNT; leg++) {
        TEST_ASSERT_EQUAL_INT(expectedBus[leg], SERVO_BUS_OF_LEG[leg]);
    }
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_defaults_trim_0_direction_plus_1);
    RUN_TEST(test_zero_pose_default_is_500);
    RUN_TEST(test_trim_changes_one_servo);
    RUN_TEST(test_trim_is_cut_to_limit);
    RUN_TEST(test_direction_and_trim_formula);
    RUN_TEST(test_trim_is_in_joint_space);
    RUN_TEST(test_invalid_data_is_detected);
    RUN_TEST(test_parse_leg_and_joint);
    RUN_TEST(test_servo_ids_and_buses);
    return UNITY_END();
}
