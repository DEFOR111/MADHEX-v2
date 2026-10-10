#include <unity.h>
#include <math.h>
#include "config.h"
#include "imu.h"

void setUp(void) {}
void tearDown(void) {}

void test_calibration_status_bits(void) {
    ImuCalibration c = imuCalibrationFromByte(0xFF);
    TEST_ASSERT_EQUAL_INT(3, c.sys);
    TEST_ASSERT_EQUAL_INT(3, c.gyro);
    TEST_ASSERT_EQUAL_INT(3, c.accel);
    TEST_ASSERT_EQUAL_INT(3, c.mag);

    c = imuCalibrationFromByte(0xE4);
    TEST_ASSERT_EQUAL_INT(3, c.sys);
    TEST_ASSERT_EQUAL_INT(2, c.gyro);
    TEST_ASSERT_EQUAL_INT(1, c.accel);
    TEST_ASSERT_EQUAL_INT(0, c.mag);
}

void test_ready_to_save_needs_gyro_accel_mag(void) {
    TEST_ASSERT_TRUE(imuCalibrationReadyToSave(imuCalibrationFromByte(0x3F)));
    TEST_ASSERT_FALSE(imuCalibrationReadyToSave(imuCalibrationFromByte(0xFB)));
    TEST_ASSERT_FALSE(imuCalibrationReadyToSave(imuCalibrationFromByte(0xFE)));
}

void test_raw_values(void) {
    TEST_ASSERT_EQUAL_INT16(16, imuRaw16(0x10, 0x00));
    TEST_ASSERT_EQUAL_INT16(-16, imuRaw16(0xF0, 0xFF));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 90.0f, imuHeadingFromRaw(1440));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 359.9375f, imuHeadingFromRaw(5759));
    Vec3 g = imuGravityFromRaw(0, -12, 981);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, g.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.12f, g.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 9.81f, g.z);
}

void test_axis_map_validity(void) {
    int same[3] = { 1, 2, 3 };
    int turned90[3] = { 2, -1, 3 };
    int turned180[3] = { -1, -2, 3 };
    int upsideDown[3] = { -1, 2, -3 };
    int mirror[3] = { 1, 2, -3 };
    int twice[3] = { 1, 1, 3 };
    int standing[3] = { 1, 3, -2 };
    int zero[3] = { 0, 2, 3 };
    TEST_ASSERT_TRUE(imuAxisMapIsValid(same));
    TEST_ASSERT_TRUE(imuAxisMapIsValid(turned90));
    TEST_ASSERT_TRUE(imuAxisMapIsValid(turned180));
    TEST_ASSERT_TRUE(imuAxisMapIsValid(upsideDown));
    TEST_ASSERT_FALSE(imuAxisMapIsValid(mirror));
    TEST_ASSERT_FALSE(imuAxisMapIsValid(twice));
    TEST_ASSERT_FALSE(imuAxisMapIsValid(standing));
    TEST_ASSERT_FALSE(imuAxisMapIsValid(zero));
}

void test_config_axis_map_is_valid(void) {
    TEST_ASSERT_TRUE(imuAxisMapIsValid(IMU_AXIS_MAP));
}

void test_sensor_to_robot(void) {
    int turned90[3] = { 2, -1, 3 };
    Vec3 s = { 1.0f, 2.0f, 3.0f };
    Vec3 r = imuSensorToRobot(s, turned90);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, r.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, r.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.0f, r.z);
}

static Vec3 gravityPitched(float pitchDeg) {
    Vec3 g = { 0.0f, -9.81f * sinf(degToRad(pitchDeg)), 9.81f * cosf(degToRad(pitchDeg)) };
    return g;
}
static Vec3 gravityRolled(float rollDeg) {
    Vec3 g = { 9.81f * sinf(degToRad(rollDeg)), 0.0f, 9.81f * cosf(degToRad(rollDeg)) };
    return g;
}

void test_tilt_flat(void) {
    float roll, pitch;
    Vec3 flat = { 0.0f, 0.0f, 9.81f };
    TEST_ASSERT_TRUE(imuTilt(flat, roll, pitch));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, roll);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, pitch);
}

void test_tilt_front_up_and_left_up(void) {
    float roll, pitch;
    TEST_ASSERT_TRUE(imuTilt(gravityPitched(10.0f), roll, pitch));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, pitch);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, roll);
    TEST_ASSERT_TRUE(imuTilt(gravityPitched(-25.0f), roll, pitch));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -25.0f, pitch);

    TEST_ASSERT_TRUE(imuTilt(gravityRolled(15.0f), roll, pitch));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 15.0f, roll);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, pitch);
    TEST_ASSERT_TRUE(imuTilt(gravityRolled(-30.0f), roll, pitch));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -30.0f, roll);
}

void test_tilt_ignores_length(void) {
    float roll, pitch;
    Vec3 g = vecScale(gravityPitched(12.0f), 0.8f);
    TEST_ASSERT_TRUE(imuTilt(g, roll, pitch));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.0f, pitch);
}

void test_tilt_rejects_missing_data(void) {
    float roll = 99.0f, pitch = 99.0f;
    Vec3 nothing = { 0.0f, 0.0f, 0.0f };
    TEST_ASSERT_FALSE(imuTilt(nothing, roll, pitch));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, roll);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pitch);
}

void test_tilt_through_mapping(void) {
    int turned90[3] = { 2, -1, 3 };
    Vec3 robotG = gravityPitched(10.0f);
    Vec3 sensorG = { -robotG.y, robotG.x, robotG.z };
    float roll, pitch;
    TEST_ASSERT_TRUE(imuTilt(imuSensorToRobot(sensorG, turned90), roll, pitch));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, pitch);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, roll);
}

void test_heading(void) {
    int same[3] = { 1, 2, 3 };
    int upsideDown[3] = { -1, 2, -3 };
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, imuRobotHeading(90.0f, same, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, imuRobotHeading(350.0f, same, 20.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 340.0f, imuRobotHeading(10.0f, same, -30.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 270.0f, imuRobotHeading(90.0f, upsideDown, 0.0f));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_calibration_status_bits);
    RUN_TEST(test_ready_to_save_needs_gyro_accel_mag);
    RUN_TEST(test_raw_values);
    RUN_TEST(test_axis_map_validity);
    RUN_TEST(test_config_axis_map_is_valid);
    RUN_TEST(test_sensor_to_robot);
    RUN_TEST(test_tilt_flat);
    RUN_TEST(test_tilt_front_up_and_left_up);
    RUN_TEST(test_tilt_ignores_length);
    RUN_TEST(test_tilt_rejects_missing_data);
    RUN_TEST(test_tilt_through_mapping);
    RUN_TEST(test_heading);
    return UNITY_END();
}
