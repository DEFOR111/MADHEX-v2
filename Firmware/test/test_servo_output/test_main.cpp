#include <unity.h>
#include <stdlib.h>
#include "config.h"
#include "body.h"
#include "controller.h"
#include "servo_output.h"

void setUp(void) {}
void tearDown(void) {}

void test_target_positions_use_calibration(void) {
    Calibration calib = makeCalibration();
    calib.trimDeg[0][JOINT_FEMUR] = 2.4f;
    calib.direction[1][JOINT_COXA] = -1;
    JointAngles angles[LEG_COUNT];
    for (int i = 0; i < LEG_COUNT; i++) {
        angles[i].coxa = 0.0f;
        angles[i].femur = 0.0f;
        angles[i].tibia = 0.0f;
    }
    angles[1].coxa = 12.0f;

    int positions[SERVO_COUNT];
    servoTargetPositions(calib, angles, positions);
    TEST_ASSERT_EQUAL_INT(500, positions[0]);
    TEST_ASSERT_EQUAL_INT(510, positions[1]);
    TEST_ASSERT_EQUAL_INT(450, positions[3]);
    TEST_ASSERT_EQUAL_INT(500, positions[17]);
}

void test_sitting_pose_positions(void) {
    Body body = makeBody();
    Controller ctrl = makeController();
    Vec3 feet[LEG_COUNT];
    JointAngles angles[LEG_COUNT];
    IkStatus status[LEG_COUNT];
    controllerFeet(ctrl, body, feet);
    TEST_ASSERT_TRUE(bodyIK(body, feet, angles, status));

    int positions[SERVO_COUNT];
    servoTargetPositions(makeCalibration(), angles, positions);
    for (int i = 0; i < SERVO_COUNT; i++) {
        TEST_ASSERT_TRUE(positions[i] >= SERVO_POSITION_MIN && positions[i] <= SERVO_POSITION_MAX);
    }
    TEST_ASSERT_EQUAL_INT(positions[3 * 0 + JOINT_FEMUR], positions[3 * 3 + JOINT_FEMUR]);
    TEST_ASSERT_EQUAL_INT(positions[3 * 0 + JOINT_TIBIA], positions[3 * 3 + JOINT_TIBIA]);
}

static void checkMovePacket(const uint8_t p[], int id, int position, int timeMs) {
    TEST_ASSERT_EQUAL_HEX8(0x55, p[0]);
    TEST_ASSERT_EQUAL_HEX8(0x55, p[1]);
    TEST_ASSERT_EQUAL_UINT8(id, p[2]);
    TEST_ASSERT_EQUAL_UINT8(7, p[3]);
    TEST_ASSERT_EQUAL_UINT8(SERVO_MOVE_TIME_WRITE, p[4]);
    TEST_ASSERT_EQUAL_INT(position, p[5] | (p[6] << 8));
    TEST_ASSERT_EQUAL_INT(timeMs, p[7] | (p[8] << 8));
    TEST_ASSERT_EQUAL_HEX8(servoChecksum(p), p[9]);
}

static void fillTestPositions(int positions[]) {
    for (int i = 0; i < SERVO_COUNT; i++) {
        positions[i] = 100 + 10 * i;
    }
}

void test_frame_bus1_legs_a_b_f(void) {
    int positions[SERVO_COUNT];
    fillTestPositions(positions);
    uint8_t frame[SERVO_FRAME_MAX_BYTES];
    TEST_ASSERT_EQUAL_INT(90, servoBuildFrame(1, positions, 20, frame));
    int servo[9] = { 0, 1, 2, 3, 4, 5, 15, 16, 17 };
    for (int k = 0; k < 9; k++) {
        checkMovePacket(frame + 10 * k, servo[k] + 1, positions[servo[k]], 20);
    }
}

void test_frame_bus2_legs_c_d_e(void) {
    int positions[SERVO_COUNT];
    fillTestPositions(positions);
    uint8_t frame[SERVO_FRAME_MAX_BYTES];
    TEST_ASSERT_EQUAL_INT(90, servoBuildFrame(2, positions, 20, frame));
    for (int k = 0; k < 9; k++) {
        checkMovePacket(frame + 10 * k, 7 + k, positions[6 + k], 20);
    }
}

void test_wire_time_fits_in_update_period(void) {
    TEST_ASSERT_EQUAL_UINT32(7812, servoWireTimeUs(90, SERVO_BAUD));
    TEST_ASSERT_EQUAL_UINT32(15625, servoWireTimeUs(180, SERVO_BAUD));
    TEST_ASSERT_TRUE(servoWireTimeUs(90, SERVO_BAUD) < UPDATE_PERIOD_MS * 1000UL);
}

void test_bus_window_between_frames(void) {
    TEST_ASSERT_FALSE(servoBusWindowFree(3, 8, 10, 20));
    TEST_ASSERT_FALSE(servoBusWindowFree(8, 8, 10, 20));
    TEST_ASSERT_TRUE(servoBusWindowFree(9, 8, 10, 20));
    TEST_ASSERT_FALSE(servoBusWindowFree(10, 8, 10, 20));
    TEST_ASSERT_FALSE(servoBusWindowFree(25, 8, 10, 20));
}

void test_startup_move_ends(void) {
    int from[SERVO_COUNT];
    int to[SERVO_COUNT];
    int out[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
        from[i] = 800;
        to[i] = 200;
    }
    servoStartupPositions(from, to, -1.0f, 2.5f, out);
    TEST_ASSERT_EQUAL_INT(800, out[0]);
    servoStartupPositions(from, to, 0.0f, 2.5f, out);
    TEST_ASSERT_EQUAL_INT(800, out[0]);
    servoStartupPositions(from, to, 1.25f, 2.5f, out);
    TEST_ASSERT_EQUAL_INT(500, out[5]);
    servoStartupPositions(from, to, 2.5f, 2.5f, out);
    TEST_ASSERT_EQUAL_INT(200, out[17]);
    servoStartupPositions(from, to, 9.0f, 2.5f, out);
    TEST_ASSERT_EQUAL_INT(200, out[17]);
}

void test_startup_move_is_slow_and_smooth(void) {
    int from[SERVO_COUNT];
    int to[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
        from[i] = 0;
        to[i] = 1000;
    }
    int before[SERVO_COUNT];
    int now[SERVO_COUNT];
    servoStartupPositions(from, to, 0.0f, SERVO_STARTUP_MOVE_S, before);
    int biggestStep = 0;
    int frames = (int)(SERVO_STARTUP_MOVE_S * 1000.0f / UPDATE_PERIOD_MS);
    for (int k = 1; k <= frames; k++) {
        servoStartupPositions(from, to, k * UPDATE_PERIOD_MS / 1000.0f, SERVO_STARTUP_MOVE_S, now);
        int step = now[0] - before[0];
        TEST_ASSERT_TRUE(step >= 0);
        if (step > biggestStep) {
            biggestStep = step;
        }
        before[0] = now[0];
    }
    TEST_ASSERT_EQUAL_INT(1000, before[0]);
    TEST_ASSERT_TRUE(biggestStep <= 13);
}

void test_servos_on_refusals(void) {
    TEST_ASSERT_NULL(servoOnRefusal(SERVO_OUTPUT_OFF, STATE_SIT, SAFETY_FAULT_NONE, false, true, BATTERY_OK));
    TEST_ASSERT_NULL(servoOnRefusal(SERVO_OUTPUT_OFF, STATE_SIT, SAFETY_FAULT_NONE, false, true, BATTERY_LOW));

    TEST_ASSERT_EQUAL_STRING("battery not connected (servo power is 0 V)",
        servoOnRefusal(SERVO_OUTPUT_OFF, STATE_SIT, SAFETY_FAULT_NONE, false, true, BATTERY_NOT_CONNECTED));
    TEST_ASSERT_EQUAL_STRING("no battery reading yet, try again",
        servoOnRefusal(SERVO_OUTPUT_OFF, STATE_SIT, SAFETY_FAULT_NONE, false, false, BATTERY_NOT_CONNECTED));
    TEST_ASSERT_EQUAL_STRING("only while sitting (the start pose is the sitting pose)",
        servoOnRefusal(SERVO_OUTPUT_OFF, STATE_STAND, SAFETY_FAULT_NONE, false, true, BATTERY_OK));
    TEST_ASSERT_EQUAL_STRING("only while sitting (the start pose is the sitting pose)",
        servoOnRefusal(SERVO_OUTPUT_OFF, STATE_ESTOP, SAFETY_FAULT_NONE, false, true, BATTERY_OK));
    TEST_ASSERT_EQUAL_STRING("safety fault, fix the cause and type 'safety clear' first",
        servoOnRefusal(SERVO_OUTPUT_OFF, STATE_SIT, SAFETY_FAULT_OVERHEAT, false, true, BATTERY_OK));
    TEST_ASSERT_EQUAL_STRING("servos do not answer the safety checks (no servo power?)",
        servoOnRefusal(SERVO_OUTPUT_OFF, STATE_SIT, SAFETY_FAULT_NONE, true, true, BATTERY_OK));
    TEST_ASSERT_EQUAL_STRING("servos are starting already",
        servoOnRefusal(SERVO_OUTPUT_STARTING, STATE_SIT, SAFETY_FAULT_NONE, false, true, BATTERY_OK));
    TEST_ASSERT_EQUAL_STRING("servos are on already",
        servoOnRefusal(SERVO_OUTPUT_ON, STATE_SIT, SAFETY_FAULT_NONE, false, true, BATTERY_OK));
}

void test_force_off_reasons(void) {
    TEST_ASSERT_NULL(servoForceOffReason(STATE_WALK, SAFETY_FAULT_NONE, false, true, BATTERY_OK));
    TEST_ASSERT_NULL(servoForceOffReason(STATE_WALK, SAFETY_FAULT_NONE, false, true, BATTERY_CRITICAL));

    TEST_ASSERT_NULL(servoForceOffReason(STATE_WALK, SAFETY_FAULT_LOW_VOLTAGE, false, true, BATTERY_LOW));
    TEST_ASSERT_NULL(servoForceOffReason(STATE_SITTING_DOWN, SAFETY_FAULT_OVERHEAT, false, true, BATTERY_OK));
    TEST_ASSERT_EQUAL_STRING("safety fault, robot is sitting",
        servoForceOffReason(STATE_SIT, SAFETY_FAULT_OVERHEAT, false, true, BATTERY_OK));

    TEST_ASSERT_EQUAL_STRING("battery not connected",
        servoForceOffReason(STATE_WALK, SAFETY_FAULT_NONE, false, true, BATTERY_NOT_CONNECTED));
    TEST_ASSERT_EQUAL_STRING("servos do not answer",
        servoForceOffReason(STATE_STAND, SAFETY_FAULT_NONE, true, true, BATTERY_OK));
    TEST_ASSERT_NULL(servoForceOffReason(STATE_STAND, SAFETY_FAULT_NONE, false, false, BATTERY_NOT_CONNECTED));

    TEST_ASSERT_EQUAL_STRING("emergency stop",
        servoForceOffReason(STATE_ESTOP, SAFETY_FAULT_NONE, false, true, BATTERY_OK));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_target_positions_use_calibration);
    RUN_TEST(test_sitting_pose_positions);
    RUN_TEST(test_frame_bus1_legs_a_b_f);
    RUN_TEST(test_frame_bus2_legs_c_d_e);
    RUN_TEST(test_wire_time_fits_in_update_period);
    RUN_TEST(test_bus_window_between_frames);
    RUN_TEST(test_startup_move_ends);
    RUN_TEST(test_startup_move_is_slow_and_smooth);
    RUN_TEST(test_servos_on_refusals);
    RUN_TEST(test_force_off_reasons);
    return UNITY_END();
}
