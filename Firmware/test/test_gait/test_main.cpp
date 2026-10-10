#include <math.h>
#include <stdio.h>
#include <unity.h>
#include "gait.h"

const float TOL_MM = 0.01f;
const float DT = 0.02f;
const float MAX_STEP_DEG = 3.0f;
const GaitCommand FORWARD = { 0.0f, -1.0f, 0.0f };

void setUp(void) {}
void tearDown(void) {}

void test_groups_ACE_and_BDF(void) {
    float expected[LEG_COUNT] = { 0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.5f };
    for (int i = 0; i < LEG_COUNT; i++) {
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.0001f, expected[i], gaitLegOffset(GAIT_TRIPOD, i), LEGS[i].name);
    }
}

void test_one_group_always_on_floor(void) {
    Body body = makeBody();
    for (int k = 0; k <= 1000; k++) {
        float phase = k / 1000.0f;
        Vec3 feet[LEG_COUNT];
        gaitFeet(body, GAIT_TRIPOD, phase, FORWARD, feet);
        bool groupACE = fabsf(feet[0].z) < TOL_MM && fabsf(feet[2].z) < TOL_MM && fabsf(feet[4].z) < TOL_MM;
        bool groupBDF = fabsf(feet[1].z) < TOL_MM && fabsf(feet[3].z) < TOL_MM && fabsf(feet[5].z) < TOL_MM;
        TEST_ASSERT_TRUE_MESSAGE(groupACE || groupBDF, "no group on the floor");
    }
}

void test_leg_A_stance_and_swing_points(void) {
    Body body = makeBody();
    const Leg &a = body.legs[0];
    Vec3 n = neutralFoot(a);

    Vec3 stanceStart = gaitFoot(a, 0, GAIT_TRIPOD, 0.0f, FORWARD);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, n.y - 40.0f, stanceStart.y);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, 0.0f, stanceStart.z);

    Vec3 stanceMiddle = gaitFoot(a, 0, GAIT_TRIPOD, 0.25f, FORWARD);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, n.y, stanceMiddle.y);

    Vec3 swingStart = gaitFoot(a, 0, GAIT_TRIPOD, 0.5f, FORWARD);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, n.y + 40.0f, swingStart.y);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, 0.0f, swingStart.z);

    Vec3 swingMiddle = gaitFoot(a, 0, GAIT_TRIPOD, 0.75f, FORWARD);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, n.y, swingMiddle.y);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, 25.0f, swingMiddle.z);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, n.x, swingMiddle.x);
}

void test_stance_feet_move_together_at_constant_speed(void) {
    Body body = makeBody();
    Vec3 start[LEG_COUNT];
    gaitFeet(body, GAIT_TRIPOD, 0.0f, FORWARD, start);
    for (int k = 1; k < 50; k++) {
        float phase = k / 100.0f;
        Vec3 feet[LEG_COUNT];
        gaitFeet(body, GAIT_TRIPOD, phase, FORWARD, feet);
        float expectedMove = 160.0f * phase;
        for (int i = 0; i < LEG_COUNT; i += 2) {
            TEST_ASSERT_FLOAT_WITHIN(TOL_MM, expectedMove, feet[i].y - start[i].y);
            TEST_ASSERT_FLOAT_WITHIN(TOL_MM, start[i].x, feet[i].x);
        }
    }
}

void test_zero_command_stands_still_and_big_values_are_cut(void) {
    Body body = makeBody();
    const Leg &b = body.legs[1];
    Vec3 n = neutralFoot(b);
    GaitCommand zero = { 0.0f, 0.0f, 0.0f };
    GaitCommand tooBig = { 0.0f, -5.0f, 0.0f };
    for (int k = 0; k < 100; k++) {
        float phase = k / 100.0f;
        Vec3 f = gaitFoot(b, 1, GAIT_TRIPOD, phase, zero);
        TEST_ASSERT_FLOAT_WITHIN(TOL_MM, n.x, f.x);
        TEST_ASSERT_FLOAT_WITHIN(TOL_MM, n.y, f.y);
        TEST_ASSERT_FLOAT_WITHIN(TOL_MM, n.z, f.z);

        Vec3 big = gaitFoot(b, 1, GAIT_TRIPOD, phase, tooBig);
        Vec3 normal = gaitFoot(b, 1, GAIT_TRIPOD, phase, FORWARD);
        TEST_ASSERT_FLOAT_WITHIN(TOL_MM, normal.y, big.y);
    }
}

static void stepAndCheck(const Body &body, const Gait &gait, JointAngles previous[], float &maxStep) {
    Vec3 feet[LEG_COUNT];
    JointAngles angles[LEG_COUNT];
    IkStatus status[LEG_COUNT];
    gaitFeet(body, gait.type, gait.phase, gait.current, feet);
    TEST_ASSERT_TRUE_MESSAGE(bodyIK(body, feet, angles, status), "IK failed while walking");
    for (int i = 0; i < LEG_COUNT; i++) {
        float d1 = fabsf(angles[i].coxa - previous[i].coxa);
        float d2 = fabsf(angles[i].femur - previous[i].femur);
        float d3 = fabsf(angles[i].tibia - previous[i].tibia);
        float biggest = fmaxf(d1, fmaxf(d2, d3));
        if (biggest > maxStep) {
            maxStep = biggest;
        }
        TEST_ASSERT_TRUE_MESSAGE(biggest < MAX_STEP_DEG, LEGS[i].name);
        previous[i] = angles[i];
    }
}

void test_full_walk_start_to_stop(void) {
    Body body = makeBody();

    Vec3 feet[LEG_COUNT];
    JointAngles previous[LEG_COUNT];
    IkStatus status[LEG_COUNT];
    for (int i = 0; i < LEG_COUNT; i++) {
        feet[i] = neutralFoot(body.legs[i]);
    }
    TEST_ASSERT_TRUE(bodyIK(body, feet, previous, status));

    Gait gait = makeGait();
    gait.type = GAIT_TRIPOD;
    gaitWalk(gait, FORWARD);
    float maxStep = 0.0f;
    int updates = 0;

    for (int k = 0; k < 200; k++) {
        gaitUpdate(gait, DT);
        updates++;
        stepAndCheck(body, gait, previous, maxStep);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, gait.current.vy);

    gaitStop(gait);
    int stopUpdates = 0;
    while (gait.walking && stopUpdates < 500) {
        gaitUpdate(gait, DT);
        stopUpdates++;
        stepAndCheck(body, gait, previous, maxStep);
    }
    TEST_ASSERT_FALSE_MESSAGE(gait.walking, "gait did not stop");
    TEST_ASSERT_TRUE(stopUpdates >= 50 && stopUpdates <= 51);

    gaitFeet(body, gait.type, gait.phase, gait.current, feet);
    for (int i = 0; i < LEG_COUNT; i++) {
        Vec3 n = neutralFoot(body.legs[i]);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_MM, n.x, feet[i].x, LEGS[i].name);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_MM, n.y, feet[i].y, LEGS[i].name);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_MM, n.z, feet[i].z, LEGS[i].name);
    }

    char message[96];
    snprintf(message, sizeof(message), "%d walk updates + %d stop updates, biggest joint step %.2f deg",
             updates, stopUpdates, maxStep);
    TEST_MESSAGE(message);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_groups_ACE_and_BDF);
    RUN_TEST(test_one_group_always_on_floor);
    RUN_TEST(test_leg_A_stance_and_swing_points);
    RUN_TEST(test_stance_feet_move_together_at_constant_speed);
    RUN_TEST(test_zero_command_stands_still_and_big_values_are_cut);
    RUN_TEST(test_full_walk_start_to_stop);
    return UNITY_END();
}
