#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>
#include "controller.h"

const float DT = 0.02f;
const float TOL_MM = 0.01f;
const float MAX_STEP_DEG = 3.0f;
const GaitCommand FORWARD = { 0.0f, -1.0f, 0.0f };

void setUp(void) {}
void tearDown(void) {}

static int runUntil(Controller &c, RobotState wanted) {
    int count = 0;
    while (c.state != wanted && count < 1000) {
        controllerUpdate(c, DT);
        count++;
    }
    TEST_ASSERT_EQUAL_STRING(stateName(wanted), stateName(c.state));
    return count;
}

static Controller standingController() {
    Controller c = makeController();
    controllerUp(c);
    runUntil(c, STATE_STAND);
    return c;
}

static Controller walkingController() {
    Controller c = standingController();
    controllerWalk(c, FORWARD);
    for (int k = 0; k < 100; k++) {
        controllerUpdate(c, DT);
    }
    return c;
}

void test_starts_sitting_on_the_floor(void) {
    Controller c = makeController();
    TEST_ASSERT_EQUAL_STRING("SIT", stateName(c.state));
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, BODY_UNDERSIDE_Z, c.floorZ);
    TEST_ASSERT_FALSE(controllerIsMoving(c));
}

void test_sit_rejects_walk_down_stop(void) {
    Controller c = makeController();
    ControlReply r = controllerWalk(c, FORWARD);
    TEST_ASSERT_FALSE(r.accepted);
    TEST_ASSERT_NOT_NULL(strstr(r.message, "(up)"));
    TEST_ASSERT_FALSE(controllerDown(c).accepted);
    TEST_ASSERT_FALSE(controllerStop(c).accepted);
    TEST_ASSERT_EQUAL_STRING("SIT", stateName(c.state));
}

void test_sit_up_starts_standing_up(void) {
    Controller c = makeController();
    TEST_ASSERT_TRUE(controllerUp(c).accepted);
    TEST_ASSERT_EQUAL_STRING("STANDING_UP", stateName(c.state));
    TEST_ASSERT_TRUE(controllerIsMoving(c));
}

void test_standing_up_rejects_everything_and_ends_in_stand(void) {
    Controller c = makeController();
    controllerUp(c);
    controllerUpdate(c, DT);
    TEST_ASSERT_FALSE(controllerUp(c).accepted);
    TEST_ASSERT_FALSE(controllerDown(c).accepted);
    TEST_ASSERT_FALSE(controllerWalk(c, FORWARD).accepted);
    TEST_ASSERT_FALSE(controllerStop(c).accepted);
    TEST_ASSERT_EQUAL_STRING("STANDING_UP", stateName(c.state));

    int updates = runUntil(c, STATE_STAND) + 1;
    TEST_ASSERT_TRUE(updates >= 74 && updates <= 76);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, 0.0f, c.floorZ);
}

void test_stand_up_is_rejected_stop_is_rejected(void) {
    Controller c = standingController();
    TEST_ASSERT_FALSE(controllerUp(c).accepted);
    TEST_ASSERT_FALSE(controllerStop(c).accepted);
    TEST_ASSERT_EQUAL_STRING("STAND", stateName(c.state));
}

void test_stand_walk_starts_walking(void) {
    Controller c = standingController();
    TEST_ASSERT_TRUE(controllerWalk(c, FORWARD).accepted);
    TEST_ASSERT_EQUAL_STRING("WALK", stateName(c.state));
    TEST_ASSERT_TRUE(c.gait.walking);
}

void test_stand_down_sits_down(void) {
    Controller c = standingController();
    TEST_ASSERT_TRUE(controllerDown(c).accepted);
    TEST_ASSERT_EQUAL_STRING("SITTING_DOWN", stateName(c.state));
}

void test_walk_accepts_new_direction_rejects_up(void) {
    Controller c = walkingController();
    ControlReply r = controllerWalk(c, makeCommand(1, 0, 0));
    TEST_ASSERT_TRUE(r.accepted);
    TEST_ASSERT_EQUAL_STRING("walk: new direction", r.message);
    TEST_ASSERT_FALSE(controllerUp(c).accepted);
    TEST_ASSERT_EQUAL_STRING("WALK", stateName(c.state));
}

void test_walk_stop_slows_down_then_stand(void) {
    Controller c = walkingController();
    TEST_ASSERT_TRUE(controllerStop(c).accepted);
    TEST_ASSERT_FALSE(controllerStop(c).accepted);
    TEST_ASSERT_EQUAL_STRING("WALK", stateName(c.state));
    runUntil(c, STATE_STAND);
    TEST_ASSERT_FALSE(c.gait.walking);
}

void test_down_while_walking_stops_then_sits(void) {
    Controller c = walkingController();
    ControlReply r = controllerDown(c);
    TEST_ASSERT_TRUE(r.accepted);
    TEST_ASSERT_EQUAL_STRING("down: stopping first, then sitting down", r.message);
    TEST_ASSERT_FALSE(controllerWalk(c, FORWARD).accepted);
    TEST_ASSERT_FALSE(controllerDown(c).accepted);

    RobotState order[3] = { STATE_WALK, STATE_SITTING_DOWN, STATE_SIT };
    int seen = 0;
    for (int k = 0; k < 1000 && c.state != STATE_SIT; k++) {
        controllerUpdate(c, DT);
        if (seen < 2 && c.state == order[seen + 1]) {
            seen++;
        }
        TEST_ASSERT_TRUE(c.state == order[seen]);
    }
    TEST_ASSERT_EQUAL_STRING("SIT", stateName(c.state));
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, BODY_UNDERSIDE_Z, c.floorZ);
    TEST_ASSERT_FALSE(c.sitAfterStop);
}

void test_sitting_down_rejects_everything_and_ends_in_sit(void) {
    Controller c = standingController();
    controllerDown(c);
    controllerUpdate(c, DT);
    TEST_ASSERT_FALSE(controllerUp(c).accepted);
    TEST_ASSERT_FALSE(controllerDown(c).accepted);
    TEST_ASSERT_FALSE(controllerWalk(c, FORWARD).accepted);
    TEST_ASSERT_FALSE(controllerStop(c).accepted);
    runUntil(c, STATE_SIT);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, BODY_UNDERSIDE_Z, c.floorZ);
}

void test_feet_height_follows_the_body(void) {
    Body body = makeBody();
    Controller c = makeController();
    Vec3 feet[LEG_COUNT];
    controllerFeet(c, body, feet);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, 30.0f, feet[0].z);

    controllerUp(c);
    float previousZ = feet[0].z;
    while (c.state == STATE_STANDING_UP) {
        controllerUpdate(c, DT);
        controllerFeet(c, body, feet);
        TEST_ASSERT_TRUE(feet[0].z <= previousZ + 0.0001f);
        previousZ = feet[0].z;
    }
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, 0.0f, feet[0].z);
}

void test_whole_story_is_smooth(void) {
    Body body = makeBody();
    Controller c = makeController();
    Vec3 feet[LEG_COUNT];
    JointAngles last[LEG_COUNT];
    IkStatus status[LEG_COUNT];
    controllerFeet(c, body, feet);
    TEST_ASSERT_TRUE(bodyIK(body, feet, last, status));

    float maxStep = 0.0f;
    for (int k = 0; k < 600; k++) {
        if (k == 0) controllerUp(c);
        if (k == 100) controllerWalk(c, FORWARD);
        if (k == 250) controllerWalk(c, makeCommand(1, 0, 0));
        if (k == 350) controllerDown(c);
        controllerUpdate(c, DT);

        JointAngles angles[LEG_COUNT];
        controllerFeet(c, body, feet);
        TEST_ASSERT_TRUE_MESSAGE(bodyIK(body, feet, angles, status), stateName(c.state));
        for (int i = 0; i < LEG_COUNT; i++) {
            float jump = fmaxf(fabsf(angles[i].coxa - last[i].coxa),
                         fmaxf(fabsf(angles[i].femur - last[i].femur),
                               fabsf(angles[i].tibia - last[i].tibia)));
            if (jump > maxStep) {
                maxStep = jump;
            }
            TEST_ASSERT_TRUE_MESSAGE(jump < MAX_STEP_DEG, stateName(c.state));
            last[i] = angles[i];
        }
    }
    TEST_ASSERT_EQUAL_STRING("SIT", stateName(c.state));

    char message[64];
    snprintf(message, sizeof(message), "biggest joint step %.2f deg", maxStep);
    TEST_MESSAGE(message);
}

void test_height_is_cut_to_the_limits(void) {
    Controller c = makeController();
    TEST_ASSERT_TRUE(controllerSetHeight(c, 15.0f).accepted);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, 15.0f, controllerHeightMm(c));
    controllerSetHeight(c, 500.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, STAND_HEIGHT_MAX_MM, controllerHeightMm(c));
    controllerSetHeight(c, -500.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, STAND_HEIGHT_MIN_MM, controllerHeightMm(c));
    TEST_ASSERT_FALSE(controllerSetHeight(c, NAN).accepted);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, STAND_HEIGHT_MIN_MM, controllerHeightMm(c));
}

void test_up_goes_to_the_chosen_height(void) {
    Controller c = makeController();
    controllerSetHeight(c, 20.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, 30.0f, c.floorZ);
    controllerUp(c);
    runUntil(c, STATE_STAND);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, -20.0f, c.floorZ);
}

void test_height_change_while_standing_is_smooth(void) {
    Controller c = standingController();
    controllerSetHeight(c, 10.0f);
    float before = c.floorZ;
    controllerUpdate(c, DT);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, STAND_HEIGHT_RATE_MM_S * DT, before - c.floorZ);
    for (int k = 0; k < 100; k++) {
        controllerUpdate(c, DT);
    }
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, -10.0f, c.floorZ);
    TEST_ASSERT_TRUE(controllerSetHeight(c, 0.0f).accepted);
}

void test_walking_feet_follow_the_height(void) {
    Body body = makeBody();
    Controller c = makeController();
    controllerSetHeight(c, 25.0f);
    controllerUp(c);
    runUntil(c, STATE_STAND);
    controllerWalk(c, FORWARD);
    Vec3 feet[LEG_COUNT];
    float lowest = 1000.0f;
    for (int k = 0; k < 200; k++) {
        controllerUpdate(c, DT);
        controllerFeet(c, body, feet);
        for (int i = 0; i < LEG_COUNT; i++) {
            lowest = fminf(lowest, feet[i].z);
        }
    }
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, -25.0f, lowest);
}

static void walkSidewaysAtHeight(float heightMm) {
    Body body = makeBody();
    Controller c = makeController();
    controllerSetHeight(c, heightMm);
    controllerUp(c);
    runUntil(c, STATE_STAND);
    Vec3 feet[LEG_COUNT];
    JointAngles last[LEG_COUNT];
    JointAngles angles[LEG_COUNT];
    IkStatus status[LEG_COUNT];
    controllerFeet(c, body, feet);
    TEST_ASSERT_TRUE(bodyIK(body, feet, last, status));
    controllerWalk(c, makeCommand(1, 0, 0));
    for (int k = 0; k < 300; k++) {
        controllerUpdate(c, DT);
        controllerFeet(c, body, feet);
        TEST_ASSERT_TRUE(bodyIK(body, feet, angles, status));
        for (int i = 0; i < LEG_COUNT; i++) {
            float jump = fmaxf(fabsf(angles[i].coxa - last[i].coxa),
                         fmaxf(fabsf(angles[i].femur - last[i].femur), fabsf(angles[i].tibia - last[i].tibia)));
            TEST_ASSERT_TRUE(jump < MAX_STEP_DEG);
            last[i] = angles[i];
        }
    }
}

void test_height_limits_walk_smoothly(void) {
    walkSidewaysAtHeight(STAND_HEIGHT_MIN_MM);
    walkSidewaysAtHeight(STAND_HEIGHT_MAX_MM);
}

void test_height_refused_in_estop(void) {
    Controller c = standingController();
    controllerEmergencyStop(c);
    TEST_ASSERT_FALSE(controllerSetHeight(c, 10.0f).accepted);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_starts_sitting_on_the_floor);
    RUN_TEST(test_sit_rejects_walk_down_stop);
    RUN_TEST(test_sit_up_starts_standing_up);
    RUN_TEST(test_standing_up_rejects_everything_and_ends_in_stand);
    RUN_TEST(test_stand_up_is_rejected_stop_is_rejected);
    RUN_TEST(test_stand_walk_starts_walking);
    RUN_TEST(test_stand_down_sits_down);
    RUN_TEST(test_walk_accepts_new_direction_rejects_up);
    RUN_TEST(test_walk_stop_slows_down_then_stand);
    RUN_TEST(test_down_while_walking_stops_then_sits);
    RUN_TEST(test_sitting_down_rejects_everything_and_ends_in_sit);
    RUN_TEST(test_feet_height_follows_the_body);
    RUN_TEST(test_whole_story_is_smooth);
    RUN_TEST(test_height_is_cut_to_the_limits);
    RUN_TEST(test_up_goes_to_the_chosen_height);
    RUN_TEST(test_height_change_while_standing_is_smooth);
    RUN_TEST(test_walking_feet_follow_the_height);
    RUN_TEST(test_height_limits_walk_smoothly);
    RUN_TEST(test_height_refused_in_estop);
    return UNITY_END();
}
