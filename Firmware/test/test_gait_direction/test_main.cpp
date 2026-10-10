#include <math.h>
#include <stdio.h>
#include <unity.h>
#include "gait.h"

const float TOL_MM = 0.01f;
const float TOL_DEG = 0.01f;
const float DT = 0.02f;
const float MAX_STEP_DEG = 3.0f;

void setUp(void) {}
void tearDown(void) {}

static JointAngles anglesOf(const Leg &leg, Vec3 foot) {
    JointAngles a;
    TEST_ASSERT_EQUAL_INT_MESSAGE(IK_OK, legIK(leg, foot, a), leg.name);
    return a;
}

void test_limit_command(void) {
    GaitCommand c = gaitLimitCommand(makeCommand(2, 0, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, c.vx);
    c = gaitLimitCommand(makeCommand(1, -1, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.7071f, c.vx);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.7071f, c.vy);
    c = gaitLimitCommand(makeCommand(0, -1, 1));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.5f, c.vy);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, c.turn);
    c = gaitLimitCommand(makeCommand(0.3f, -0.4f, 0.2f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.3f, c.vx);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.4f, c.vy);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, c.turn);
    c = gaitLimitCommand(makeCommand(NAN, 0, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, c.vx);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, c.vy);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, c.turn);
}

void test_backward_mirrors_forward(void) {
    Body body = makeBody();
    GaitCommand forward = makeCommand(0, -1, 0);
    GaitCommand backward = makeCommand(0, 1, 0);
    for (int i = 0; i < LEG_COUNT; i++) {
        const Leg &leg = body.legs[i];
        Vec3 n = neutralFoot(leg);
        for (int k = 0; k < 50; k++) {
            float phase = k / 50.0f;
            Vec3 f = gaitFoot(leg, i, GAIT_TRIPOD, phase, forward);
            Vec3 b = gaitFoot(leg, i, GAIT_TRIPOD, phase, backward);
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_MM, f.x, b.x, leg.name);
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_MM, -(f.y - n.y), b.y - n.y, leg.name);
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_MM, f.z, b.z, leg.name);
            if (i == 0 || i == 3) {
                JointAngles af = anglesOf(leg, f);
                JointAngles ab = anglesOf(leg, b);
                TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_DEG, -af.coxa, ab.coxa, leg.name);
                TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_DEG, af.femur, ab.femur, leg.name);
                TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_DEG, af.tibia, ab.tibia, leg.name);
            }
        }
    }
}

void test_left_turn_mirrors_right_turn(void) {
    Body body = makeBody();
    GaitCommand left = makeCommand(0, 0, 1);
    GaitCommand right = makeCommand(0, 0, -1);
    for (int i = 0; i < LEG_COUNT; i++) {
        const Leg &leg = body.legs[i];
        Vec3 n = neutralFoot(leg);
        float radialAngle = atan2f(n.y, n.x);
        for (int k = 0; k < 50; k++) {
            float phase = k / 50.0f;
            Vec3 l = gaitFoot(leg, i, GAIT_TRIPOD, phase, left);
            Vec3 r = gaitFoot(leg, i, GAIT_TRIPOD, phase, right);
            Vec3 lLocal = rotateZ(l, -radialAngle);
            Vec3 rLocal = rotateZ(r, -radialAngle);
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_MM, rLocal.x, lLocal.x, leg.name);
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_MM, -rLocal.y, lLocal.y, leg.name);
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_MM, r.z, l.z, leg.name);
            if (i == 0 || i == 3) {
                JointAngles al = anglesOf(leg, l);
                JointAngles ar = anglesOf(leg, r);
                TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_DEG, -ar.coxa, al.coxa, leg.name);
                TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_DEG, ar.femur, al.femur, leg.name);
                TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_DEG, ar.tibia, al.tibia, leg.name);
            }
        }
    }
}

void test_positive_turn_turns_left(void) {
    Body body = makeBody();
    GaitCommand left = makeCommand(0, 0, 1);
    Vec3 start = gaitFoot(body.legs[3], 3, GAIT_TRIPOD, 0.5f, left);
    Vec3 end = gaitFoot(body.legs[3], 3, GAIT_TRIPOD, 0.99f, left);
    float turned = radToDeg(atan2f(end.y, end.x) - atan2f(start.y, start.x));
    TEST_ASSERT_TRUE(turned < -13.0f);
}

void test_stance_feet_move_rigidly_with_combined_commands(void) {
    Body body = makeBody();
    GaitCommand commands[3] = { makeCommand(0, -0.5f, 0.5f), makeCommand(0.6f, 0, -0.4f),
                                makeCommand(0.4f, 0.4f, 0.2f) };
    for (int c = 0; c < 3; c++) {
        Vec3 start[LEG_COUNT];
        gaitFeet(body, GAIT_TRIPOD, 0.0f, commands[c], start);
        for (int k = 1; k < 50; k++) {
            Vec3 feet[LEG_COUNT];
            gaitFeet(body, GAIT_TRIPOD, k / 100.0f, commands[c], feet);
            int pairs[3][2] = { { 0, 2 }, { 2, 4 }, { 4, 0 } };
            for (int p = 0; p < 3; p++) {
                float d0 = vecLength(vecSub(start[pairs[p][0]], start[pairs[p][1]]));
                float d = vecLength(vecSub(feet[pairs[p][0]], feet[pairs[p][1]]));
                TEST_ASSERT_FLOAT_WITHIN(TOL_MM, d0, d);
            }
        }
    }
}

void test_no_limit_hits_in_any_direction(void) {
    Body body = makeBody();
    GaitCommand commands[40];
    int count = 0;
    for (int a = 0; a < 360; a += 15) {
        float r = degToRad((float)a);
        commands[count++] = makeCommand(cosf(r), sinf(r), 0);
    }
    commands[count++] = makeCommand(0, 0, 1);
    commands[count++] = makeCommand(0, 0, -1);
    commands[count++] = makeCommand(0, -1, 1);
    commands[count++] = makeCommand(0, -1, -1);
    commands[count++] = makeCommand(0, 1, 1);
    commands[count++] = makeCommand(1, 0, -1);
    commands[count++] = makeCommand(-1, 0, 1);
    commands[count++] = makeCommand(0.7f, -0.7f, 0.3f);

    float smallestMargin = 1000.0f;
    for (int c = 0; c < count; c++) {
        for (int k = 0; k < 50; k++) {
            Vec3 feet[LEG_COUNT];
            gaitFeet(body, GAIT_TRIPOD, k / 50.0f, commands[c], feet);
            for (int i = 0; i < LEG_COUNT; i++) {
                JointAngles a;
                TEST_ASSERT_EQUAL_INT_MESSAGE(IK_OK, legIK(body.legs[i], feet[i], a), LEGS[i].name);
                const Leg &leg = body.legs[i];
                float margins[6] = { a.coxa - leg.coxaLimits.minDeg, leg.coxaLimits.maxDeg - a.coxa,
                                     a.femur - leg.femurLimits.minDeg, leg.femurLimits.maxDeg - a.femur,
                                     a.tibia - leg.tibiaLimits.minDeg, leg.tibiaLimits.maxDeg - a.tibia };
                for (int m = 0; m < 6; m++) {
                    if (margins[m] < smallestMargin) {
                        smallestMargin = margins[m];
                    }
                }
            }
        }
    }
    char message[80];
    snprintf(message, sizeof(message), "%d commands, smallest margin to a joint limit %.1f deg",
             count, smallestMargin);
    TEST_MESSAGE(message);
}

static void stepAndCheck(const Body &body, const Gait &gait, JointAngles previous[], float &maxStep) {
    Vec3 feet[LEG_COUNT];
    JointAngles angles[LEG_COUNT];
    IkStatus status[LEG_COUNT];
    gaitFeet(body, gait.type, gait.phase, gait.current, feet);
    TEST_ASSERT_TRUE_MESSAGE(bodyIK(body, feet, angles, status), "IK failed while walking");
    for (int i = 0; i < LEG_COUNT; i++) {
        float biggest = fmaxf(fabsf(angles[i].coxa - previous[i].coxa),
                        fmaxf(fabsf(angles[i].femur - previous[i].femur),
                              fabsf(angles[i].tibia - previous[i].tibia)));
        if (biggest > maxStep) {
            maxStep = biggest;
        }
        TEST_ASSERT_TRUE_MESSAGE(biggest < MAX_STEP_DEG, LEGS[i].name);
        previous[i] = angles[i];
    }
}

void test_change_direction_while_walking_is_smooth(void) {
    Body body = makeBody();
    Vec3 feet[LEG_COUNT];
    JointAngles previous[LEG_COUNT];
    IkStatus status[LEG_COUNT];
    for (int i = 0; i < LEG_COUNT; i++) {
        feet[i] = neutralFoot(body.legs[i]);
    }
    TEST_ASSERT_TRUE(bodyIK(body, feet, previous, status));

    GaitCommand plan[4] = { makeCommand(0, -1, 0), makeCommand(1, 0, -1),
                            makeCommand(0, 1, 1), makeCommand(0, 0, 1) };
    Gait gait = makeGait();
    gait.type = GAIT_TRIPOD;
    float maxStep = 0.0f;
    for (int p = 0; p < 4; p++) {
        gaitWalk(gait, plan[p]);
        for (int k = 0; k < 150; k++) {
            gaitUpdate(gait, DT);
            stepAndCheck(body, gait, previous, maxStep);
        }
    }
    gaitStop(gait);
    int guard = 0;
    while (gait.walking && guard < 500) {
        gaitUpdate(gait, DT);
        stepAndCheck(body, gait, previous, maxStep);
        guard++;
    }
    TEST_ASSERT_FALSE(gait.walking);

    char message[80];
    snprintf(message, sizeof(message), "biggest joint step %.2f deg", maxStep);
    TEST_MESSAGE(message);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_limit_command);
    RUN_TEST(test_backward_mirrors_forward);
    RUN_TEST(test_left_turn_mirrors_right_turn);
    RUN_TEST(test_positive_turn_turns_left);
    RUN_TEST(test_stance_feet_move_rigidly_with_combined_commands);
    RUN_TEST(test_no_limit_hits_in_any_direction);
    RUN_TEST(test_change_direction_while_walking_is_smooth);
    return UNITY_END();
}
