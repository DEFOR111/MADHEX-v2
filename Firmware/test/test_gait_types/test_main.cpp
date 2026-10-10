#include <math.h>
#include <stdio.h>
#include <unity.h>
#include "controller.h"

const float DT = 0.02f;
const float MAX_STEP_DEG = 3.0f;
const GaitType ALL_GAITS[3] = { GAIT_TRIPOD, GAIT_RIPPLE, GAIT_WAVE };

void setUp(void) {}
void tearDown(void) {}

void test_names_and_swing_fractions(void) {
    TEST_ASSERT_EQUAL_STRING("tripod", gaitName(GAIT_TRIPOD));
    TEST_ASSERT_EQUAL_STRING("ripple", gaitName(GAIT_RIPPLE));
    TEST_ASSERT_EQUAL_STRING("wave", gaitName(GAIT_WAVE));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f / 2, gaitSwingFraction(GAIT_TRIPOD));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f / 3, gaitSwingFraction(GAIT_RIPPLE));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f / 6, gaitSwingFraction(GAIT_WAVE));

    GaitType t;
    TEST_ASSERT_TRUE(gaitFromName("wave", t));
    TEST_ASSERT_EQUAL_INT(GAIT_WAVE, t);
    TEST_ASSERT_FALSE(gaitFromName("run", t));
}

void test_number_of_legs_in_the_air(void) {
    int expected[3] = { 3, 2, 1 };
    for (int g = 0; g < 3; g++) {
        for (int k = 0; k < 600; k++) {
            float phase = (k + 0.5f) / 600.0f;
            int inAir = 0;
            for (int i = 0; i < LEG_COUNT; i++) {
                if (gaitLegInSwing(ALL_GAITS[g], i, phase)) {
                    inAir++;
                }
            }
            TEST_ASSERT_EQUAL_INT_MESSAGE(expected[g], inAir, gaitName(ALL_GAITS[g]));
        }
    }
}

void test_neighbors_never_in_the_air_together(void) {
    int pairs[4][2] = { { 5, 0 }, { 0, 1 }, { 4, 3 }, { 3, 2 } };
    for (int g = 0; g < 3; g++) {
        for (int k = 0; k < 600; k++) {
            float phase = (k + 0.5f) / 600.0f;
            for (int p = 0; p < 4; p++) {
                bool both = gaitLegInSwing(ALL_GAITS[g], pairs[p][0], phase)
                         && gaitLegInSwing(ALL_GAITS[g], pairs[p][1], phase);
                TEST_ASSERT_FALSE_MESSAGE(both, gaitName(ALL_GAITS[g]));
            }
        }
    }
}

static float swingStart(GaitType type, int legIndex) {
    for (int k = 0; k < 12000; k++) {
        float before = k / 6000.0f;
        float after = (k + 1) / 6000.0f;
        if (!gaitLegInSwing(type, legIndex, before) && gaitLegInSwing(type, legIndex, after)) {
            return after - floorf(after);
        }
    }
    return -1.0f;
}

void test_ripple_order(void) {
    float expected[6] = { 2.0f / 6, 0.0f, 3.0f / 6, 5.0f / 6, 1.0f / 6, 4.0f / 6 };
    for (int i = 0; i < LEG_COUNT; i++) {
        float start = swingStart(GAIT_RIPPLE, i);
        if (start > 0.999f) {
            start = 0.0f;
        }
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.001f, expected[i], start, LEGS[i].name);
    }
}

void test_wave_order(void) {
    float expected[6] = { 1.0f / 6, 0.0f, 3.0f / 6, 4.0f / 6, 5.0f / 6, 2.0f / 6 };
    for (int i = 0; i < LEG_COUNT; i++) {
        float start = swingStart(GAIT_WAVE, i);
        if (start > 0.999f) {
            start = 0.0f;
        }
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.001f, expected[i], start, LEGS[i].name);
    }
}

void test_cycle_time(void) {
    int expected[3] = { 50, 75, 150 };
    for (int g = 0; g < 3; g++) {
        Gait gait = makeGait();
        gait.type = ALL_GAITS[g];
        gaitWalk(gait, makeCommand(0, -1, 0));
        float last = gait.phase;
        int updates = 0;
        do {
            last = gait.phase;
            gaitUpdate(gait, DT);
            updates++;
        } while (gait.phase >= last && updates < 1000);
        TEST_ASSERT_TRUE_MESSAGE(updates >= expected[g] - 1 && updates <= expected[g] + 1,
                                 gaitName(ALL_GAITS[g]));
    }
}

void test_every_gait_no_limit_hits_and_smooth(void) {
    Body body = makeBody();
    GaitCommand commands[8] = { makeCommand(0, -1, 0), makeCommand(0, 1, 0), makeCommand(1, 0, 0),
                                makeCommand(-1, 0, 0), makeCommand(0, 0, 1), makeCommand(0, 0, -1),
                                makeCommand(0, -1, 1), makeCommand(0.7f, -0.7f, 0.3f) };
    for (int g = 0; g < 3; g++) {
        float worst = 0.0f;
        for (int c = 0; c < 8; c++) {
            Gait gait = makeGait();
            gait.type = ALL_GAITS[g];
            JointAngles last[LEG_COUNT];
            IkStatus status[LEG_COUNT];
            Vec3 feet[LEG_COUNT];
            for (int i = 0; i < LEG_COUNT; i++) {
                feet[i] = neutralFoot(body.legs[i]);
            }
            TEST_ASSERT_TRUE(bodyIK(body, feet, last, status));

            gaitWalk(gait, commands[c]);
            for (int k = 0; k < 60 + 2 * 150; k++) {
                gaitUpdate(gait, DT);
                JointAngles angles[LEG_COUNT];
                gaitFeet(body, gait.type, gait.phase, gait.current, feet);
                for (int i = 0; i < LEG_COUNT; i++) {
                    TEST_ASSERT_EQUAL_INT_MESSAGE(IK_OK, legIK(body.legs[i], feet[i], angles[i]),
                                                  gaitName(ALL_GAITS[g]));
                    float jump = fmaxf(fabsf(angles[i].coxa - last[i].coxa),
                                 fmaxf(fabsf(angles[i].femur - last[i].femur),
                                       fabsf(angles[i].tibia - last[i].tibia)));
                    if (jump > worst) {
                        worst = jump;
                    }
                    TEST_ASSERT_TRUE_MESSAGE(jump < MAX_STEP_DEG, gaitName(ALL_GAITS[g]));
                    last[i] = angles[i];
                }
            }
        }
        char message[80];
        snprintf(message, sizeof(message), "%s: 8 commands, biggest joint step %.2f deg",
                 gaitName(ALL_GAITS[g]), worst);
        TEST_MESSAGE(message);
    }
}

void test_default_gait_is_ripple(void) {
    Controller c = makeController();
    TEST_ASSERT_EQUAL_STRING("ripple", gaitName(c.gait.type));
}

void test_gait_change_only_when_not_moving(void) {
    Controller c = makeController();
    TEST_ASSERT_TRUE(controllerSetGait(c, GAIT_WAVE).accepted);
    TEST_ASSERT_EQUAL_INT(GAIT_WAVE, c.gait.type);

    controllerUp(c);
    TEST_ASSERT_FALSE(controllerSetGait(c, GAIT_TRIPOD).accepted);
    while (c.state != STATE_STAND) {
        controllerUpdate(c, DT);
    }
    TEST_ASSERT_TRUE(controllerSetGait(c, GAIT_TRIPOD).accepted);

    controllerWalk(c, makeCommand(0, -1, 0));
    TEST_ASSERT_FALSE(controllerSetGait(c, GAIT_RIPPLE).accepted);
    TEST_ASSERT_EQUAL_INT(GAIT_TRIPOD, c.gait.type);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_names_and_swing_fractions);
    RUN_TEST(test_number_of_legs_in_the_air);
    RUN_TEST(test_neighbors_never_in_the_air_together);
    RUN_TEST(test_ripple_order);
    RUN_TEST(test_wave_order);
    RUN_TEST(test_cycle_time);
    RUN_TEST(test_every_gait_no_limit_hits_and_smooth);
    RUN_TEST(test_default_gait_is_ripple);
    RUN_TEST(test_gait_change_only_when_not_moving);
    return UNITY_END();
}
