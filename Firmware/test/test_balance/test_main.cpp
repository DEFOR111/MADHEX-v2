#include <unity.h>
#include <math.h>
#include <stdio.h>
#include "config.h"
#include "body.h"
#include "gait.h"
#include "balance.h"

void setUp(void) {}
void tearDown(void) {}

static const float DT = UPDATE_PERIOD_MS / 1000.0f;

static void standingFeet(const Body &body, Vec3 feet[]) {
    for (int i = 0; i < LEG_COUNT; i++) {
        feet[i] = neutralFoot(body.legs[i]);
    }
}

static void allFeetReachable(const Body &body, const Vec3 feet[]) {
    for (int i = 0; i < LEG_COUNT; i++) {
        JointAngles a;
        TEST_ASSERT_EQUAL_INT(IK_OK, legIK(body.legs[i], feet[i], a));
    }
}

void test_roll_moves_left_feet_up_right_feet_down(void) {
    Body body = makeBody();
    Vec3 flat[LEG_COUNT];
    Vec3 out[LEG_COUNT];
    standingFeet(body, flat);
    levelRotateFeet(5.0f, 0.0f, 0.0f, flat, out);
    for (int i = 0; i < LEG_COUNT; i++) {
        TEST_ASSERT_FLOAT_WITHIN(0.5f, flat[i].x * sinf(degToRad(5.0f)), out[i].z);
    }
    TEST_ASSERT_TRUE(out[0].z < -20.0f);
    TEST_ASSERT_TRUE(out[3].z > 20.0f);
}

void test_pitch_moves_front_feet_up_back_feet_down(void) {
    Body body = makeBody();
    Vec3 flat[LEG_COUNT];
    Vec3 out[LEG_COUNT];
    standingFeet(body, flat);
    levelRotateFeet(0.0f, 5.0f, 0.0f, flat, out);
    TEST_ASSERT_TRUE(out[4].z > 10.0f);
    TEST_ASSERT_TRUE(out[5].z > 10.0f);
    TEST_ASSERT_TRUE(out[1].z < -10.0f);
    TEST_ASSERT_TRUE(out[2].z < -10.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out[0].z);
}

void test_rotation_is_rigid_around_floor_point(void) {
    Body body = makeBody();
    Vec3 flat[LEG_COUNT];
    standingFeet(body, flat);
    for (int i = 0; i < LEG_COUNT; i++) {
        flat[i].z = 30.0f;
    }
    Vec3 out[LEG_COUNT];
    levelRotateFeet(-7.0f, 4.0f, 30.0f, flat, out);
    for (int i = 0; i < LEG_COUNT; i++) {
        for (int j = i + 1; j < LEG_COUNT; j++) {
            TEST_ASSERT_FLOAT_WITHIN(0.01f, vecLength(vecSub(flat[i], flat[j])), vecLength(vecSub(out[i], out[j])));
        }
    }
    Vec3 pivot[LEG_COUNT];
    for (int i = 0; i < LEG_COUNT; i++) {
        pivot[i] = { 0.0f, 0.0f, 30.0f };
    }
    levelRotateFeet(-7.0f, 4.0f, 30.0f, pivot, out);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 30.0f, out[0].z);
}

static void simulate(Leveling &lv, float slopeRoll, float slopePitch, float seconds, bool active,
                     const Body &body, Vec3 feet[], float &biggestStep) {
    biggestStep = 0.0f;
    int steps = (int)(seconds / DT + 0.5f);
    for (int k = 0; k < steps; k++) {
        standingFeet(body, feet);
        float before = lv.correctionRollDeg;
        float beforeP = lv.correctionPitchDeg;
        levelUpdate(lv, active, slopeRoll - lv.correctionRollDeg, slopePitch - lv.correctionPitchDeg, DT,
                    body, 0.0f, feet);
        biggestStep = fmaxf(biggestStep, fabsf(lv.correctionRollDeg - before));
        biggestStep = fmaxf(biggestStep, fabsf(lv.correctionPitchDeg - beforeP));
    }
}

void test_slope_5_deg_body_becomes_level(void) {
    Body body = makeBody();
    Leveling lv = makeLeveling();
    lv.enabled = true;
    Vec3 feet[LEG_COUNT];
    float biggestStep;
    simulate(lv, 5.0f, 0.0f, 10.0f, true, body, feet, biggestStep);

    float tilt = 5.0f - lv.correctionRollDeg;
    TEST_ASSERT_TRUE(fabsf(tilt) <= LEVEL_DEADBAND_DEG + 0.05f);
    TEST_ASSERT_FALSE(lv.limited);
    TEST_ASSERT_TRUE(feet[0].z < -20.0f);
    TEST_ASSERT_TRUE(feet[1].z < -10.0f);
    TEST_ASSERT_TRUE(feet[5].z < -10.0f);
    TEST_ASSERT_TRUE(feet[3].z > 20.0f);
    allFeetReachable(body, feet);
    TEST_ASSERT_TRUE(biggestStep <= LEVEL_MAX_RATE_DEG_S * DT + 0.0001f);
}

void test_slope_front_up_and_sideways_together(void) {
    Body body = makeBody();
    Leveling lv = makeLeveling();
    Vec3 feet[LEG_COUNT];
    float biggestStep;
    simulate(lv, -4.0f, 5.0f, 10.0f, true, body, feet, biggestStep);
    TEST_ASSERT_TRUE(fabsf(-4.0f - lv.correctionRollDeg) <= LEVEL_DEADBAND_DEG + 0.05f);
    TEST_ASSERT_TRUE(fabsf(5.0f - lv.correctionPitchDeg) <= LEVEL_DEADBAND_DEG + 0.05f);
    allFeetReachable(body, feet);
}

void test_slope_steeper_than_limit(void) {
    Body body = makeBody();
    Leveling lv = makeLeveling();
    Vec3 feet[LEG_COUNT];
    float biggestStep;
    simulate(lv, 15.0f, 0.0f, 10.0f, true, body, feet, biggestStep);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, LEVEL_MAX_DEG, lv.correctionRollDeg);
    allFeetReachable(body, feet);
}

void test_not_active_goes_back_to_zero_slowly(void) {
    Body body = makeBody();
    Leveling lv = makeLeveling();
    lv.correctionRollDeg = 6.0f;
    Vec3 feet[LEG_COUNT];
    float biggestStep;
    simulate(lv, 6.0f, 0.0f, 0.6f, false, body, feet, biggestStep);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.0f, lv.correctionRollDeg);
    simulate(lv, 6.0f, 0.0f, 1.0f, false, body, feet, biggestStep);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, lv.correctionRollDeg);
    TEST_ASSERT_TRUE(biggestStep <= LEVEL_MAX_RATE_DEG_S * DT + 0.0001f);
}

void test_dead_band(void) {
    Body body = makeBody();
    Leveling lv = makeLeveling();
    Vec3 feet[LEG_COUNT];
    float biggestStep;
    simulate(lv, 0.3f, -0.4f, 5.0f, true, body, feet, biggestStep);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, lv.correctionRollDeg);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, lv.correctionPitchDeg);
}

void test_sway_is_filtered(void) {
    Body body = makeBody();
    Leveling lv = makeLeveling();
    Vec3 feet[LEG_COUNT];
    float biggest = 0.0f;
    for (int k = 0; k < 500; k++) {
        standingFeet(body, feet);
        float sway = 2.0f * sinf(2.0f * MATHX_PI * 1.0f * k * DT);
        levelUpdate(lv, true, sway, 0.5f * sway, DT, body, 0.0f, feet);
        biggest = fmaxf(biggest, fmaxf(fabsf(lv.correctionRollDeg), fabsf(lv.correctionPitchDeg)));
    }
    TEST_ASSERT_TRUE(biggest < 0.5f);
}

void test_max_correction_reachable_standing_and_walking(void) {
    Body body = makeBody();
    float m = LEVEL_MAX_DEG;
    float corrections[8][2] = { { m, 0 }, { -m, 0 }, { 0, m }, { 0, -m }, { m, m }, { m, -m }, { -m, m }, { -m, -m } };
    GaitType gaits[3] = { GAIT_TRIPOD, GAIT_RIPPLE, GAIT_WAVE };
    GaitCommand commands[5] = { makeCommand(0, 0, 0), makeCommand(0, -1, 0), makeCommand(1, 0, 0),
                                makeCommand(0, 0, 1), makeCommand(-0.7f, 0.7f, 0) };
    Vec3 feet[LEG_COUNT];
    int checked = 0;
    for (int g = 0; g < 3; g++) {
        for (int c = 0; c < 5; c++) {
            for (int p = 0; p < 100; p++) {
                gaitFeet(body, gaits[g], p / 100.0f, commands[c], feet);
                for (int k = 0; k < 8; k++) {
                    if (!levelFeetReachable(body, feet, corrections[k][0], corrections[k][1], 0.0f)) {
                        char msg[96];
                        snprintf(msg, sizeof(msg), "gait %d command %d phase %d correction %.0f %.0f",
                                 g, c, p, corrections[k][0], corrections[k][1]);
                        TEST_FAIL_MESSAGE(msg);
                    }
                    checked++;
                }
            }
        }
    }
    TEST_ASSERT_EQUAL_INT(3 * 5 * 100 * 8, checked);
}

void test_unreachable_correction_is_reduced(void) {
    Body body = makeBody();
    Vec3 feet[LEG_COUNT];
    for (int i = 0; i < LEG_COUNT; i++) {
        feet[i] = footOnLegLine(body.legs[i], 300.0f, 0.0f);
    }
    TEST_ASSERT_TRUE(levelFeetReachable(body, feet, 0.0f, 0.0f, 0.0f));
    TEST_ASSERT_FALSE(levelFeetReachable(body, feet, LEVEL_MAX_DEG, 0.0f, 0.0f));

    Leveling lv = makeLeveling();
    lv.correctionRollDeg = LEVEL_MAX_DEG;
    levelUpdate(lv, true, 0.0f, 0.0f, DT, body, 0.0f, feet);
    TEST_ASSERT_TRUE(lv.limited);
    TEST_ASSERT_TRUE(lv.correctionRollDeg < LEVEL_MAX_DEG);
    allFeetReachable(body, feet);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_roll_moves_left_feet_up_right_feet_down);
    RUN_TEST(test_pitch_moves_front_feet_up_back_feet_down);
    RUN_TEST(test_rotation_is_rigid_around_floor_point);
    RUN_TEST(test_slope_5_deg_body_becomes_level);
    RUN_TEST(test_slope_front_up_and_sideways_together);
    RUN_TEST(test_slope_steeper_than_limit);
    RUN_TEST(test_not_active_goes_back_to_zero_slowly);
    RUN_TEST(test_dead_band);
    RUN_TEST(test_sway_is_filtered);
    RUN_TEST(test_max_correction_reachable_standing_and_walking);
    RUN_TEST(test_unreachable_correction_is_reduced);
    return UNITY_END();
}
