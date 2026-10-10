#include <math.h>
#include <stdio.h>
#include <unity.h>
#include "gait.h"

const float TOL_MM = 0.01f;
const float DT = 0.02f;
const float MAX_STEP_DEG = 3.0f;

void setUp(void) {}
void tearDown(void) {}

struct Sim {
    Body body;
    Gait gait;
    JointAngles last[LEG_COUNT];
    float maxStep;
};

static Sim makeSim() {
    Sim sim;
    sim.body = makeBody();
    sim.gait = makeGait();
    sim.gait.type = GAIT_TRIPOD;
    sim.maxStep = 0.0f;
    Vec3 feet[LEG_COUNT];
    IkStatus status[LEG_COUNT];
    for (int i = 0; i < LEG_COUNT; i++) {
        feet[i] = neutralFoot(sim.body.legs[i]);
    }
    TEST_ASSERT_TRUE(bodyIK(sim.body, feet, sim.last, status));
    return sim;
}

static void tick(Sim &sim) {
    gaitUpdate(sim.gait, DT);
    Vec3 feet[LEG_COUNT];
    JointAngles angles[LEG_COUNT];
    IkStatus status[LEG_COUNT];
    gaitFeet(sim.body, sim.gait.type, sim.gait.phase, sim.gait.current, feet);
    TEST_ASSERT_TRUE_MESSAGE(bodyIK(sim.body, feet, angles, status), "IK failed");
    for (int i = 0; i < LEG_COUNT; i++) {
        float jump = fmaxf(fabsf(angles[i].coxa - sim.last[i].coxa),
                     fmaxf(fabsf(angles[i].femur - sim.last[i].femur),
                           fabsf(angles[i].tibia - sim.last[i].tibia)));
        if (jump > sim.maxStep) {
            sim.maxStep = jump;
        }
        TEST_ASSERT_TRUE_MESSAGE(jump < MAX_STEP_DEG, LEGS[i].name);
        sim.last[i] = angles[i];
    }
}

static void ticks(Sim &sim, int count) {
    for (int k = 0; k < count; k++) {
        tick(sim);
    }
}

static int stopAndWait(Sim &sim) {
    gaitStop(sim.gait);
    int count = 0;
    while (sim.gait.walking && count < 500) {
        tick(sim);
        count++;
    }
    TEST_ASSERT_FALSE_MESSAGE(sim.gait.walking, "did not stop");
    return count;
}

static void assertFeetAtNeutral(const Sim &sim) {
    Vec3 feet[LEG_COUNT];
    gaitFeet(sim.body, sim.gait.type, sim.gait.phase, sim.gait.current, feet);
    for (int i = 0; i < LEG_COUNT; i++) {
        Vec3 n = neutralFoot(sim.body.legs[i]);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_MM, n.x, feet[i].x, LEGS[i].name);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_MM, n.y, feet[i].y, LEGS[i].name);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_MM, n.z, feet[i].z, LEGS[i].name);
    }
}

static void report(const char *name, const Sim &sim) {
    char message[80];
    snprintf(message, sizeof(message), "%s: biggest joint step %.2f deg", name, sim.maxStep);
    TEST_MESSAGE(message);
}

void test_lift_fades_with_motion(void) {
    Body body = makeBody();
    const Leg &a = body.legs[0];
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, 0.0f, gaitFoot(a, 0, GAIT_TRIPOD, 0.75f, makeCommand(0, 0, 0)).z);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, 10.0f, gaitFoot(a, 0, GAIT_TRIPOD, 0.75f, makeCommand(0, -0.1f, 0)).z);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, 5.0f, gaitFoot(a, 0, GAIT_TRIPOD, 0.75f, makeCommand(0, 0, 0.05f)).z);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, 25.0f, gaitFoot(a, 0, GAIT_TRIPOD, 0.75f, makeCommand(0, -0.25f, 0)).z);
    TEST_ASSERT_FLOAT_WITHIN(TOL_MM, 25.0f, gaitFoot(a, 0, GAIT_TRIPOD, 0.75f, makeCommand(0, -1, 0)).z);
}

void test_ramp_goes_straight_and_stays_safe(void) {
    Gait gait = makeGait();
    gaitWalk(gait, makeCommand(0, -1, 0));
    for (int k = 0; k < 60; k++) {
        gaitUpdate(gait, DT);
    }
    GaitCommand from = gait.current;
    GaitCommand to = makeCommand(0.5f, 0, -0.5f);
    gaitWalk(gait, to);

    int updates = 0;
    while ((gait.current.vx != to.vx || gait.current.vy != to.vy || gait.current.turn != to.turn)
           && updates < 200) {
        gaitUpdate(gait, DT);
        updates++;
        GaitCommand c = gait.current;
        float total = sqrtf(c.vx * c.vx + c.vy * c.vy) + fabsf(c.turn);
        TEST_ASSERT_TRUE(total <= 1.0001f);
        float pvx = (c.vx - from.vx) / (to.vx - from.vx);
        float pvy = (c.vy - from.vy) / (to.vy - from.vy);
        float pturn = (c.turn - from.turn) / (to.turn - from.turn);
        TEST_ASSERT_FLOAT_WITHIN(0.0001f, pvy, pvx);
        TEST_ASSERT_FLOAT_WITHIN(0.0001f, pvy, pturn);
    }
    TEST_ASSERT_TRUE(updates >= 50 && updates <= 51);
}

void test_start_from_stance(void) {
    Sim sim = makeSim();
    gaitWalk(sim.gait, makeCommand(0, -1, 0));
    ticks(sim, 150);
    report("start", sim);
}

void test_stop_ends_with_feet_at_neutral(void) {
    Sim sim = makeSim();
    gaitWalk(sim.gait, makeCommand(0, -1, 0));
    ticks(sim, 150);
    sim.maxStep = 0.0f;
    int stopUpdates = stopAndWait(sim);
    TEST_ASSERT_TRUE(stopUpdates >= 50 && stopUpdates <= 51);
    assertFeetAtNeutral(sim);
    report("stop", sim);
}

void test_sudden_reverse(void) {
    Sim sim = makeSim();
    gaitWalk(sim.gait, makeCommand(0, -1, 0));
    ticks(sim, 150);
    sim.maxStep = 0.0f;
    gaitWalk(sim.gait, makeCommand(0, 1, 0));
    ticks(sim, 200);
    report("sudden reverse", sim);
}

void test_sudden_turn(void) {
    Sim sim = makeSim();
    gaitWalk(sim.gait, makeCommand(0, -1, 0));
    ticks(sim, 150);
    sim.maxStep = 0.0f;
    gaitWalk(sim.gait, makeCommand(0, 0, -1));
    ticks(sim, 150);
    report("sudden turn", sim);
}

void test_sudden_left_to_right(void) {
    Sim sim = makeSim();
    gaitWalk(sim.gait, makeCommand(1, 0, 0));
    ticks(sim, 150);
    sim.maxStep = 0.0f;
    gaitWalk(sim.gait, makeCommand(-1, 0, 0));
    ticks(sim, 200);
    report("left -> right", sim);
}

void test_sudden_walk_turn_flip(void) {
    Sim sim = makeSim();
    gaitWalk(sim.gait, makeCommand(0, -0.5f, 0.5f));
    ticks(sim, 150);
    sim.maxStep = 0.0f;
    gaitWalk(sim.gait, makeCommand(0, 0.5f, -0.5f));
    ticks(sim, 200);
    report("walk+turn flip", sim);
}

void test_steady_walking_in_all_directions(void) {
    GaitCommand commands[30];
    int count = 0;
    for (int a = 0; a < 360; a += 15) {
        float r = degToRad((float)a);
        commands[count++] = makeCommand(cosf(r), sinf(r), 0);
    }
    commands[count++] = makeCommand(0, 0, 1);
    commands[count++] = makeCommand(0, 0, -1);
    commands[count++] = makeCommand(0, -1, 1);
    commands[count++] = makeCommand(1, 0, -1);

    float worst = 0.0f;
    for (int c = 0; c < count; c++) {
        Sim sim = makeSim();
        gaitWalk(sim.gait, commands[c]);
        ticks(sim, 60);
        sim.maxStep = 0.0f;
        ticks(sim, 100);
        if (sim.maxStep > worst) {
            worst = sim.maxStep;
        }
    }
    char message[80];
    snprintf(message, sizeof(message), "%d steady commands: biggest joint step %.2f deg", count, worst);
    TEST_MESSAGE(message);
}

void test_stop_at_any_moment_ends_at_neutral(void) {
    for (int extra = 0; extra < 50; extra++) {
        Sim sim = makeSim();
        gaitWalk(sim.gait, makeCommand(0.3f, -0.6f, 0.1f));
        ticks(sim, 100 + extra);
        stopAndWait(sim);
        assertFeetAtNeutral(sim);
    }
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_lift_fades_with_motion);
    RUN_TEST(test_ramp_goes_straight_and_stays_safe);
    RUN_TEST(test_start_from_stance);
    RUN_TEST(test_stop_ends_with_feet_at_neutral);
    RUN_TEST(test_sudden_reverse);
    RUN_TEST(test_sudden_turn);
    RUN_TEST(test_sudden_left_to_right);
    RUN_TEST(test_sudden_walk_turn_flip);
    RUN_TEST(test_steady_walking_in_all_directions);
    RUN_TEST(test_stop_at_any_moment_ends_at_neutral);
    return UNITY_END();
}
