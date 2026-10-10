#include <math.h>
#include <stdio.h>
#include <unity.h>
#include "safety.h"

const float DT = 0.02f;
const int UPDATES_PER_CHECK = 50;
const float MAX_STEP_DEG = 3.0f;

void setUp(void) {}
void tearDown(void) {}

void test_normal_readings_no_fault(void) {
    Safety s = makeSafety();
    for (int i = 0; i < 40; i++) {
        TEST_ASSERT_FALSE(safetyReport(s, i % SAFETY_SERVO_COUNT, true, 7400, 40));
    }
    TEST_ASSERT_EQUAL_INT(SAFETY_FAULT_NONE, s.fault);
    TEST_ASSERT_FALSE(s.noPower);
}

void test_low_voltage_needs_two_readings_in_a_row(void) {
    Safety s = makeSafety();
    TEST_ASSERT_FALSE(safetyReport(s, 0, true, 6400, 40));
    TEST_ASSERT_EQUAL_INT(SAFETY_FAULT_NONE, s.fault);
    TEST_ASSERT_TRUE(safetyReport(s, 1, true, 6400, 40));
    TEST_ASSERT_EQUAL_INT(SAFETY_FAULT_LOW_VOLTAGE, s.fault);
    TEST_ASSERT_EQUAL_INT(1, s.faultServo);
    TEST_ASSERT_EQUAL_INT(6400, s.faultValue);
    TEST_ASSERT_FALSE(safetyReport(s, 2, true, 6400, 40));
}

void test_low_voltage_not_in_a_row_is_ignored(void) {
    Safety s = makeSafety();
    safetyReport(s, 0, true, 6400, 40);
    safetyReport(s, 1, true, 7400, 40);
    safetyReport(s, 2, true, 6400, 40);
    TEST_ASSERT_EQUAL_INT(SAFETY_FAULT_NONE, s.fault);
}

void test_overheat_threshold(void) {
    Safety s = makeSafety();
    TEST_ASSERT_FALSE(safetyReport(s, 0, true, 7400, 69));
    TEST_ASSERT_EQUAL_INT(SAFETY_FAULT_NONE, s.fault);
    TEST_ASSERT_TRUE(safetyReport(s, 1, true, 7400, 70));
    TEST_ASSERT_EQUAL_INT(SAFETY_FAULT_OVERHEAT, s.fault);

    Safety s2 = makeSafety();
    TEST_ASSERT_TRUE(safetyReport(s2, 5, true, 7400, 80));
    TEST_ASSERT_EQUAL_INT(SAFETY_FAULT_OVERHEAT, s2.fault);
    TEST_ASSERT_EQUAL_INT(80, s2.faultValue);
}

void test_no_answer_is_no_power_not_a_fault(void) {
    Safety s = makeSafety();
    for (int i = 0; i < SAFETY_SERVO_COUNT - 1; i++) {
        safetyReport(s, i, false, 0, 0);
    }
    TEST_ASSERT_FALSE(s.noPower);
    safetyReport(s, SAFETY_SERVO_COUNT - 1, false, 0, 0);
    TEST_ASSERT_TRUE(s.noPower);
    TEST_ASSERT_EQUAL_INT(SAFETY_FAULT_NONE, s.fault);

    safetyReport(s, 0, true, 7400, 40);
    TEST_ASSERT_FALSE(s.noPower);
}

void test_fault_stays_until_cleared(void) {
    Safety s = makeSafety();
    safetyReport(s, 0, true, 7400, 85);
    for (int i = 1; i < 10; i++) {
        safetyReport(s, i, true, 7400, 40);
    }
    TEST_ASSERT_EQUAL_INT(SAFETY_FAULT_OVERHEAT, s.fault);
    safetyClear(s);
    TEST_ASSERT_EQUAL_INT(SAFETY_FAULT_NONE, s.fault);
}

void test_round_robin_over_18_servos(void) {
    Safety s = makeSafety();
    for (int i = 0; i < 40; i++) {
        TEST_ASSERT_EQUAL_INT(i % SAFETY_SERVO_COUNT, s.nextServo);
        safetyReport(s, s.nextServo, true, 7400, 40);
    }
}

struct Sim {
    Body body;
    Controller ctrl;
    Safety safety;
    JointAngles last[LEG_COUNT];
    float maxStep;
    int updates;
};

static Sim makeSim() {
    Sim sim;
    sim.body = makeBody();
    sim.ctrl = makeController();
    sim.safety = makeSafety();
    sim.maxStep = 0.0f;
    sim.updates = 0;
    Vec3 feet[LEG_COUNT];
    IkStatus status[LEG_COUNT];
    controllerFeet(sim.ctrl, sim.body, feet);
    TEST_ASSERT_TRUE(bodyIK(sim.body, feet, sim.last, status));
    return sim;
}

static void tick(Sim &sim) {
    sim.updates++;
    if (sim.safety.fake && sim.updates % UPDATES_PER_CHECK == 0) {
        safetyReport(sim.safety, sim.safety.nextServo, true, sim.safety.fakeVoltageMv, sim.safety.fakeTempC);
    }
    safetyApply(sim.safety, sim.ctrl);
    controllerUpdate(sim.ctrl, DT);

    Vec3 feet[LEG_COUNT];
    JointAngles angles[LEG_COUNT];
    IkStatus status[LEG_COUNT];
    controllerFeet(sim.ctrl, sim.body, feet);
    TEST_ASSERT_TRUE_MESSAGE(bodyIK(sim.body, feet, angles, status), stateName(sim.ctrl.state));
    for (int i = 0; i < LEG_COUNT; i++) {
        float jump = fmaxf(fabsf(angles[i].coxa - sim.last[i].coxa),
                     fmaxf(fabsf(angles[i].femur - sim.last[i].femur),
                           fabsf(angles[i].tibia - sim.last[i].tibia)));
        if (jump > sim.maxStep) {
            sim.maxStep = jump;
        }
        TEST_ASSERT_TRUE_MESSAGE(jump < MAX_STEP_DEG, stateName(sim.ctrl.state));
        sim.last[i] = angles[i];
    }
}

static void ticks(Sim &sim, int count) {
    for (int k = 0; k < count; k++) {
        tick(sim);
    }
}

static void standUp(Sim &sim) {
    controllerUp(sim.ctrl);
    ticks(sim, 80);
    TEST_ASSERT_EQUAL_STRING("STAND", stateName(sim.ctrl.state));
}

void test_fake_low_voltage_while_walking_sits_down(void) {
    Sim sim = makeSim();
    standUp(sim);
    controllerWalk(sim.ctrl, makeCommand(0, -1, 0));
    ticks(sim, 150);
    TEST_ASSERT_EQUAL_STRING("WALK", stateName(sim.ctrl.state));

    sim.safety.fake = true;
    sim.safety.fakeVoltageMv = 6400;
    sim.safety.fakeTempC = 40;
    int k = 0;
    while (sim.ctrl.state != STATE_SIT && k < 1000) {
        tick(sim);
        k++;
    }
    TEST_ASSERT_EQUAL_STRING("SIT", stateName(sim.ctrl.state));
    TEST_ASSERT_EQUAL_INT(SAFETY_FAULT_LOW_VOLTAGE, sim.safety.fault);
    TEST_ASSERT_FALSE(controllerUp(sim.ctrl).accepted);
    TEST_ASSERT_FALSE(controllerWalk(sim.ctrl, makeCommand(0, -1, 0)).accepted);

    char message[96];
    snprintf(message, sizeof(message), "low voltage: SIT after %d updates (%.1f s), biggest joint step %.2f deg",
             k, k * DT, sim.maxStep);
    TEST_MESSAGE(message);

    sim.safety.fake = false;
    safetyClear(sim.safety);
    controllerClearSafety(sim.ctrl);
    TEST_ASSERT_TRUE(controllerUp(sim.ctrl).accepted);
}

void test_fake_overheat_while_standing_up(void) {
    Sim sim = makeSim();
    controllerUp(sim.ctrl);
    ticks(sim, 10);
    sim.safety.fake = true;
    sim.safety.fakeVoltageMv = 7400;
    sim.safety.fakeTempC = 80;
    int k = 0;
    while (sim.ctrl.state != STATE_SIT && k < 1000) {
        tick(sim);
        k++;
    }
    TEST_ASSERT_EQUAL_STRING("SIT", stateName(sim.ctrl.state));
    TEST_ASSERT_EQUAL_INT(SAFETY_FAULT_OVERHEAT, sim.safety.fault);

    char message[96];
    snprintf(message, sizeof(message), "overheat: SIT after %d updates, biggest joint step %.2f deg", k, sim.maxStep);
    TEST_MESSAGE(message);
}

void test_emergency_stop_freezes_everything(void) {
    Sim sim = makeSim();
    standUp(sim);
    controllerWalk(sim.ctrl, makeCommand(0, -1, 0));
    ticks(sim, 137);

    TEST_ASSERT_TRUE(controllerEmergencyStop(sim.ctrl).accepted);
    TEST_ASSERT_EQUAL_STRING("ESTOP", stateName(sim.ctrl.state));

    Vec3 frozen[LEG_COUNT];
    controllerFeet(sim.ctrl, sim.body, frozen);
    ticks(sim, 200);
    Vec3 later[LEG_COUNT];
    controllerFeet(sim.ctrl, sim.body, later);
    for (int i = 0; i < LEG_COUNT; i++) {
        TEST_ASSERT_FLOAT_WITHIN(0.0001f, frozen[i].x, later[i].x);
        TEST_ASSERT_FLOAT_WITHIN(0.0001f, frozen[i].y, later[i].y);
        TEST_ASSERT_FLOAT_WITHIN(0.0001f, frozen[i].z, later[i].z);
    }
    TEST_ASSERT_TRUE(sim.maxStep < MAX_STEP_DEG);

    TEST_ASSERT_FALSE(controllerUp(sim.ctrl).accepted);
    TEST_ASSERT_FALSE(controllerDown(sim.ctrl).accepted);
    TEST_ASSERT_FALSE(controllerWalk(sim.ctrl, makeCommand(1, 0, 0)).accepted);
    TEST_ASSERT_FALSE(controllerStop(sim.ctrl).accepted);
    TEST_ASSERT_FALSE(controllerSetGait(sim.ctrl, GAIT_TRIPOD).accepted);
    controllerClearSafety(sim.ctrl);
    TEST_ASSERT_EQUAL_STRING("ESTOP", stateName(sim.ctrl.state));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_normal_readings_no_fault);
    RUN_TEST(test_low_voltage_needs_two_readings_in_a_row);
    RUN_TEST(test_low_voltage_not_in_a_row_is_ignored);
    RUN_TEST(test_overheat_threshold);
    RUN_TEST(test_no_answer_is_no_power_not_a_fault);
    RUN_TEST(test_fault_stays_until_cleared);
    RUN_TEST(test_round_robin_over_18_servos);
    RUN_TEST(test_fake_low_voltage_while_walking_sits_down);
    RUN_TEST(test_fake_overheat_while_standing_up);
    RUN_TEST(test_emergency_stop_freezes_everything);
    return UNITY_END();
}
