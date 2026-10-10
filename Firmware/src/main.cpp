#include <Arduino.h>
#include "commands.h"
#include "body.h"
#include "controller.h"
#include "servo_bus.h"
#include "calibration_store.h"
#include "safety.h"
#include "power_monitor_hw.h"
#include "servo_output_hw.h"
#include "status_led.h"
#include "imu_hw.h"
#include "balance.h"
#include "web.h"

const unsigned long REPORT_PERIOD_MS = 1000;

const int MOVE_PRINT_EVERY = 10;

unsigned long lastUpdateMs = 0;
unsigned long lastReportMs = 0;

unsigned long updateCount = 0;

unsigned long maxLoopUs = 0;
unsigned long maxGapUs = 0;
unsigned long lastUpdateUs = 0;
LoopStats loopStats = { 0, 0, 0 };

Body body;
Controller ctrl;
Calibration calib;
Safety safety;
Leveling leveling;

enum SafetyStep { SAFETY_IDLE, SAFETY_WAIT_VOLTAGE, SAFETY_TEMP_NEXT, SAFETY_WAIT_TEMP };
SafetyStep safetyStep = SAFETY_IDLE;
ServoPendingRequest safetyRequest;
unsigned long lastSafetyMs = 0;
int safetyVoltageMv = 0;

void safetyReading(bool answered, int voltageMv, int tempC) {
    int servo = safety.nextServo;
    bool hadNoPower = safety.noPower;
    bool newFault = safetyReport(safety, servo, answered, voltageMv, tempC);

    char line[160];
    int n = 0;
    if (newFault) {
        n = snprintf(line, sizeof(line), "SAFETY: %s (servo %s %s: %d %s) -> sitting down\r\n",
                     safetyFaultText(safety.fault), LEGS[servo / 3].name, jointName(servo % 3),
                     safety.faultValue, safety.fault == SAFETY_FAULT_OVERHEAT ? "C" : "mV");
    } else if (safety.noPower && !hadNoPower) {
        n = snprintf(line, sizeof(line),
                     "safety: servos do not answer (no servo power / battery not connected?) - no shutdown\r\n");
    } else if (!safety.noPower && hadNoPower) {
        n = snprintf(line, sizeof(line), "safety: servos answer again\r\n");
    }
    if (n > 0) {
        Serial.write((const uint8_t *)line, n);
    }
}

void safetyLoop() {
    int servo = safety.nextServo;
    int leg = servo / 3;
    int joint = servo % 3;
    uint8_t request[SERVO_MAX_PACKET];

    if (safetyStep == SAFETY_IDLE) {
        if (millis() - lastSafetyMs < SAFETY_CHECK_PERIOD_MS) {
            return;
        }
        if (safety.fake) {
            lastSafetyMs = millis();
            safetyReading(true, safety.fakeVoltageMv, safety.fakeTempC);
            return;
        }
        if (!servoOutputBusFree(SAFETY_REPLY_TIMEOUT_MS)) {
            return;
        }
        lastSafetyMs = millis();
        int length = servoBuildRead(SERVO_ID[leg][joint], SERVO_VIN_READ, request);
        servoRequestStart(safetyRequest, SERVO_BUS_OF_LEG[leg], request, length);
        safetyStep = SAFETY_WAIT_VOLTAGE;
        return;
    }
    if (safetyStep == SAFETY_TEMP_NEXT) {
        if (!servoOutputBusFree(SAFETY_REPLY_TIMEOUT_MS)) {
            return;
        }
        int length = servoBuildRead(SERVO_ID[leg][joint], SERVO_TEMP_READ, request);
        servoRequestStart(safetyRequest, SERVO_BUS_OF_LEG[leg], request, length);
        safetyStep = SAFETY_WAIT_TEMP;
        return;
    }

    ServoReplyStatus status = servoRequestPoll(safetyRequest, SAFETY_REPLY_TIMEOUT_MS);
    if (status == SERVO_PENDING) {
        return;
    }
    if (safetyStep == SAFETY_WAIT_VOLTAGE) {
        if (status == SERVO_REPLY_OK && servoReplyVoltage(safetyRequest.receiver.packet, safetyVoltageMv)) {
            safetyStep = SAFETY_TEMP_NEXT;
        } else {
            safetyReading(false, 0, 0);
            safetyStep = SAFETY_IDLE;
        }
        return;
    }
    int tempC = 0;
    bool ok = (status == SERVO_REPLY_OK && servoReplyTemperature(safetyRequest.receiver.packet, tempC));
    safetyReading(ok, safetyVoltageMv, tempC);
    safetyStep = SAFETY_IDLE;
}
JointAngles jointAngles[LEG_COUNT];
IkStatus ikStatus[LEG_COUNT];
int movePrintCounter = 0;

void printStatusLine(bool allOk) {
    char line[200];
    int n = snprintf(line, sizeof(line), "%s %s z %3d ph %3d vx %4d vy %4d tn %4d |",
                     stateName(ctrl.state),
                     gaitName(ctrl.gait.type),
                     (int)lroundf(ctrl.floorZ),
                     (int)lroundf(ctrl.gait.phase * 100.0f),
                     (int)lroundf(ctrl.gait.current.vx * 100.0f),
                     (int)lroundf(ctrl.gait.current.vy * 100.0f),
                     (int)lroundf(ctrl.gait.current.turn * 100.0f));
    for (int i = 0; i < LEG_COUNT; i++) {
        n += snprintf(line + n, sizeof(line) - n, " %s %d %d %d", body.legs[i].name,
                      (int)lroundf(jointAngles[i].coxa),
                      (int)lroundf(jointAngles[i].femur),
                      (int)lroundf(jointAngles[i].tibia));
    }
    n += snprintf(line + n, sizeof(line) - n, allOk ? " ok\r\n" : " IK ERROR\r\n");
    Serial.write((const uint8_t *)line, n);
}

void robotUpdate() {
    RobotState before = ctrl.state;
    safetyApply(safety, ctrl);
    controllerUpdate(ctrl, UPDATE_PERIOD_MS / 1000.0f);

    Vec3 feet[LEG_COUNT];
    controllerFeet(ctrl, body, feet);

    if (leveling.enabled && !imuPresent()) {
        leveling.enabled = false;
        const char text[] = "level: OFF (IMU lost)\r\n";
        Serial.write((const uint8_t *)text, sizeof(text) - 1);
    }
    if (ctrl.state == STATE_ESTOP) {
        levelRotateFeet(leveling.correctionRollDeg, leveling.correctionPitchDeg, ctrl.floorZ, feet, feet);
    } else {
        ImuReading imu = imuLatest();
        bool levelActive = leveling.enabled && imu.valid && imu.tiltValid
                           && (ctrl.state == STATE_STAND || ctrl.state == STATE_WALK);
        levelUpdate(leveling, levelActive, imu.rollDeg, imu.pitchDeg, UPDATE_PERIOD_MS / 1000.0f,
                    body, ctrl.floorZ, feet);
    }
    bool allOk = bodyIK(body, feet, jointAngles, ikStatus);
    servoOutputUpdate(ctrl, safety, calib, jointAngles);

    bool stateChanged = (ctrl.state != before);
    movePrintCounter++;
    if ((controllerIsMoving(ctrl) && movePrintCounter >= MOVE_PRINT_EVERY) || stateChanged) {
        movePrintCounter = 0;
        printStatusLine(allOk);
    }
    if (stateChanged) {
        char text[32];
        int n = snprintf(text, sizeof(text), "state: %s\r\n", stateName(ctrl.state));
        Serial.write((const uint8_t *)text, n);
    }
}

void checkLegLengths() {
    const LegConfig &ref = LEGS[REFERENCE_LEG];
    for (int i = 0; i < LEG_COUNT; i++) {
        float diff = maxLengthDifferenceMm(LEGS[i], ref);
        if (diff > LENGTH_WARNING_MM) {
            Serial.print("warning: leg ");
            Serial.print(LEGS[i].name);
            Serial.print(" segment length differs from leg ");
            Serial.print(ref.name);
            Serial.print(" by ");
            Serial.print(diff, 2);
            Serial.println(" mm");
        }
    }
}

void setup() {
    servoBusBegin();

    Serial.setTxBufferSize(2048);

    Serial.begin(115200);
    Serial.println("Hexapod started");
    checkLegLengths();
    powerBegin();
    imuBegin();
    statusLedBegin();
    webBegin();

    body = makeBody();
    ctrl = makeController();
    safety = makeSafety();
    leveling = makeLeveling();
    if (calibrationLoad(calib)) {
        Serial.println("calibration: loaded from flash");
    } else {
        Serial.println("calibration: nothing saved yet, using trims 0 and directions +1");
    }
    Serial.println("servos: OFF (type 'servos on' to drive the servos)");
    Serial.println("state: SIT (type up to stand up)");

    lastUpdateMs = millis();
    lastReportMs = lastUpdateMs;
}

void loop() {
    unsigned long passStartUs = micros();

    commandsUpdate(ctrl, calib, safety, leveling);
    webUpdate(ctrl, calib, safety, leveling, loopStats);
    safetyLoop();
    servoOutputLoop(!safetyRequest.active);
    powerUpdate();
    imuUpdate();
    statusLedUpdate(ctrl, safety);

    unsigned long now = millis();

    if (now - lastUpdateMs >= UPDATE_PERIOD_MS) {
        lastUpdateMs += UPDATE_PERIOD_MS;

        if (now - lastUpdateMs >= UPDATE_PERIOD_MS) {
            lastUpdateMs = now;
        }

        unsigned long nowUs = micros();
        if (lastUpdateUs != 0 && nowUs - lastUpdateUs > maxGapUs) {
            maxGapUs = nowUs - lastUpdateUs;
        }
        lastUpdateUs = nowUs;

        robotUpdate();
        updateCount++;
    }

    if (now - lastReportMs >= REPORT_PERIOD_MS) {
        lastReportMs += REPORT_PERIOD_MS;

        loopStats.updatesPerSecond = (int)updateCount;
        loopStats.maxLoopUs = maxLoopUs;
        loopStats.maxGapUs = maxGapUs;

        char line[128];
        int n = snprintf(line, sizeof(line),
                         "t=%lu ms, updates in last second: %lu, longest loop pass %lu us, longest gap %.1f ms\r\n",
                         now, updateCount, maxLoopUs, maxGapUs / 1000.0f);
        Serial.write((const uint8_t *)line, n);

        updateCount = 0;
        maxLoopUs = 0;
        maxGapUs = 0;
    }

    unsigned long passUs = micros() - passStartUs;
    if (passUs > maxLoopUs) {
        maxLoopUs = passUs;
    }
}
