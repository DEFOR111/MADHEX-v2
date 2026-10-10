#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "commands.h"
#include "config.h"
#include "body.h"
#include "calibration_store.h"
#include "servo_bus.h"
#include "servo_packet.h"
#include "safety.h"
#include "power_monitor_hw.h"
#include "servo_output_hw.h"
#include "imu_hw.h"

const int MAX_LINE_LENGTH = 63;

const int MAX_ARGS = 3;

static char lineBuffer[MAX_LINE_LENGTH + 1];
static int lineLength = 0;

static bool lineTooLong = false;

static Print *out = &Serial;

static int splitWords(char *text, char *words[], int maxWords) {
    int count = 0;
    int i = 0;

    while (true) {
        while (text[i] == ' ' || text[i] == '\t') {
            i++;
        }
        if (text[i] == '\0') {
            break;
        }
        if (count == maxWords) {
            return -1;
        }

        words[count] = &text[i];
        count++;

        while (text[i] != '\0' && text[i] != ' ' && text[i] != '\t') {
            i++;
        }
        if (text[i] == '\0') {
            break;
        }

        text[i] = '\0';
        i++;
    }
    return count;
}

static void printHelp() {
    out->println("Commands:");
    out->println("  help         - show this list");
    out->println("  legs         - print leg points and joint limits");
    out->println("  stance       - IK angles of all legs in the standing pose");
    out->println("  up / down    - stand up / sit down (down while walking: stop, then sit)");
    out->println("  w s a d      - walk forward / back / left / right");
    out->println("  q e          - turn left / right");
    out->println("  x            - stop walking (gentle: slow down, feet down)");
    out->println("  stop         - EMERGENCY STOP: freeze everything now (RESET to continue)");
    out->println("  safety       - safety status (voltage, temperature, fault)");
    out->println("  safety fake <mV> <C> / safety real - test with fake readings / back to servos");
    out->println("  safety clear - forget the fault after fixing the cause");
    out->println("  power        - battery voltage, current, power and state (one line)");
    out->println("  servos       - servo output status (OFF after every boot)");
    out->println("  servos on    - read all servo positions, move slowly to the sitting pose, then follow");
    out->println("  servos off   - stop sending angles (servos hold their position; not while moving)");
    out->println("  imu          - roll, pitch, heading and calibration (one line)");
    out->println("  imu raw      - gravity in sensor and robot axes (for checking the axis mapping)");
    out->println("  imu save     - save the IMU calibration to flash (when fully calibrated, not moving)");
    out->println("  level [on|off] - body leveling with the IMU: status / on / off (OFF after boot)");
    out->println("  gait [name]  - show / set gait: tripod, ripple, wave (only when not moving)");
    out->println("  height [mm]  - show (with the range) / set standing height from the CAD pose (moves slowly)");
    out->println("Calibration (only while sitting, robot on a stand):");
    out->println("  center       - all servos to raw 500 (for mounting the horns)");
    out->println("  zero         - zero pose + trims");
    out->println("  trim L J D   - add D deg to leg L (A-F) joint J (1-3 or coxa/femur/tibia)");
    out->println("  dir L J      - flip the direction of one servo");
    out->println("  trims        - show all trims and directions");
    out->println("  save         - save the calibration to flash");
    out->println("Servo setup (only ONE servo connected, only while sitting):");
    out->println("  servo id           - read the ID of the connected servo (both buses)");
    out->println("  servo setid <new>  - give it a new ID (1..253), then read it back");
    out->println("  servo move <id> <deg> - move servo <id> slowly to <deg> (raw, -120..120)");
    out->println("  walk vx vy t - walk/turn, each -1..1 (+vx left, +vy back, +t turn left)");
}

static void printPoint(const char *label, Vec3 p) {
    out->print(label);
    out->print("(");
    out->print(p.x, 1);
    out->print(", ");
    out->print(p.y, 1);
    out->print(", ");
    out->print(p.z, 1);
    out->println(")");
}

static void printLimits(const char *label, JointLimits limits) {
    out->print(label);
    out->print(limits.minDeg, 0);
    out->print("..");
    out->print(limits.maxDeg, 0);
}

static void printLegs() {
    for (int i = 0; i < LEG_COUNT; i++) {
        const LegConfig &leg = LEGS[i];

        out->print("Leg ");
        out->print(leg.name);
        out->print(" (");
        out->print(leg.position);
        out->println(")");

        printPoint("  P1 coxa  : ", leg.p1);
        printPoint("  P2 femur : ", leg.p2);
        printPoint("  P3 tibia : ", leg.p3);
        printPoint("  P4 foot  : ", leg.p4);

        printLimits("  limits deg: coxa ", leg.coxa);
        printLimits(", femur ", leg.femur);
        printLimits(", tibia ", leg.tibia);
        out->println();
    }
}

static const char *ikStatusText(IkStatus status) {
    if (status == IK_OK) {
        return "OK";
    }
    if (status == IK_OUT_OF_REACH) {
        return "OUT_OF_REACH";
    }
    return "LIMIT_HIT";
}

static void printStance() {
    Body body = makeBody();
    Vec3 feet[LEG_COUNT];
    JointAngles angles[LEG_COUNT];
    IkStatus status[LEG_COUNT];

    for (int i = 0; i < LEG_COUNT; i++) {
        feet[i] = neutralFoot(body.legs[i]);
    }
    bool allOk = bodyIK(body, feet, angles, status);

    out->print("stance radius ");
    out->print(cadStanceRadius(), 1);
    out->println(" mm, feet on the floor (Z = 0)");
    for (int i = 0; i < LEG_COUNT; i++) {
        out->print("  ");
        out->print(body.legs[i].name);
        out->print(": coxa ");
        out->print(angles[i].coxa, 1);
        out->print("  femur ");
        out->print(angles[i].femur, 1);
        out->print("  tibia ");
        out->print(angles[i].tibia, 1);
        out->print("  ");
        out->println(ikStatusText(status[i]));
    }
    out->println(allOk ? "stance: all legs OK" : "stance: ERROR, some legs not OK");
}

static bool parseNumber(const char *text, float &value) {
    char *end;
    value = strtof(text, &end);
    return end != text && *end == '\0' && isfinite(value);
}

static void printReply(ControlReply r) {
    out->println(r.message);
}

static bool walkKey(const char *command, GaitCommand &out) {
    if (strcmp(command, "w") == 0) { out = makeCommand(0, -1, 0); return true; }
    if (strcmp(command, "s") == 0) { out = makeCommand(0, 1, 0); return true; }
    if (strcmp(command, "a") == 0) { out = makeCommand(1, 0, 0); return true; }
    if (strcmp(command, "d") == 0) { out = makeCommand(-1, 0, 0); return true; }
    if (strcmp(command, "q") == 0) { out = makeCommand(0, 0, 1); return true; }
    if (strcmp(command, "e") == 0) { out = makeCommand(0, 0, -1); return true; }
    return false;
}

static void runWalk(char *words[], int wordCount, Controller &ctrl) {
    GaitCommand walkCommand = makeCommand(0.0f, -1.0f, 0.0f);
    if (wordCount == 4) {
        float vx;
        float vy;
        float turn;
        if (!parseNumber(words[1], vx) || !parseNumber(words[2], vy) || !parseNumber(words[3], turn)) {
            out->println("error: walk needs numbers, e.g. walk 0 -1 0");
            return;
        }
        if (fabsf(vx) > 1.0f || fabsf(vy) > 1.0f || fabsf(turn) > 1.0f) {
            out->println("error: walk values must be -1..1");
            return;
        }
        walkCommand = makeCommand(vx, vy, turn);
    } else if (wordCount != 1) {
        out->println("error: use 'walk' or 'walk <vx> <vy> <turn>'");
        return;
    }

    ControlReply r = controllerWalk(ctrl, walkCommand);
    if (!r.accepted) {
        printReply(r);
        return;
    }

    GaitCommand t = ctrl.gait.target;
    char line[96];
    int n = snprintf(line, sizeof(line), "%s, vx %.2f vy %.2f turn %.2f\r\n",
                     r.message, t.vx, t.vy, t.turn);
    out->write((const uint8_t *)line, n);
}

static void runGait(char *words[], int wordCount, Controller &ctrl) {
    if (wordCount == 1) {
        out->print("gait: ");
        out->println(gaitName(ctrl.gait.type));
        return;
    }
    GaitType type;
    if (wordCount != 2 || !gaitFromName(words[1], type)) {
        out->println("error: use 'gait tripod', 'gait ripple' or 'gait wave'");
        return;
    }
    ControlReply r = controllerSetGait(ctrl, type);
    if (!r.accepted) {
        printReply(r);
        return;
    }
    out->print("gait: ");
    out->println(gaitName(ctrl.gait.type));
}

static void runHeight(char *words[], int wordCount, Controller &ctrl) {
    char line[96];
    int n;
    float mm;
    if (wordCount == 1) {
        n = snprintf(line, sizeof(line), "height: %.0f mm (range %.0f .. %.0f)\r\n", controllerHeightMm(ctrl),
                     STAND_HEIGHT_MIN_MM, STAND_HEIGHT_MAX_MM);
    } else if (wordCount == 2 && parseNumber(words[1], mm)) {
        ControlReply r = controllerSetHeight(ctrl, mm);
        if (r.accepted) {
            n = snprintf(line, sizeof(line), "%s (%.0f mm)\r\n", r.message, controllerHeightMm(ctrl));
        } else {
            n = snprintf(line, sizeof(line), "%s\r\n", r.message);
        }
    } else {
        n = snprintf(line, sizeof(line), "error: use 'height' or 'height <mm>'\r\n");
    }
    out->write((const uint8_t *)line, n);
}

static bool isLongPrintCommand(const char *command) {
    return strcmp(command, "help") == 0
        || strcmp(command, "legs") == 0
        || strcmp(command, "stance") == 0
        || strcmp(command, "trims") == 0;
}

static void sendJointZero(const Calibration &calib, int leg, int joint, int timeMs) {
    servoMoveTime(SERVO_BUS_OF_LEG[leg], SERVO_ID[leg][joint],
                  calibratedServoAngle(calib, leg, joint, 0.0f), timeMs);
}

static void runCenter() {
    uint8_t packet[SERVO_MAX_PACKET];
    int length = servoBuildMoveTime(SERVO_BROADCAST_ID, SERVO_POSITION_ZERO, CALIBRATION_MOVE_MS, packet);
    servoBusSend(1, packet, length);
    servoBusSend(2, packet, length);
    out->println("center: all servos -> raw 500 (no trims)");
}

static void runZero(const Calibration &calib) {
    for (int leg = 0; leg < LEG_COUNT; leg++) {
        for (int joint = 0; joint < JOINT_COUNT; joint++) {
            sendJointZero(calib, leg, joint, CALIBRATION_MOVE_MS);
        }
    }
    out->println("zero: zero pose + trims sent to 18 servos");
}

static void printJoint(const char *what, const Calibration &calib, int leg, int joint) {
    char line[96];
    int n = snprintf(line, sizeof(line), "%s %s %s: %+.1f deg, dir %+d, id %d -> position %d\r\n",
                     what, LEGS[leg].name, jointName(joint), calib.trimDeg[leg][joint],
                     calib.direction[leg][joint], SERVO_ID[leg][joint],
                     calibratedServoPosition(calib, leg, joint, 0.0f));
    out->write((const uint8_t *)line, n);
}

static void runTrimOrDir(char *words[], int wordCount, Calibration &calib, bool isTrim) {
    int leg;
    int joint;
    float delta = 0.0f;
    bool ok = (isTrim ? wordCount == 4 : wordCount == 3)
           && parseLeg(words[1], leg) && parseJoint(words[2], joint)
           && (!isTrim || parseNumber(words[3], delta));
    if (!ok) {
        out->println(isTrim ? "error: use 'trim <A-F> <1-3> <deg>', e.g. trim A 2 +1.5"
                              : "error: use 'dir <A-F> <1-3>', e.g. dir D 1");
        return;
    }
    if (isTrim) {
        calibrationAddTrim(calib, leg, joint, delta);
    } else {
        calibrationFlipDirection(calib, leg, joint);
    }
    sendJointZero(calib, leg, joint, TRIM_MOVE_MS);
    printJoint(isTrim ? "trim" : "dir", calib, leg, joint);
}

static void printTrims(const Calibration &calib) {
    out->println("leg  coxa (dir)     femur (dir)    tibia (dir)   [deg]");
    for (int leg = 0; leg < LEG_COUNT; leg++) {
        char line[80];
        int n = snprintf(line, sizeof(line), "%s    %+5.1f (%+d)    %+5.1f (%+d)    %+5.1f (%+d)\r\n",
                         LEGS[leg].name,
                         calib.trimDeg[leg][0], calib.direction[leg][0],
                         calib.trimDeg[leg][1], calib.direction[leg][1],
                         calib.trimDeg[leg][2], calib.direction[leg][2]);
        out->write((const uint8_t *)line, n);
    }
}

static bool parseWholeNumber(const char *text, int minValue, int maxValue, int &value) {
    float f;
    if (!parseNumber(text, f) || f != floorf(f) || f < minValue || f > maxValue) {
        return false;
    }
    value = (int)f;
    return true;
}

static ServoReplyStatus readIdOnBus(int bus, uint8_t &id, unsigned long timeoutMs) {
    uint8_t request[SERVO_MAX_PACKET];
    uint8_t reply[SERVO_MAX_PACKET];
    int length = servoBuildIdRead(SERVO_BROADCAST_ID, request);
    ServoReplyStatus status = servoBusRequest(bus, request, length, reply, timeoutMs);
    if (status == SERVO_REPLY_OK && !servoReplyId(reply, id)) {
        status = SERVO_NO_REPLY;
    }
    return status;
}

static int findSingleServo(uint8_t &id) {
    int foundBus = 0;
    int answers = 0;
    for (int bus = 1; bus <= 2; bus++) {
        uint8_t busId = 0;
        ServoReplyStatus status = readIdOnBus(bus, busId, SERVO_REPLY_TIMEOUT_MS);
        char line[80];
        int n;
        if (status == SERVO_REPLY_OK) {
            n = snprintf(line, sizeof(line), "servo: bus %d: ID %d\r\n", bus, busId);
            answers++;
            foundBus = bus;
            id = busId;
        } else {
            n = snprintf(line, sizeof(line), "servo: bus %d: %s\r\n", bus, servoReplyStatusText(status));
        }
        out->write((const uint8_t *)line, n);
    }
    if (answers > 1) {
        out->println("error: servos answer on both buses, connect only ONE servo");
        return 0;
    }
    return foundBus;
}

static void runServo(char *words[], int wordCount) {
    if (wordCount == 2 && strcmp(words[1], "id") == 0) {
        uint8_t id;
        if (findSingleServo(id) == 0) {
            out->println("servo: no single servo found");
        }
        return;
    }

    int newId;
    if (wordCount == 3 && strcmp(words[1], "setid") == 0) {
        if (!parseWholeNumber(words[2], 1, SERVO_ID_MAX, newId)) {
            out->println("error: new ID must be a whole number 1..253");
            return;
        }
        uint8_t oldId;
        int bus = findSingleServo(oldId);
        if (bus == 0) {
            out->println("error: no single servo found, ID not changed");
            return;
        }
        uint8_t packet[SERVO_MAX_PACKET];
        int length = servoBuildIdWrite(oldId, (uint8_t)newId, packet);
        servoBusSend(bus, packet, length);

        for (int attempt = 0; attempt < 3; attempt++) {
            uint8_t check;
            if (readIdOnBus(bus, check, SERVO_ID_VERIFY_TIMEOUT_MS) == SERVO_REPLY_OK && check == newId) {
                char line[80];
                int n = snprintf(line, sizeof(line), "servo: ID %d -> %d on bus %d, verified\r\n",
                                 oldId, newId, bus);
                out->write((const uint8_t *)line, n);
                return;
            }
        }
        out->println("error: could not read the new ID back, check the servo");
        return;
    }

    int id;
    float angle;
    if (wordCount == 4 && strcmp(words[1], "move") == 0) {
        if (!parseWholeNumber(words[2], 0, SERVO_ID_MAX, id) || !parseNumber(words[3], angle)
            || fabsf(angle) > 120.0f) {
            out->println("error: use 'servo move <id 0..253> <deg -120..120>'");
            return;
        }
        servoMoveTime(1, (uint8_t)id, angle, SERVO_TEST_MOVE_MS);
        servoMoveTime(2, (uint8_t)id, angle, SERVO_TEST_MOVE_MS);
        char line[96];
        int n = snprintf(line, sizeof(line), "servo: id %d -> %+.1f deg (position %d) in %d ms\r\n",
                         id, angle, servoAngleToPosition(angle), SERVO_TEST_MOVE_MS);
        out->write((const uint8_t *)line, n);
        return;
    }

    out->println("error: use 'servo id', 'servo setid <new>' or 'servo move <id> <deg>'");
}

static void runEmergencyStop(Controller &ctrl) {
    servoOutputOff("emergency stop");
    uint8_t packet[SERVO_MAX_PACKET];
    int length = servoBuildMoveStop(SERVO_BROADCAST_ID, packet);
    servoBusSend(1, packet, length);
    servoBusSend(2, packet, length);
    printReply(controllerEmergencyStop(ctrl));
}

static void printSafetyStatus(const Safety &safety) {
    int last = (safety.nextServo + SAFETY_SERVO_COUNT - 1) % SAFETY_SERVO_COUNT;
    char line[160];
    int n = snprintf(line, sizeof(line),
                     "safety: %s, fault %s, last servo %s %s: %s %.2f V %d C%s\r\n",
                     safety.fake ? "FAKE" : "real", safetyFaultText(safety.fault),
                     LEGS[last / 3].name, jointName(last % 3),
                     safety.answered[last] ? "answered" : "no answer",
                     safety.voltageMv[last] / 1000.0f, safety.tempC[last],
                     safety.noPower ? " | servos do not answer (no servo power?)" : "");
    out->write((const uint8_t *)line, n);
}

static void runSafety(char *words[], int wordCount, Safety &safety, Controller &ctrl) {
    if (wordCount == 1) {
        printSafetyStatus(safety);
        return;
    }
    int mv;
    int celsius;
    if (wordCount == 4 && strcmp(words[1], "fake") == 0) {
        if (!parseWholeNumber(words[2], 0, 20000, mv) || !parseWholeNumber(words[3], -40, 200, celsius)) {
            out->println("error: use 'safety fake <mV 0..20000> <C -40..200>'");
            return;
        }
        safety.fake = true;
        safety.fakeVoltageMv = mv;
        safety.fakeTempC = celsius;
        out->println("safety: FAKE readings on (every check uses these values)");
        return;
    }
    if (wordCount == 2 && strcmp(words[1], "real") == 0) {
        safety.fake = false;
        out->println("safety: real readings from the servos");
        return;
    }
    if (wordCount == 2 && strcmp(words[1], "clear") == 0) {
        safetyClear(safety);
        controllerClearSafety(ctrl);
        out->println("safety: fault cleared (it comes back if the cause is still there)");
        return;
    }
    out->println("error: use 'safety', 'safety fake <mV> <C>', 'safety real' or 'safety clear'");
}

static void printPower() {
    PowerReading p = powerLatest();
    char line[96];
    int n;
    if (!p.valid) {
        n = snprintf(line, sizeof(line), "power: no reading yet\r\n");
    } else if (p.fromIna226) {
        n = snprintf(line, sizeof(line), "power: %.2f V  %.3f A  %.2f W  %s (INA226)\r\n",
                     p.voltageMv / 1000.0f, p.currentMa / 1000.0f, p.powerMw / 1000.0f,
                     batteryStateName(p.state));
    } else {
        n = snprintf(line, sizeof(line), "power: %.2f V  -- A  -- W  %s (ADC backup)\r\n",
                     p.voltageMv / 1000.0f, batteryStateName(p.state));
    }
    out->write((const uint8_t *)line, n);
}

static void runImu(char *words[], int wordCount, const Controller &ctrl) {
    ImuReading imu = imuLatest();
    char line[160];
    int n;
    if (wordCount == 2 && strcmp(words[1], "save") == 0) {
        if (controllerIsMoving(ctrl)) {
            out->println("error: robot is moving (the IMU stops for 30 ms while saving)");
            return;
        }
        const char *problem = imuSaveCalibration();
        if (problem == NULL) {
            n = snprintf(line, sizeof(line), "imu: calibration saved to flash\r\n");
        } else {
            n = snprintf(line, sizeof(line), "error: %s\r\n", problem);
        }
    } else if (wordCount > 2 || (wordCount == 2 && strcmp(words[1], "raw") != 0)) {
        n = snprintf(line, sizeof(line), "error: use 'imu', 'imu raw' or 'imu save'\r\n");
    } else if (!imuPresent()) {
        n = snprintf(line, sizeof(line), "imu: no BNO055 (not found at startup or stopped answering)\r\n");
    } else if (!imu.valid) {
        n = snprintf(line, sizeof(line), "imu: no reading yet\r\n");
    } else if (wordCount == 2) {
        Vec3 g = imu.gravitySensor;
        Vec3 r = imuSensorToRobot(g, IMU_AXIS_MAP);
        n = snprintf(line, sizeof(line), "imu raw: gravity sensor x %.2f y %.2f z %.2f | robot x %.2f y %.2f z %.2f m/s^2\r\n",
                     g.x, g.y, g.z, r.x, r.y, r.z);
    } else if (imu.tiltValid) {
        n = snprintf(line, sizeof(line), "imu: roll %.1f pitch %.1f heading %.1f | calib sys %d gyro %d accel %d mag %d\r\n",
                     imu.rollDeg, imu.pitchDeg, imu.headingDeg,
                     imu.calib.sys, imu.calib.gyro, imu.calib.accel, imu.calib.mag);
    } else {
        n = snprintf(line, sizeof(line), "imu: roll -- pitch -- heading %.1f | calib sys %d gyro %d accel %d mag %d\r\n",
                     imu.headingDeg, imu.calib.sys, imu.calib.gyro, imu.calib.accel, imu.calib.mag);
    }
    out->write((const uint8_t *)line, n);
}

static void runLevel(char *words[], int wordCount, Leveling &leveling) {
    char line[160];
    int n;
    if (wordCount == 2 && strcmp(words[1], "on") == 0) {
        if (!imuPresent()) {
            n = snprintf(line, sizeof(line), "error: no IMU (BNO055), leveling needs it\r\n");
        } else {
            leveling.enabled = true;
            n = snprintf(line, sizeof(line), "level: ON (while standing and walking, up to %.0f deg)\r\n",
                         LEVEL_MAX_DEG);
        }
    } else if (wordCount == 2 && strcmp(words[1], "off") == 0) {
        leveling.enabled = false;
        n = snprintf(line, sizeof(line), "level: OFF (the correction goes back to 0 slowly)\r\n");
    } else if (wordCount == 1) {
        n = snprintf(line, sizeof(line), "level: %s, correction roll %.1f pitch %.1f%s",
                     leveling.enabled ? "ON" : "OFF", leveling.correctionRollDeg, leveling.correctionPitchDeg,
                     leveling.limited ? " (limited by leg reach)" : "");
        ImuReading imu = imuLatest();
        if (imuPresent() && imu.valid && imu.tiltValid) {
            n += snprintf(line + n, sizeof(line) - n, " | body tilt roll %.1f pitch %.1f\r\n",
                          imu.rollDeg, imu.pitchDeg);
        } else {
            n += snprintf(line + n, sizeof(line) - n, " | no IMU data\r\n");
        }
    } else {
        n = snprintf(line, sizeof(line), "error: use 'level', 'level on' or 'level off'\r\n");
    }
    out->write((const uint8_t *)line, n);
}

static void runServos(char *words[], int wordCount, const Controller &ctrl, const Safety &safety) {
    if (wordCount == 1) {
        servoOutputPrintStatus();
        return;
    }
    if (wordCount == 2 && strcmp(words[1], "on") == 0) {
        servoOutputOn(ctrl, safety);
        return;
    }
    if (wordCount == 2 && strcmp(words[1], "off") == 0) {
        if (controllerIsMoving(ctrl)) {
            out->println("error: robot is moving (x or down first, or stop for an emergency)");
            return;
        }
        if (!servoOutputOff("by command")) {
            out->println("servos: OFF already");
        }
        return;
    }
    out->println("error: use 'servos', 'servos on' or 'servos off'");
}

static bool isServoCalibrationCommand(const char *command) {
    return strcmp(command, "center") == 0
        || strcmp(command, "zero") == 0
        || strcmp(command, "trim") == 0
        || strcmp(command, "dir") == 0
        || strcmp(command, "servo") == 0;
}

static void runLine(char *line, Controller &ctrl, Calibration &calib, Safety &safety, Leveling &leveling) {
    char *words[1 + MAX_ARGS];
    int wordCount = splitWords(line, words, 1 + MAX_ARGS);

    if (wordCount == 0) {
        return;
    }
    if (wordCount < 0) {
        out->print("error: too many arguments (max ");
        out->print(MAX_ARGS);
        out->println(")");
        return;
    }

    char *command = words[0];
    GaitCommand keyCommand;

    if (controllerIsMoving(ctrl) && isLongPrintCommand(command)) {
        out->println("error: busy, stop first");
        return;
    }

    if (strcmp(command, "help") == 0) {
        printHelp();
    } else if (strcmp(command, "legs") == 0) {
        printLegs();
    } else if (strcmp(command, "stance") == 0) {
        printStance();
    } else if (strcmp(command, "up") == 0) {
        if (servoOutputMode() == SERVO_OUTPUT_READING || servoOutputMode() == SERVO_OUTPUT_STARTING) {
            out->println("error: servos are starting, wait until they are ON");
        } else {
            printReply(controllerUp(ctrl));
        }
    } else if (strcmp(command, "down") == 0) {
        printReply(controllerDown(ctrl));
    } else if (strcmp(command, "stop") == 0) {
        runEmergencyStop(ctrl);
    } else if (strcmp(command, "x") == 0) {
        printReply(controllerStop(ctrl));
    } else if (strcmp(command, "safety") == 0) {
        runSafety(words, wordCount, safety, ctrl);
    } else if (strcmp(command, "power") == 0) {
        printPower();
    } else if (strcmp(command, "servos") == 0) {
        runServos(words, wordCount, ctrl, safety);
    } else if (strcmp(command, "imu") == 0) {
        runImu(words, wordCount, ctrl);
    } else if (strcmp(command, "level") == 0) {
        runLevel(words, wordCount, leveling);
    } else if (walkKey(command, keyCommand)) {
        printReply(controllerWalk(ctrl, keyCommand));
    } else if (strcmp(command, "walk") == 0) {
        runWalk(words, wordCount, ctrl);
    } else if (strcmp(command, "gait") == 0) {
        runGait(words, wordCount, ctrl);
    } else if (strcmp(command, "height") == 0) {
        runHeight(words, wordCount, ctrl);
    } else if (isServoCalibrationCommand(command) && servoOutputMode() != SERVO_OUTPUT_OFF) {
        out->println("error: servo output is on, type 'servos off' first");
    } else if (isServoCalibrationCommand(command) && ctrl.state != STATE_SIT) {
        out->println("error: only while sitting (put the robot on a stand)");
    } else if (strcmp(command, "center") == 0) {
        runCenter();
    } else if (strcmp(command, "zero") == 0) {
        runZero(calib);
    } else if (strcmp(command, "trim") == 0) {
        runTrimOrDir(words, wordCount, calib, true);
    } else if (strcmp(command, "dir") == 0) {
        runTrimOrDir(words, wordCount, calib, false);
    } else if (strcmp(command, "servo") == 0) {
        runServo(words, wordCount);
    } else if (strcmp(command, "trims") == 0) {
        printTrims(calib);
    } else if (strcmp(command, "save") == 0) {
        out->println(calibrationSave(calib) ? "save: calibration saved to flash"
                                              : "error: saving to flash failed");
    } else {
        out->print("error: unknown command '");
        out->print(command);
        out->println("', type help");
    }
}

void commandsUpdate(Controller &ctrl, Calibration &calib, Safety &safety, Leveling &leveling) {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();

        if (c == '\n' || c == '\r') {
            if (lineTooLong) {
                out->print("error: line too long (max ");
                out->print(MAX_LINE_LENGTH);
                out->println(" chars), ignored");
            } else {
                lineBuffer[lineLength] = '\0';
                runLine(lineBuffer, ctrl, calib, safety, leveling);
            }
            lineLength = 0;
            lineTooLong = false;
        } else if (lineLength < MAX_LINE_LENGTH) {
            lineBuffer[lineLength] = c;
            lineLength++;
        } else {
            lineTooLong = true;
        }
    }
}

void commandsRunLine(const char *line, Print &reply, Controller &ctrl, Calibration &calib,
                     Safety &safety, Leveling &leveling) {
    out = &reply;
    if (strlen(line) > (size_t)MAX_LINE_LENGTH) {
        out->println("error: line too long, ignored");
    } else {
        char copy[MAX_LINE_LENGTH + 1];
        strcpy(copy, line);
        runLine(copy, ctrl, calib, safety, leveling);
    }
    out = &Serial;
}
