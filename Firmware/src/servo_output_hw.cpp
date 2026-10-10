#include <Arduino.h>
#include "config.h"
#include "servo_bus.h"
#include "servo_output_hw.h"
#include "power_monitor_hw.h"

static ServoOutputMode mode = SERVO_OUTPUT_OFF;

static int readIndex = 0;
static ServoPendingRequest readRequest;
static ServoReplyStatus readStatus[SERVO_COUNT];
static int startPosition[SERVO_COUNT];

static int startupFrames = 0;

static unsigned long lastFrameMs = 0;
static unsigned long frameWireMs = 0;
static unsigned long lastFrameCpuUs = 0;
static unsigned long maxFrameCpuUs = 0;
static bool measureWire = false;
static unsigned long measuredWireUs = 0;

static void printLine(const char *line, int n) {
    Serial.write((const uint8_t *)line, n);
}

void servoOutputOn(const Controller &ctrl, const Safety &safety) {
    PowerReading power = powerLatest();
    const char *refusal = servoOnRefusal(mode, ctrl.state, safety.fault, safety.noPower,
                                         power.valid, power.state);
    char line[120];
    if (refusal != NULL) {
        printLine(line, snprintf(line, sizeof(line), "error: %s\r\n", refusal));
        return;
    }
    mode = SERVO_OUTPUT_READING;
    readIndex = 0;
    readRequest.active = false;
    printLine(line, snprintf(line, sizeof(line), "servos: reading the positions of all %d servos...\r\n",
                             SERVO_COUNT));
}

bool servoOutputOff(const char *reason) {
    if (mode == SERVO_OUTPUT_OFF) {
        return false;
    }
    mode = SERVO_OUTPUT_OFF;
    char line[120];
    printLine(line, snprintf(line, sizeof(line), "servos: OFF (%s)\r\n", reason));
    return true;
}

static const char *shortStatus(ServoReplyStatus status) {
    if (status == SERVO_NO_ECHO) {
        return "no echo";
    }
    if (status == SERVO_ECHO_WRONG) {
        return "echo wrong";
    }
    return "no reply";
}

static void finishReading() {
    int missing = 0;
    int firstMissing = -1;
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (readStatus[i] != SERVO_REPLY_OK) {
            missing++;
            if (firstMissing < 0) {
                firstMissing = i;
            }
        }
    }

    char line[640];
    int n;
    if (missing == 0) {
        mode = SERVO_OUTPUT_STARTING;
        startupFrames = 0;
        maxFrameCpuUs = 0;
        measureWire = true;
        n = snprintf(line, sizeof(line), "servos: all %d answered, moving slowly to the sitting pose (%.1f s)\r\n",
                     SERVO_COUNT, SERVO_STARTUP_MOVE_S);
    } else if (missing == SERVO_COUNT) {
        mode = SERVO_OUTPUT_OFF;
        n = snprintf(line, sizeof(line),
                     "servos: no servo answers (%s) -> output stays OFF, robot stays in SIT\r\n",
                     servoReplyStatusText(readStatus[firstMissing]));
    } else {
        mode = SERVO_OUTPUT_OFF;
        n = snprintf(line, sizeof(line), "servos: %d of %d do not answer:", missing, SERVO_COUNT);
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (readStatus[i] != SERVO_REPLY_OK) {
                n += snprintf(line + n, sizeof(line) - n, "%s %s %s (ID %d) %s", i == firstMissing ? "" : ",",
                              LEGS[i / 3].name, jointName(i % 3), SERVO_ID[i / 3][i % 3],
                              shortStatus(readStatus[i]));
            }
        }
        n += snprintf(line + n, sizeof(line) - n, " -> output stays OFF, robot stays in SIT\r\n");
    }
    printLine(line, n);
}

void servoOutputLoop(bool busFree) {
    if (mode != SERVO_OUTPUT_READING) {
        return;
    }
    if (!readRequest.active) {
        if (!busFree) {
            return;
        }
        int leg = readIndex / 3;
        int joint = readIndex % 3;
        uint8_t request[SERVO_MAX_PACKET];
        int length = servoBuildRead(SERVO_ID[leg][joint], SERVO_POS_READ, request);
        servoRequestStart(readRequest, SERVO_BUS_OF_LEG[leg], request, length);
        return;
    }

    ServoReplyStatus status = servoRequestPoll(readRequest, SERVO_REPLY_TIMEOUT_MS);
    if (status == SERVO_PENDING) {
        return;
    }
    int position = SERVO_POSITION_ZERO;
    if (status == SERVO_REPLY_OK && !servoReplyPosition(readRequest.receiver.packet, position)) {
        status = SERVO_NO_REPLY;
    }
    readStatus[readIndex] = status;
    startPosition[readIndex] = constrain(position, SERVO_POSITION_MIN, SERVO_POSITION_MAX);
    readIndex++;
    if (readIndex == SERVO_COUNT) {
        finishReading();
    }
}

static void sendFrame(const int positions[]) {
    uint8_t frame1[SERVO_FRAME_MAX_BYTES];
    uint8_t frame2[SERVO_FRAME_MAX_BYTES];
    unsigned long startUs = micros();
    int length1 = servoBuildFrame(1, positions, UPDATE_PERIOD_MS, frame1);
    int length2 = servoBuildFrame(2, positions, UPDATE_PERIOD_MS, frame2);
    servoBusSend(1, frame1, length1);
    servoBusSend(2, frame2, length2);
    lastFrameCpuUs = micros() - startUs;
    if (lastFrameCpuUs > maxFrameCpuUs) {
        maxFrameCpuUs = lastFrameCpuUs;
    }
    lastFrameMs = millis();
    int longest = (length1 > length2) ? length1 : length2;
    frameWireMs = (servoWireTimeUs(longest, SERVO_BAUD) + 999) / 1000;

    if (measureWire) {
        servoBusWaitSent(1);
        servoBusWaitSent(2);
        measuredWireUs = micros() - startUs;
        measureWire = false;
        char line[200];
        int n = snprintf(line, sizeof(line),
                         "servos: frame = %d packets (bus 1: %d bytes, bus 2: %d bytes): CPU %lu us, "
                         "on the wire %.2f ms (calculated %.2f ms), update period %lu ms -> %s\r\n",
                         (length1 + length2) / SERVO_MAX_PACKET, length1, length2, lastFrameCpuUs,
                         measuredWireUs / 1000.0f, servoWireTimeUs(longest, SERVO_BAUD) / 1000.0f,
                         UPDATE_PERIOD_MS, measuredWireUs < UPDATE_PERIOD_MS * 1000UL ? "fits" : "TOO SLOW");
        printLine(line, n);
    }
}

void servoOutputUpdate(const Controller &ctrl, const Safety &safety, const Calibration &calib,
                       const JointAngles angles[]) {
    if (mode == SERVO_OUTPUT_OFF) {
        return;
    }
    PowerReading power = powerLatest();
    const char *reason = servoForceOffReason(ctrl.state, safety.fault, safety.noPower,
                                             power.valid, power.state);
    if (reason != NULL) {
        servoOutputOff(reason);
        return;
    }
    if (mode == SERVO_OUTPUT_READING) {
        return;
    }

    int target[SERVO_COUNT];
    servoTargetPositions(calib, angles, target);
    if (mode == SERVO_OUTPUT_ON) {
        sendFrame(target);
        return;
    }

    float t = startupFrames * UPDATE_PERIOD_MS / 1000.0f;
    int positions[SERVO_COUNT];
    servoStartupPositions(startPosition, target, t, SERVO_STARTUP_MOVE_S, positions);
    sendFrame(positions);
    startupFrames++;
    if (t >= SERVO_STARTUP_MOVE_S) {
        mode = SERVO_OUTPUT_ON;
        Serial.println("servos: ON (sitting pose reached), the angles of every update are sent now");
    }
}

bool servoOutputBusFree(unsigned long requestTimeoutMs) {
    if (mode == SERVO_OUTPUT_OFF) {
        return true;
    }
    if (mode == SERVO_OUTPUT_READING) {
        return false;
    }
    return servoBusWindowFree(millis() - lastFrameMs, frameWireMs, requestTimeoutMs, UPDATE_PERIOD_MS);
}

ServoOutputMode servoOutputMode() {
    return mode;
}

void servoOutputPrintStatus() {
    char line[160];
    int n;
    if (mode == SERVO_OUTPUT_READING) {
        n = snprintf(line, sizeof(line), "servos: READING positions (%d of %d)\r\n", readIndex, SERVO_COUNT);
    } else if (mode == SERVO_OUTPUT_STARTING) {
        n = snprintf(line, sizeof(line), "servos: STARTING, moving to the sitting pose (%.1f of %.1f s)\r\n",
                     startupFrames * UPDATE_PERIOD_MS / 1000.0f, SERVO_STARTUP_MOVE_S);
    } else if (mode == SERVO_OUTPUT_ON) {
        n = snprintf(line, sizeof(line),
                     "servos: ON, frame CPU %lu us (max %lu us), first frame on the wire %.2f ms, period %lu ms\r\n",
                     lastFrameCpuUs, maxFrameCpuUs, measuredWireUs / 1000.0f, UPDATE_PERIOD_MS);
    } else {
        n = snprintf(line, sizeof(line), "servos: OFF (type 'servos on' to drive the servos)\r\n");
    }
    printLine(line, n);
}
