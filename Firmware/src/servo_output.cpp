#include <math.h>
#include <stddef.h>
#include "config.h"
#include "mathx.h"
#include "servo_output.h"

const char *servoOutputModeName(ServoOutputMode mode) {
    if (mode == SERVO_OUTPUT_READING) {
        return "READING";
    }
    if (mode == SERVO_OUTPUT_STARTING) {
        return "STARTING";
    }
    if (mode == SERVO_OUTPUT_ON) {
        return "ON";
    }
    return "OFF";
}

void servoTargetPositions(const Calibration &calib, const JointAngles angles[], int positions[]) {
    for (int leg = 0; leg < LEG_COUNT; leg++) {
        positions[3 * leg + JOINT_COXA] = calibratedServoPosition(calib, leg, JOINT_COXA, angles[leg].coxa);
        positions[3 * leg + JOINT_FEMUR] = calibratedServoPosition(calib, leg, JOINT_FEMUR, angles[leg].femur);
        positions[3 * leg + JOINT_TIBIA] = calibratedServoPosition(calib, leg, JOINT_TIBIA, angles[leg].tibia);
    }
}

int servoBuildFrame(int bus, const int positions[], int timeMs, uint8_t out[]) {
    int length = 0;
    for (int leg = 0; leg < LEG_COUNT; leg++) {
        if (SERVO_BUS_OF_LEG[leg] != bus) {
            continue;
        }
        for (int joint = 0; joint < JOINT_COUNT; joint++) {
            length += servoBuildMoveTime(SERVO_ID[leg][joint], positions[3 * leg + joint], timeMs,
                                         out + length);
        }
    }
    return length;
}

void servoStartupPositions(const int from[], const int to[], float t, float duration, int out[]) {
    float ease = 1.0f;
    if (t <= 0.0f) {
        ease = 0.0f;
    } else if (t < duration) {
        ease = 0.5f - 0.5f * cosf(MATHX_PI * t / duration);
    }
    for (int i = 0; i < SERVO_COUNT; i++) {
        out[i] = (int)lroundf(from[i] + (to[i] - from[i]) * ease);
    }
}

unsigned long servoWireTimeUs(int bytes, unsigned long baud) {
    return (unsigned long)bytes * 10UL * 1000000UL / baud;
}

bool servoBusWindowFree(unsigned long sinceFrameMs, unsigned long frameWireMs,
                        unsigned long requestTimeoutMs, unsigned long periodMs) {
    return sinceFrameMs > frameWireMs && sinceFrameMs + requestTimeoutMs < periodMs;
}

const char *servoOnRefusal(ServoOutputMode mode, RobotState state, SafetyFault fault,
                           bool noServoPower, bool powerValid, BatteryState battery) {
    if (mode == SERVO_OUTPUT_ON) {
        return "servos are on already";
    }
    if (mode != SERVO_OUTPUT_OFF) {
        return "servos are starting already";
    }
    if (state != STATE_SIT) {
        return "only while sitting (the start pose is the sitting pose)";
    }
    if (fault != SAFETY_FAULT_NONE) {
        return "safety fault, fix the cause and type 'safety clear' first";
    }
    if (!powerValid) {
        return "no battery reading yet, try again";
    }
    if (battery == BATTERY_NOT_CONNECTED) {
        return "battery not connected (servo power is 0 V)";
    }
    if (noServoPower) {
        return "servos do not answer the safety checks (no servo power?)";
    }
    return NULL;
}

const char *servoForceOffReason(RobotState state, SafetyFault fault, bool noServoPower,
                                bool powerValid, BatteryState battery) {
    if (state == STATE_ESTOP) {
        return "emergency stop";
    }
    if (powerValid && battery == BATTERY_NOT_CONNECTED) {
        return "battery not connected";
    }
    if (noServoPower) {
        return "servos do not answer";
    }
    if (fault != SAFETY_FAULT_NONE && state == STATE_SIT) {
        return "safety fault, robot is sitting";
    }
    return NULL;
}
