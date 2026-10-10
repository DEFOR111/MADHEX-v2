#pragma once

#include <stdint.h>
#include "calibration.h"
#include "leg.h"
#include "safety.h"
#include "power_monitor.h"
#include "servo_packet.h"

const int SERVO_COUNT = LEG_COUNT * JOINT_COUNT;

const int SERVO_FRAME_MAX_BYTES = SERVO_COUNT * SERVO_MAX_PACKET;

enum ServoOutputMode {
    SERVO_OUTPUT_OFF,
    SERVO_OUTPUT_READING,
    SERVO_OUTPUT_STARTING,
    SERVO_OUTPUT_ON
};

const char *servoOutputModeName(ServoOutputMode mode);

void servoTargetPositions(const Calibration &calib, const JointAngles angles[], int positions[]);

int servoBuildFrame(int bus, const int positions[], int timeMs, uint8_t out[]);

void servoStartupPositions(const int from[], const int to[], float t, float duration, int out[]);

unsigned long servoWireTimeUs(int bytes, unsigned long baud);

bool servoBusWindowFree(unsigned long sinceFrameMs, unsigned long frameWireMs,
                        unsigned long requestTimeoutMs, unsigned long periodMs);

const char *servoOnRefusal(ServoOutputMode mode, RobotState state, SafetyFault fault,
                           bool noServoPower, bool powerValid, BatteryState battery);

const char *servoForceOffReason(RobotState state, SafetyFault fault, bool noServoPower,
                                bool powerValid, BatteryState battery);
