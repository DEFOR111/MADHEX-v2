#pragma once

#include "servo_output.h"

void servoOutputOn(const Controller &ctrl, const Safety &safety);

bool servoOutputOff(const char *reason);

void servoOutputLoop(bool busFree);

void servoOutputUpdate(const Controller &ctrl, const Safety &safety, const Calibration &calib,
                       const JointAngles angles[]);

bool servoOutputBusFree(unsigned long requestTimeoutMs);

ServoOutputMode servoOutputMode();

void servoOutputPrintStatus();
