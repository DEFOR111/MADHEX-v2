#pragma once

#include <stdint.h>
#include "config.h"

const int JOINT_COUNT = 3;
const int JOINT_COXA = 0;
const int JOINT_FEMUR = 1;
const int JOINT_TIBIA = 2;

struct Calibration {
    float trimDeg[LEG_COUNT][JOINT_COUNT];
    int8_t direction[LEG_COUNT][JOINT_COUNT];
};

Calibration makeCalibration();

bool calibrationIsValid(const Calibration &c);

float calibrationAddTrim(Calibration &c, int leg, int joint, float deltaDeg);

int calibrationFlipDirection(Calibration &c, int leg, int joint);

float calibratedServoAngle(const Calibration &c, int leg, int joint, float jointAngleDeg);

int calibratedServoPosition(const Calibration &c, int leg, int joint, float jointAngleDeg);

bool parseLeg(const char *text, int &leg);

bool parseJoint(const char *text, int &joint);

const char *jointName(int joint);
