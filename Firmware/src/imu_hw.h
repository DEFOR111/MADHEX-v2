#pragma once

#include "imu.h"

struct ImuReading {
    bool valid;
    bool tiltValid;
    float rollDeg;
    float pitchDeg;
    float headingDeg;
    ImuCalibration calib;
    Vec3 gravitySensor;
};

void imuBegin();

void imuUpdate();

bool imuPresent();

ImuReading imuLatest();

const char *imuSaveCalibration();
