#pragma once

#include "config.h"

struct LegLengths {
    float coxa;
    float femur;
    float tibia;
};

struct Leg {
    const char *name;
    Vec3 mount;

    float directionDeg;

    float coxaLength;
    float femurLength;
    float tibiaLength;

    float femurZeroDeg;
    float tibiaZeroDeg;
    float kneeZeroDeg;

    JointLimits coxaLimits;
    JointLimits femurLimits;
    JointLimits tibiaLimits;
};

LegLengths measureLegLengths(const LegConfig &config);

float maxLengthDifferenceMm(const LegConfig &config, const LegConfig &reference);

Leg makeLeg(const LegConfig &config);

Vec3 bodyToLeg(const Leg &leg, Vec3 pointBody);

Vec3 legToBody(const Leg &leg, Vec3 pointLeg);

struct JointAngles {
    float coxa;
    float femur;
    float tibia;
};

enum IkStatus {
    IK_OK,
    IK_OUT_OF_REACH,
    IK_LIMIT_HIT
};

IkStatus legIK(const Leg &leg, Vec3 footBody, JointAngles &angles);

Vec3 legFK(const Leg &leg, JointAngles angles);
