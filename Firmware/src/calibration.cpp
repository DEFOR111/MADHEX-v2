#include <ctype.h>
#include <math.h>
#include <string.h>
#include "calibration.h"
#include "servo_packet.h"

Calibration makeCalibration() {
    Calibration c;
    for (int leg = 0; leg < LEG_COUNT; leg++) {
        for (int joint = 0; joint < JOINT_COUNT; joint++) {
            c.trimDeg[leg][joint] = 0.0f;
            c.direction[leg][joint] = 1;
        }
    }
    return c;
}

bool calibrationIsValid(const Calibration &c) {
    for (int leg = 0; leg < LEG_COUNT; leg++) {
        for (int joint = 0; joint < JOINT_COUNT; joint++) {
            float trim = c.trimDeg[leg][joint];
            if (!isfinite(trim) || fabsf(trim) > TRIM_LIMIT_DEG) {
                return false;
            }
            int dir = c.direction[leg][joint];
            if (dir != 1 && dir != -1) {
                return false;
            }
        }
    }
    return true;
}

float calibrationAddTrim(Calibration &c, int leg, int joint, float deltaDeg) {
    float trim = clamp(c.trimDeg[leg][joint] + deltaDeg, -TRIM_LIMIT_DEG, TRIM_LIMIT_DEG);
    c.trimDeg[leg][joint] = trim;
    return trim;
}

int calibrationFlipDirection(Calibration &c, int leg, int joint) {
    c.direction[leg][joint] = (c.direction[leg][joint] == 1) ? -1 : 1;
    return c.direction[leg][joint];
}

float calibratedServoAngle(const Calibration &c, int leg, int joint, float jointAngleDeg) {
    return c.direction[leg][joint] * (jointAngleDeg + c.trimDeg[leg][joint]);
}

int calibratedServoPosition(const Calibration &c, int leg, int joint, float jointAngleDeg) {
    return servoAngleToPosition(calibratedServoAngle(c, leg, joint, jointAngleDeg));
}

bool parseLeg(const char *text, int &leg) {
    if (strlen(text) != 1) {
        return false;
    }
    char letter = (char)toupper((unsigned char)text[0]);
    for (int i = 0; i < LEG_COUNT; i++) {
        if (LEGS[i].name[0] == letter) {
            leg = i;
            return true;
        }
    }
    return false;
}

bool parseJoint(const char *text, int &joint) {
    if (strcmp(text, "1") == 0 || strcmp(text, "coxa") == 0) {
        joint = JOINT_COXA;
        return true;
    }
    if (strcmp(text, "2") == 0 || strcmp(text, "femur") == 0) {
        joint = JOINT_FEMUR;
        return true;
    }
    if (strcmp(text, "3") == 0 || strcmp(text, "tibia") == 0) {
        joint = JOINT_TIBIA;
        return true;
    }
    return false;
}

const char *jointName(int joint) {
    if (joint == JOINT_COXA) {
        return "coxa";
    }
    if (joint == JOINT_FEMUR) {
        return "femur";
    }
    return "tibia";
}
