#pragma once

#include "controller.h"

const int SAFETY_SERVO_COUNT = LEG_COUNT * 3;

enum SafetyFault {
    SAFETY_FAULT_NONE,
    SAFETY_FAULT_LOW_VOLTAGE,
    SAFETY_FAULT_OVERHEAT
};

struct Safety {
    bool fake;
    int fakeVoltageMv;
    int fakeTempC;

    int nextServo;
    int lowVoltageCount;
    int noAnswerCount;
    bool noPower;

    SafetyFault fault;
    int faultServo;
    int faultValue;

    bool answered[SAFETY_SERVO_COUNT];
    int voltageMv[SAFETY_SERVO_COUNT];
    int tempC[SAFETY_SERVO_COUNT];
};

Safety makeSafety();

bool safetyReport(Safety &s, int servoIndex, bool answered, int voltageMv, int tempC);

void safetyApply(const Safety &s, Controller &c);

void safetyClear(Safety &s);

const char *safetyFaultText(SafetyFault fault);
