#include "safety.h"

Safety makeSafety() {
    Safety s;
    s.fake = false;
    s.fakeVoltageMv = 0;
    s.fakeTempC = 0;
    s.nextServo = 0;
    s.lowVoltageCount = 0;
    s.noAnswerCount = 0;
    s.noPower = false;
    s.fault = SAFETY_FAULT_NONE;
    s.faultServo = -1;
    s.faultValue = 0;
    for (int i = 0; i < SAFETY_SERVO_COUNT; i++) {
        s.answered[i] = false;
        s.voltageMv[i] = 0;
        s.tempC[i] = 0;
    }
    return s;
}

static bool setFault(Safety &s, SafetyFault fault, int servoIndex, int value) {
    if (s.fault != SAFETY_FAULT_NONE) {
        return false;
    }
    s.fault = fault;
    s.faultServo = servoIndex;
    s.faultValue = value;
    return true;
}

bool safetyReport(Safety &s, int servoIndex, bool answered, int voltageMv, int tempC) {
    s.nextServo = (servoIndex + 1) % SAFETY_SERVO_COUNT;
    s.answered[servoIndex] = answered;

    if (!answered) {
        s.noAnswerCount++;
        if (s.noAnswerCount >= SAFETY_SERVO_COUNT) {
            s.noPower = true;
        }
        return false;
    }

    s.noAnswerCount = 0;
    s.noPower = false;
    s.voltageMv[servoIndex] = voltageMv;
    s.tempC[servoIndex] = tempC;

    if (tempC >= SAFETY_MAX_TEMP_C) {
        return setFault(s, SAFETY_FAULT_OVERHEAT, servoIndex, tempC);
    }

    if (voltageMv < SAFETY_LOW_VOLTAGE_MV) {
        s.lowVoltageCount++;
        if (s.lowVoltageCount >= SAFETY_LOW_VOLTAGE_COUNT) {
            return setFault(s, SAFETY_FAULT_LOW_VOLTAGE, servoIndex, voltageMv);
        }
    } else {
        s.lowVoltageCount = 0;
    }
    return false;
}

void safetyApply(const Safety &s, Controller &c) {
    if (s.fault != SAFETY_FAULT_NONE) {
        controllerSafetySit(c);
    }
}

void safetyClear(Safety &s) {
    s.fault = SAFETY_FAULT_NONE;
    s.faultServo = -1;
    s.faultValue = 0;
    s.lowVoltageCount = 0;
}

const char *safetyFaultText(SafetyFault fault) {
    if (fault == SAFETY_FAULT_LOW_VOLTAGE) {
        return "low battery";
    }
    if (fault == SAFETY_FAULT_OVERHEAT) {
        return "overheat";
    }
    return "none";
}
