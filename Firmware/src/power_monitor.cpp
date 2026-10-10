#include <math.h>
#include "config.h"
#include "power_monitor.h"

uint16_t ina226Calibration(float currentLsbA, float shuntOhm) {
    return (uint16_t)lroundf(0.00512f / (currentLsbA * shuntOhm));
}

uint16_t ina226ConfigValue(int avgCode, int busCtCode, int shuntCtCode) {
    uint16_t value = 0x4000;
    value = value | ((avgCode & 7) << 9);
    value = value | ((busCtCode & 7) << 6);
    value = value | ((shuntCtCode & 7) << 3);
    value = value | 7;
    return value;
}

float ina226BusVoltageMv(uint16_t raw) {
    return raw * 1.25f;
}

float ina226ShuntVoltageUv(uint16_t raw) {
    return (int16_t)raw * 2.5f;
}

float ina226CurrentMa(uint16_t raw, float currentLsbA) {
    return (int16_t)raw * currentLsbA * 1000.0f;
}

float ina226PowerMw(uint16_t raw, float currentLsbA) {
    return raw * 25.0f * currentLsbA * 1000.0f;
}

float adcToBatteryMv(float adcMv) {
    return adcMv * (VBATT_DIVIDER_TOP_OHM + VBATT_DIVIDER_BOTTOM_OHM) / VBATT_DIVIDER_BOTTOM_OHM;
}

static BatteryState plainState(float mv) {
    if (mv < BATTERY_NOT_CONNECTED_MV) {
        return BATTERY_NOT_CONNECTED;
    }
    if (mv < BATTERY_CRITICAL_MV) {
        return BATTERY_CRITICAL;
    }
    if (mv < BATTERY_LOW_MV) {
        return BATTERY_LOW;
    }
    return BATTERY_OK;
}

static int badness(BatteryState s) {
    if (s == BATTERY_CRITICAL) {
        return 2;
    }
    if (s == BATTERY_LOW) {
        return 1;
    }
    return 0;
}

BatteryState batteryState(float voltageMv, BatteryState previous) {
    BatteryState plain = plainState(voltageMv);
    if (plain == BATTERY_NOT_CONNECTED || previous == BATTERY_NOT_CONNECTED) {
        return plain;
    }
    if (badness(plain) >= badness(previous)) {
        return plain;
    }
    BatteryState careful = plainState(voltageMv - BATTERY_HYSTERESIS_MV);
    if (careful != BATTERY_NOT_CONNECTED && badness(careful) < badness(previous)) {
        return careful;
    }
    return previous;
}

const char *batteryStateName(BatteryState state) {
    if (state == BATTERY_NOT_CONNECTED) {
        return "NOT_CONNECTED";
    }
    if (state == BATTERY_OK) {
        return "OK";
    }
    if (state == BATTERY_LOW) {
        return "LOW";
    }
    return "CRITICAL";
}
