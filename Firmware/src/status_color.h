#pragma once

#include <stdint.h>
#include "controller.h"
#include "power_monitor.h"

struct LedColor {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

const LedColor LED_OFF = { 0, 0, 0 };
const LedColor LED_DIM_BLUE = { 0, 0, 80 };
const LedColor LED_GREEN = { 0, 255, 0 };
const LedColor LED_CYAN = { 0, 255, 255 };
const LedColor LED_YELLOW = { 255, 160, 0 };
const LedColor LED_RED = { 255, 0, 0 };
const LedColor LED_PURPLE = { 160, 0, 255 };
const LedColor LED_WHITE = { 255, 255, 255 };

struct LedInputs {
    RobotState state;
    bool safetyFault;
    bool batteryValid;
    BatteryState battery;
    bool servoOutputOff;
};

struct LedPattern {
    LedColor color;
    unsigned long blinkMs;
    bool whiteFlash;
};

LedPattern statusLedPattern(const LedInputs &in);

LedColor statusLedColor(const LedInputs &in, unsigned long nowMs);

bool ledColorEqual(LedColor a, LedColor b);

const char *ledColorName(LedColor c);
