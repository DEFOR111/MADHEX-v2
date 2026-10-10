#include "config.h"
#include "status_color.h"

static LedPattern makePattern(LedColor color, unsigned long blinkMs, bool whiteFlash) {
    LedPattern p = { color, blinkMs, whiteFlash };
    return p;
}

LedPattern statusLedPattern(const LedInputs &in) {
    bool critical = in.batteryValid && in.battery == BATTERY_CRITICAL;
    if (in.state == STATE_ESTOP || in.safetyFault || critical) {
        return makePattern(LED_RED, LED_BLINK_FAST_MS, in.servoOutputOff);
    }
    if (in.batteryValid && in.battery == BATTERY_NOT_CONNECTED) {
        return makePattern(LED_PURPLE, 0, in.servoOutputOff);
    }
    if (in.batteryValid && in.battery == BATTERY_LOW) {
        return makePattern(LED_YELLOW, LED_BLINK_SLOW_MS, in.servoOutputOff);
    }
    if (in.state == STATE_SIT) {
        return makePattern(LED_DIM_BLUE, 0, in.servoOutputOff);
    }
    if (in.state == STATE_STAND) {
        return makePattern(LED_GREEN, 0, in.servoOutputOff);
    }
    return makePattern(LED_CYAN, 0, in.servoOutputOff);
}

LedColor statusLedColor(const LedInputs &in, unsigned long nowMs) {
    LedPattern p = statusLedPattern(in);
    if (p.whiteFlash && (nowMs % LED_WHITE_PERIOD_MS) < LED_WHITE_FLASH_MS) {
        return LED_WHITE;
    }
    if (p.blinkMs > 0 && (nowMs % p.blinkMs) >= p.blinkMs / 2) {
        return LED_OFF;
    }
    return p.color;
}

bool ledColorEqual(LedColor a, LedColor b) {
    return a.r == b.r && a.g == b.g && a.b == b.b;
}

const char *ledColorName(LedColor c) {
    if (ledColorEqual(c, LED_OFF)) { return "off"; }
    if (ledColorEqual(c, LED_DIM_BLUE)) { return "dim blue"; }
    if (ledColorEqual(c, LED_GREEN)) { return "green"; }
    if (ledColorEqual(c, LED_CYAN)) { return "cyan"; }
    if (ledColorEqual(c, LED_YELLOW)) { return "yellow"; }
    if (ledColorEqual(c, LED_RED)) { return "red"; }
    if (ledColorEqual(c, LED_PURPLE)) { return "purple"; }
    if (ledColorEqual(c, LED_WHITE)) { return "white"; }
    return "?";
}
