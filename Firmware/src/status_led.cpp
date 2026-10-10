#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"
#include "status_led.h"
#include "status_color.h"
#include "power_monitor_hw.h"
#include "servo_output_hw.h"

static Adafruit_NeoPixel led(1, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);

static LedColor shown = LED_OFF;

void statusLedBegin() {
    led.begin();
    led.setBrightness(STATUS_LED_BRIGHTNESS);
    led.clear();
    led.show();
    shown = LED_OFF;
}

void statusLedUpdate(const Controller &ctrl, const Safety &safety) {
    PowerReading power = powerLatest();
    LedInputs in;
    in.state = ctrl.state;
    in.safetyFault = (safety.fault != SAFETY_FAULT_NONE);
    in.batteryValid = power.valid;
    in.battery = power.state;
    in.servoOutputOff = (servoOutputMode() == SERVO_OUTPUT_OFF);

    LedColor color = statusLedColor(in, millis());
    if (ledColorEqual(color, shown)) {
        return;
    }
    led.setPixelColor(0, led.Color(color.r, color.g, color.b));
    led.show();
    shown = color;
}
