#pragma once

#include "power_monitor.h"

struct PowerReading {
    bool valid;
    bool fromIna226;
    float voltageMv;
    float currentMa;
    float powerMw;
    BatteryState state;
};

void powerBegin();

void powerUpdate();

PowerReading powerLatest();
