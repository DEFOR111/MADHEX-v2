#pragma once

#include "status_color.h"

struct LoopStats {
    int updatesPerSecond;
    unsigned long maxLoopUs;
    unsigned long maxGapUs;
};

struct WebStatus {
    const char *state;
    const char *gait;
    const char *servos;
    float heightMm;
    float heightMinMm;
    float heightMaxMm;
    bool levelOn;
    bool batteryValid;
    float batteryV;
    const char *batteryState;
    bool currentValid;
    float currentA;
    bool imuValid;
    float rollDeg;
    float pitchDeg;
    LedPattern led;
    LoopStats loop;
};

int webStatusJson(const WebStatus &s, char out[], int size);

int webReplyJson(const char *text, char out[], int size);

bool webLinkLost(bool driving, unsigned long msSinceLastMessage, unsigned long timeoutMs);

bool webIsWalkCommand(const char *line);
