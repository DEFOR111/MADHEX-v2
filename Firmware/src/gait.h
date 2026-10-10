#pragma once

#include "body.h"

enum GaitType {
    GAIT_TRIPOD,
    GAIT_RIPPLE,
    GAIT_WAVE
};

const char *gaitName(GaitType type);

bool gaitFromName(const char *name, GaitType &type);

float gaitSwingFraction(GaitType type);

float gaitLegOffset(GaitType type, int legIndex);

float gaitLegPhase(GaitType type, int legIndex, float phase);

bool gaitLegInSwing(GaitType type, int legIndex, float phase);

struct GaitCommand {
    float vx;
    float vy;
    float turn;
};

GaitCommand makeCommand(float vx, float vy, float turn);

GaitCommand gaitLimitCommand(GaitCommand command);

Vec3 gaitFoot(const Leg &leg, int legIndex, GaitType type, float phase, GaitCommand command);

void gaitFeet(const Body &body, GaitType type, float phase, GaitCommand command, Vec3 feet[]);

struct Gait {
    GaitType type;
    bool walking;
    bool stopping;
    float phase;
    GaitCommand target;
    GaitCommand current;
};

Gait makeGait();

void gaitWalk(Gait &gait, GaitCommand command);

void gaitStop(Gait &gait);

void gaitUpdate(Gait &gait, float dtSeconds);
