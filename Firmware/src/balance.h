#pragma once

#include "body.h"

struct Leveling {
    bool enabled;
    float filteredRollDeg;
    float filteredPitchDeg;
    float correctionRollDeg;
    float correctionPitchDeg;
    bool limited;
};

Leveling makeLeveling();

void levelRotateFeet(float rollDeg, float pitchDeg, float floorZ, const Vec3 in[], Vec3 out[]);

bool levelFeetReachable(const Body &body, const Vec3 feet[], float rollDeg, float pitchDeg, float floorZ);

void levelUpdate(Leveling &lv, bool active, float rollDeg, float pitchDeg, float dtSeconds,
                 const Body &body, float floorZ, Vec3 feet[]);
