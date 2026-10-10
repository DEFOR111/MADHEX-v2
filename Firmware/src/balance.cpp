#include <math.h>
#include "config.h"
#include "balance.h"

Leveling makeLeveling() {
    Leveling lv;
    lv.enabled = false;
    lv.filteredRollDeg = 0.0f;
    lv.filteredPitchDeg = 0.0f;
    lv.correctionRollDeg = 0.0f;
    lv.correctionPitchDeg = 0.0f;
    lv.limited = false;
    return lv;
}

void levelRotateFeet(float rollDeg, float pitchDeg, float floorZ, const Vec3 in[], Vec3 out[]) {
    float cr = cosf(degToRad(rollDeg));
    float sr = sinf(degToRad(rollDeg));
    float cp = cosf(degToRad(pitchDeg));
    float sp = sinf(degToRad(pitchDeg));
    for (int i = 0; i < LEG_COUNT; i++) {
        float x = in[i].x;
        float y = in[i].y;
        float z = in[i].z - floorZ;
        float x1 = x * cr - z * sr;
        float z1 = x * sr + z * cr;
        float y2 = y * cp + z1 * sp;
        float z2 = -y * sp + z1 * cp;
        out[i].x = x1;
        out[i].y = y2;
        out[i].z = z2 + floorZ;
    }
}

bool levelFeetReachable(const Body &body, const Vec3 feet[], float rollDeg, float pitchDeg, float floorZ) {
    Vec3 rotated[LEG_COUNT];
    levelRotateFeet(rollDeg, pitchDeg, floorZ, feet, rotated);
    for (int i = 0; i < LEG_COUNT; i++) {
        JointAngles angles;
        if (legIK(body.legs[i], rotated[i], angles) != IK_OK) {
            return false;
        }
    }
    return true;
}

static float lowPass(float value, float input, float dt) {
    return value + (input - value) * dt / (LEVEL_FILTER_S + dt);
}

static float stepCorrection(float correction, float tilt, bool active, float dt) {
    float change;
    if (!active) {
        change = -correction;
    } else if (fabsf(tilt) < LEVEL_DEADBAND_DEG) {
        change = 0.0f;
    } else {
        change = LEVEL_GAIN_PER_S * tilt * dt;
    }
    float maxStep = LEVEL_MAX_RATE_DEG_S * dt;
    change = clamp(change, -maxStep, maxStep);
    return clamp(correction + change, -LEVEL_MAX_DEG, LEVEL_MAX_DEG);
}

void levelUpdate(Leveling &lv, bool active, float rollDeg, float pitchDeg, float dtSeconds,
                 const Body &body, float floorZ, Vec3 feet[]) {
    if (active) {
        lv.filteredRollDeg = lowPass(lv.filteredRollDeg, rollDeg, dtSeconds);
        lv.filteredPitchDeg = lowPass(lv.filteredPitchDeg, pitchDeg, dtSeconds);
    } else {
        lv.filteredRollDeg = 0.0f;
        lv.filteredPitchDeg = 0.0f;
    }
    lv.correctionRollDeg = stepCorrection(lv.correctionRollDeg, lv.filteredRollDeg, active, dtSeconds);
    lv.correctionPitchDeg = stepCorrection(lv.correctionPitchDeg, lv.filteredPitchDeg, active, dtSeconds);

    int tenths = 10;
    while (tenths > 0 && !levelFeetReachable(body, feet, lv.correctionRollDeg * tenths / 10.0f,
                                             lv.correctionPitchDeg * tenths / 10.0f, floorZ)) {
        tenths--;
    }
    lv.limited = (tenths < 10);
    lv.correctionRollDeg = lv.correctionRollDeg * tenths / 10.0f;
    lv.correctionPitchDeg = lv.correctionPitchDeg * tenths / 10.0f;

    levelRotateFeet(lv.correctionRollDeg, lv.correctionPitchDeg, floorZ, feet, feet);
}
