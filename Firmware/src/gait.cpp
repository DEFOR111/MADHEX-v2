#include <math.h>
#include <string.h>
#include "gait.h"

const float OFFSET_TRIPOD[LEG_COUNT] = { 0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.5f };

const float OFFSET_RIPPLE[LEG_COUNT] = { 2.0f / 6, 4.0f / 6, 1.0f / 6, 5.0f / 6, 3.0f / 6, 0.0f };

const float OFFSET_WAVE[LEG_COUNT] = { 4.0f / 6, 5.0f / 6, 2.0f / 6, 1.0f / 6, 0.0f, 3.0f / 6 };

const char *gaitName(GaitType type) {
    if (type == GAIT_TRIPOD) {
        return "tripod";
    }
    if (type == GAIT_RIPPLE) {
        return "ripple";
    }
    return "wave";
}

bool gaitFromName(const char *name, GaitType &type) {
    if (strcmp(name, "tripod") == 0) {
        type = GAIT_TRIPOD;
        return true;
    }
    if (strcmp(name, "ripple") == 0) {
        type = GAIT_RIPPLE;
        return true;
    }
    if (strcmp(name, "wave") == 0) {
        type = GAIT_WAVE;
        return true;
    }
    return false;
}

float gaitSwingFraction(GaitType type) {
    if (type == GAIT_TRIPOD) {
        return 1.0f / 2;
    }
    if (type == GAIT_RIPPLE) {
        return 1.0f / 3;
    }
    return 1.0f / 6;
}

float gaitLegOffset(GaitType type, int legIndex) {
    if (type == GAIT_TRIPOD) {
        return OFFSET_TRIPOD[legIndex];
    }
    if (type == GAIT_RIPPLE) {
        return OFFSET_RIPPLE[legIndex];
    }
    return OFFSET_WAVE[legIndex];
}

float gaitLegPhase(GaitType type, int legIndex, float phase) {
    float q = phase + gaitLegOffset(type, legIndex);
    return q - floorf(q);
}

bool gaitLegInSwing(GaitType type, int legIndex, float phase) {
    return gaitLegPhase(type, legIndex, phase) >= 1.0f - gaitSwingFraction(type);
}

GaitCommand makeCommand(float vx, float vy, float turn) {
    GaitCommand command = { vx, vy, turn };
    return command;
}

GaitCommand gaitLimitCommand(GaitCommand command) {
    if (!isfinite(command.vx) || !isfinite(command.vy) || !isfinite(command.turn)) {
        return makeCommand(0.0f, 0.0f, 0.0f);
    }
    GaitCommand c = makeCommand(clamp(command.vx, -1.0f, 1.0f),
                                clamp(command.vy, -1.0f, 1.0f),
                                clamp(command.turn, -1.0f, 1.0f));
    float total = sqrtf(c.vx * c.vx + c.vy * c.vy) + fabsf(c.turn);
    if (total > 1.0f) {
        c.vx = c.vx / total;
        c.vy = c.vy / total;
        c.turn = c.turn / total;
    }
    return c;
}

Vec3 gaitFoot(const Leg &leg, int legIndex, GaitType type, float phase, GaitCommand command) {
    GaitCommand c = gaitLimitCommand(command);
    float q = gaitLegPhase(type, legIndex, phase);
    float stanceEnd = 1.0f - gaitSwingFraction(type);

    float s;
    float lift;
    if (q < stanceEnd) {
        float t = q / stanceEnd;
        s = 0.5f - t;
        lift = 0.0f;
    } else {
        float t = (q - stanceEnd) / (1.0f - stanceEnd);
        s = -0.5f * cosf(MATHX_PI * t);
        lift = GAIT_LIFT_MM * sinf(MATHX_PI * t);
    }

    float motion = sqrtf(c.vx * c.vx + c.vy * c.vy) + fabsf(c.turn);
    lift = lift * clamp(motion / GAIT_LIFT_FADE, 0.0f, 1.0f);

    Vec3 neutral = neutralFoot(leg);
    Vec3 foot = rotateZ(neutral, degToRad(s * c.turn * GAIT_TURN_DEG));
    foot.x = foot.x + s * c.vx * GAIT_STRIDE_MM;
    foot.y = foot.y + s * c.vy * GAIT_STRIDE_MM;
    foot.z = neutral.z + lift;
    return foot;
}

void gaitFeet(const Body &body, GaitType type, float phase, GaitCommand command, Vec3 feet[]) {
    for (int i = 0; i < LEG_COUNT; i++) {
        feet[i] = gaitFoot(body.legs[i], i, type, phase, command);
    }
}

Gait makeGait() {
    Gait gait;
    gait.type = GAIT_RIPPLE;
    gait.walking = false;
    gait.stopping = false;
    gait.phase = 0.0f;
    gait.target = makeCommand(0.0f, 0.0f, 0.0f);
    gait.current = makeCommand(0.0f, 0.0f, 0.0f);
    return gait;
}

void gaitWalk(Gait &gait, GaitCommand command) {
    gait.target = gaitLimitCommand(command);
    if (!gait.walking) {
        gait.phase = 0.0f;
        gait.current = makeCommand(0.0f, 0.0f, 0.0f);
        gait.walking = true;
    }
    gait.stopping = false;
}

void gaitStop(Gait &gait) {
    if (gait.walking) {
        gait.stopping = true;
        gait.target = makeCommand(0.0f, 0.0f, 0.0f);
    }
}

static void rampCommand(GaitCommand &current, GaitCommand target, float step) {
    float dvx = target.vx - current.vx;
    float dvy = target.vy - current.vy;
    float dturn = target.turn - current.turn;
    float biggest = fmaxf(fabsf(dvx), fmaxf(fabsf(dvy), fabsf(dturn)));
    if (biggest <= step) {
        current = target;
        return;
    }
    float k = step / biggest;
    current.vx = current.vx + dvx * k;
    current.vy = current.vy + dvy * k;
    current.turn = current.turn + dturn * k;
}

void gaitUpdate(Gait &gait, float dtSeconds) {
    if (!gait.walking) {
        return;
    }

    rampCommand(gait.current, gait.target, dtSeconds / GAIT_RAMP_S);

    bool commandIsZero = gait.current.vx == 0.0f && gait.current.vy == 0.0f
                      && gait.current.turn == 0.0f;
    if (gait.stopping && commandIsZero) {
        gait.walking = false;
        gait.stopping = false;
        return;
    }

    float newPhase = gait.phase + dtSeconds * gaitSwingFraction(gait.type) / GAIT_SWING_S;
    if (newPhase >= 1.0f) {
        newPhase = newPhase - 1.0f;
    }
    gait.phase = newPhase;
}
