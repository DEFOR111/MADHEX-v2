#include <math.h>
#include "controller.h"

const float STAND_FLOOR_Z = STAND_FOOT_A.z;
const float SIT_FLOOR_Z = BODY_UNDERSIDE_Z;

static ControlReply reply(bool accepted, const char *message) {
    ControlReply r = { accepted, message };
    return r;
}

static ControlReply busyReply() {
    return reply(false, "error: busy, wait until standing up / sitting down is done");
}

static ControlReply estopReply() {
    return reply(false, "error: emergency stop is active, press RESET to continue");
}

static ControlReply lockedReply() {
    return reply(false, "error: safety fault, the robot must sit (see 'safety')");
}

static void startHeightChange(Controller &c, RobotState newState, float toZ) {
    c.state = newState;
    c.moveFromZ = c.floorZ;
    c.moveToZ = toZ;
    c.moveTime = 0.0f;
}

Controller makeController() {
    Controller c;
    c.state = STATE_SIT;
    c.gait = makeGait();
    c.floorZ = SIT_FLOOR_Z;
    c.standFloorZ = STAND_FLOOR_Z;
    c.moveFromZ = SIT_FLOOR_Z;
    c.moveToZ = SIT_FLOOR_Z;
    c.moveTime = 0.0f;
    c.sitAfterStop = false;
    c.safetyLock = false;
    return c;
}

ControlReply controllerUp(Controller &c) {
    if (c.state == STATE_ESTOP) {
        return estopReply();
    }
    if (c.safetyLock) {
        return lockedReply();
    }
    if (c.state == STATE_SIT) {
        startHeightChange(c, STATE_STANDING_UP, c.standFloorZ);
        return reply(true, "up: standing up");
    }
    if (c.state == STATE_STANDING_UP || c.state == STATE_SITTING_DOWN) {
        return busyReply();
    }
    return reply(false, "up: already standing");
}

ControlReply controllerDown(Controller &c) {
    if (c.state == STATE_ESTOP) {
        return estopReply();
    }
    if (c.state == STATE_STAND) {
        startHeightChange(c, STATE_SITTING_DOWN, SIT_FLOOR_Z);
        return reply(true, "down: sitting down");
    }
    if (c.state == STATE_WALK) {
        if (c.sitAfterStop) {
            return reply(false, "down: already stopping to sit down");
        }
        gaitStop(c.gait);
        c.sitAfterStop = true;
        return reply(true, "down: stopping first, then sitting down");
    }
    if (c.state == STATE_SIT) {
        return reply(false, "down: already sitting");
    }
    return busyReply();
}

ControlReply controllerWalk(Controller &c, GaitCommand command) {
    if (c.state == STATE_ESTOP) {
        return estopReply();
    }
    if (c.safetyLock) {
        return lockedReply();
    }
    if (c.state == STATE_SIT) {
        return reply(false, "error: sitting, stand up first (up)");
    }
    if (c.state == STATE_STANDING_UP || c.state == STATE_SITTING_DOWN) {
        return busyReply();
    }
    if (c.state == STATE_WALK && c.sitAfterStop) {
        return reply(false, "error: stopping to sit down, wait");
    }
    bool wasWalking = (c.state == STATE_WALK);
    gaitWalk(c.gait, command);
    c.state = STATE_WALK;
    return reply(true, wasWalking ? "walk: new direction" : "walk: starting");
}

ControlReply controllerStop(Controller &c) {
    if (c.state == STATE_ESTOP) {
        return estopReply();
    }
    if (c.state == STATE_WALK) {
        if (c.gait.stopping) {
            return reply(false, "stop: already stopping");
        }
        gaitStop(c.gait);
        return reply(true, "stop: slowing down");
    }
    if (c.state == STATE_STANDING_UP || c.state == STATE_SITTING_DOWN) {
        return busyReply();
    }
    return reply(false, "stop: not walking");
}

ControlReply controllerSetGait(Controller &c, GaitType type) {
    if (c.state == STATE_ESTOP) {
        return estopReply();
    }
    if (controllerIsMoving(c)) {
        return reply(false, "error: change the gait only while not moving (SIT or STAND)");
    }
    c.gait.type = type;
    return reply(true, "gait: changed");
}

ControlReply controllerSetHeight(Controller &c, float heightMm) {
    if (c.state == STATE_ESTOP) {
        return estopReply();
    }
    if (!isfinite(heightMm)) {
        return reply(false, "error: height must be a number");
    }
    c.standFloorZ = -clamp(heightMm, STAND_HEIGHT_MIN_MM, STAND_HEIGHT_MAX_MM);
    if (c.state == STATE_SIT || c.state == STATE_SITTING_DOWN) {
        return reply(true, "height: set, used at the next 'up'");
    }
    return reply(true, "height: moving there");
}

float controllerHeightMm(const Controller &c) {
    return -c.standFloorZ;
}

ControlReply controllerEmergencyStop(Controller &c) {
    c.state = STATE_ESTOP;
    return reply(true, "STOP: emergency stop, everything frozen. Press RESET to continue");
}

void controllerSafetySit(Controller &c) {
    c.safetyLock = true;
    if (c.state == STATE_STAND) {
        startHeightChange(c, STATE_SITTING_DOWN, SIT_FLOOR_Z);
    } else if (c.state == STATE_WALK && !c.sitAfterStop) {
        gaitStop(c.gait);
        c.sitAfterStop = true;
    }
}

void controllerClearSafety(Controller &c) {
    c.safetyLock = false;
}

static void moveToStandHeight(Controller &c, float dtSeconds) {
    float step = STAND_HEIGHT_RATE_MM_S * dtSeconds;
    c.floorZ = c.floorZ + clamp(c.standFloorZ - c.floorZ, -step, step);
}

void controllerUpdate(Controller &c, float dtSeconds) {
    if (c.state == STATE_ESTOP) {
        return;
    }
    if (c.state == STATE_STAND) {
        moveToStandHeight(c, dtSeconds);
        return;
    }
    if (c.state == STATE_WALK) {
        moveToStandHeight(c, dtSeconds);
        gaitUpdate(c.gait, dtSeconds);
        if (!c.gait.walking) {
            c.state = STATE_STAND;
            if (c.sitAfterStop) {
                c.sitAfterStop = false;
                startHeightChange(c, STATE_SITTING_DOWN, SIT_FLOOR_Z);
            }
        }
        return;
    }

    if (c.state == STATE_STANDING_UP || c.state == STATE_SITTING_DOWN) {
        c.moveTime = c.moveTime + dtSeconds;
        float t = clamp(c.moveTime / HEIGHT_CHANGE_S, 0.0f, 1.0f);
        float ease = 0.5f - 0.5f * cosf(MATHX_PI * t);
        c.floorZ = c.moveFromZ + (c.moveToZ - c.moveFromZ) * ease;
        if (t >= 1.0f) {
            c.floorZ = c.moveToZ;
            c.state = (c.state == STATE_STANDING_UP) ? STATE_STAND : STATE_SIT;
            if (c.state == STATE_STAND && c.safetyLock) {
                startHeightChange(c, STATE_SITTING_DOWN, SIT_FLOOR_Z);
            }
        }
    }
}

void controllerFeet(const Controller &c, const Body &body, Vec3 feet[]) {
    if (c.gait.walking) {
        gaitFeet(body, c.gait.type, c.gait.phase, c.gait.current, feet);
        for (int i = 0; i < LEG_COUNT; i++) {
            feet[i].z = feet[i].z + c.floorZ;
        }
        return;
    }
    for (int i = 0; i < LEG_COUNT; i++) {
        feet[i] = footOnLegLine(body.legs[i], cadStanceRadius(), c.floorZ);
    }
}

bool controllerIsMoving(const Controller &c) {
    return c.state == STATE_WALK || c.state == STATE_STANDING_UP || c.state == STATE_SITTING_DOWN;
}

const char *stateName(RobotState state) {
    if (state == STATE_SIT) {
        return "SIT";
    }
    if (state == STATE_STANDING_UP) {
        return "STANDING_UP";
    }
    if (state == STATE_STAND) {
        return "STAND";
    }
    if (state == STATE_WALK) {
        return "WALK";
    }
    if (state == STATE_SITTING_DOWN) {
        return "SITTING_DOWN";
    }
    return "ESTOP";
}
