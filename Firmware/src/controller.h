#pragma once

#include "gait.h"

enum RobotState {
    STATE_SIT,
    STATE_STANDING_UP,
    STATE_STAND,
    STATE_WALK,
    STATE_SITTING_DOWN,
    STATE_ESTOP
};

struct Controller {
    RobotState state;
    Gait gait;
    float floorZ;
    float standFloorZ;
    float moveFromZ;
    float moveToZ;
    float moveTime;
    bool sitAfterStop;
    bool safetyLock;
};

struct ControlReply {
    bool accepted;
    const char *message;
};

Controller makeController();

ControlReply controllerUp(Controller &c);
ControlReply controllerDown(Controller &c);
ControlReply controllerWalk(Controller &c, GaitCommand command);
ControlReply controllerStop(Controller &c);
ControlReply controllerSetGait(Controller &c, GaitType type);

ControlReply controllerSetHeight(Controller &c, float heightMm);

float controllerHeightMm(const Controller &c);

ControlReply controllerEmergencyStop(Controller &c);

void controllerSafetySit(Controller &c);

void controllerClearSafety(Controller &c);

void controllerUpdate(Controller &c, float dtSeconds);

void controllerFeet(const Controller &c, const Body &body, Vec3 feet[]);

bool controllerIsMoving(const Controller &c);

const char *stateName(RobotState state);
