#pragma once

#include "leg.h"

struct Body {
    Leg legs[LEG_COUNT];
};

Body makeBody();

float cadStanceRadius();

Vec3 footOnLegLine(const Leg &leg, float radius, float z);

Vec3 neutralFoot(const Leg &leg);

bool bodyIK(const Body &body, const Vec3 feet[], JointAngles angles[], IkStatus status[]);
