#include <math.h>
#include "body.h"

Body makeBody() {
    Body body;
    for (int i = 0; i < LEG_COUNT; i++) {
        body.legs[i] = makeLeg(LEGS[i]);
    }
    return body;
}

float cadStanceRadius() {
    Vec3 coxaToFoot = vecSub(STAND_FOOT_A, LEGS[0].p1);
    return sqrtf(coxaToFoot.x * coxaToFoot.x + coxaToFoot.y * coxaToFoot.y);
}

Vec3 footOnLegLine(const Leg &leg, float radius, float z) {
    float directionRad = degToRad(leg.directionDeg);
    Vec3 foot = {
        leg.mount.x + radius * cosf(directionRad),
        leg.mount.y + radius * sinf(directionRad),
        z
    };
    return foot;
}

Vec3 neutralFoot(const Leg &leg) {
    return footOnLegLine(leg, cadStanceRadius(), STAND_FOOT_A.z);
}

bool bodyIK(const Body &body, const Vec3 feet[], JointAngles angles[], IkStatus status[]) {
    bool allOk = true;
    for (int i = 0; i < LEG_COUNT; i++) {
        status[i] = legIK(body.legs[i], feet[i], angles[i]);
        if (status[i] != IK_OK) {
            allOk = false;
        }
    }
    return allOk;
}
