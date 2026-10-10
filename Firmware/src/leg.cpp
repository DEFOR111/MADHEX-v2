#include <math.h>
#include "leg.h"
#include "mathx.h"

static float horizontalLength(Vec3 v) {
    return sqrtf(v.x * v.x + v.y * v.y);
}

static float angleAboveHorizontalDeg(Vec3 v) {
    return radToDeg(atan2f(v.z, horizontalLength(v)));
}

LegLengths measureLegLengths(const LegConfig &config) {
    LegLengths lengths;
    lengths.coxa = vecLength(vecSub(config.p2, config.p1));
    lengths.femur = vecLength(vecSub(config.p3, config.p2));
    lengths.tibia = vecLength(vecSub(config.p4, config.p3));
    return lengths;
}

float maxLengthDifferenceMm(const LegConfig &config, const LegConfig &reference) {
    LegLengths a = measureLegLengths(config);
    LegLengths b = measureLegLengths(reference);

    float biggest = fabsf(a.coxa - b.coxa);
    float femurDiff = fabsf(a.femur - b.femur);
    float tibiaDiff = fabsf(a.tibia - b.tibia);
    if (femurDiff > biggest) {
        biggest = femurDiff;
    }
    if (tibiaDiff > biggest) {
        biggest = tibiaDiff;
    }
    return biggest;
}

Leg makeLeg(const LegConfig &config) {
    const LegConfig &ref = LEGS[REFERENCE_LEG];

    Leg leg;
    leg.name = config.name;

    leg.coxaLimits = config.coxa;
    leg.femurLimits = config.femur;
    leg.tibiaLimits = config.tibia;

    leg.mount = config.p1;
    Vec3 wholeLeg = vecSub(config.p4, config.p1);
    leg.directionDeg = radToDeg(atan2f(wholeLeg.y, wholeLeg.x));

    LegLengths lengths = measureLegLengths(ref);
    leg.coxaLength = lengths.coxa;
    leg.femurLength = lengths.femur;
    leg.tibiaLength = lengths.tibia;

    Vec3 femur = vecSub(ref.p3, ref.p2);
    Vec3 tibia = vecSub(ref.p4, ref.p3);
    leg.femurZeroDeg = angleAboveHorizontalDeg(femur);
    leg.tibiaZeroDeg = angleAboveHorizontalDeg(tibia);

    Vec3 kneeToFemur = vecSub(ref.p2, ref.p3);
    Vec3 kneeToFoot = vecSub(ref.p4, ref.p3);
    float cosKnee = vecDot(kneeToFemur, kneeToFoot) / (leg.femurLength * leg.tibiaLength);
    cosKnee = clamp(cosKnee, -1.0f, 1.0f);
    leg.kneeZeroDeg = radToDeg(acosf(cosKnee));

    return leg;
}

Vec3 bodyToLeg(const Leg &leg, Vec3 pointBody) {
    Vec3 fromMount = vecSub(pointBody, leg.mount);
    return rotateZ(fromMount, degToRad(-leg.directionDeg));
}

Vec3 legToBody(const Leg &leg, Vec3 pointLeg) {
    Vec3 turned = rotateZ(pointLeg, degToRad(leg.directionDeg));
    return vecAdd(turned, leg.mount);
}

static float limitAngle(float angle, JointLimits limits, bool &hit) {
    float limited = clamp(angle, limits.minDeg, limits.maxDeg);
    if (limited != angle) {
        hit = true;
    }
    return limited;
}

static void setSafeAngles(const Leg &leg, JointAngles &angles) {
    bool unused = false;
    angles.coxa = limitAngle(0.0f, leg.coxaLimits, unused);
    angles.femur = limitAngle(0.0f, leg.femurLimits, unused);
    angles.tibia = limitAngle(0.0f, leg.tibiaLimits, unused);
}

IkStatus legIK(const Leg &leg, Vec3 footBody, JointAngles &angles) {
    if (!isfinite(footBody.x) || !isfinite(footBody.y) || !isfinite(footBody.z)) {
        setSafeAngles(leg, angles);
        return IK_OUT_OF_REACH;
    }

    Vec3 foot = bodyToLeg(leg, footBody);

    float coxaRad = atan2f(foot.y, foot.x);

    float r = horizontalLength(foot) - leg.coxaLength;
    float z = foot.z;
    float d = sqrtf(r * r + z * z);

    float a = leg.femurLength;
    float b = leg.tibiaLength;
    float maxD = a + b;
    float minD = fabsf(a - b);
    bool reachable = (d <= maxD) && (d >= minD);
    d = clamp(d, minD, maxD);

    float cosAlpha = (a * a + d * d - b * b) / (2.0f * a * d);
    float cosKnee = (a * a + b * b - d * d) / (2.0f * a * b);
    float alpha = acosf(clamp(cosAlpha, -1.0f, 1.0f));
    float knee = acosf(clamp(cosKnee, -1.0f, 1.0f));

    float lineRad = atan2f(z, r);
    float femurRad = lineRad + alpha;

    angles.coxa = radToDeg(coxaRad);
    angles.femur = radToDeg(femurRad) - leg.femurZeroDeg;
    angles.tibia = radToDeg(knee) - leg.kneeZeroDeg;

    if (!isfinite(angles.coxa) || !isfinite(angles.femur) || !isfinite(angles.tibia)) {
        setSafeAngles(leg, angles);
        return IK_OUT_OF_REACH;
    }

    bool limitHit = false;
    angles.coxa = limitAngle(angles.coxa, leg.coxaLimits, limitHit);
    angles.femur = limitAngle(angles.femur, leg.femurLimits, limitHit);
    angles.tibia = limitAngle(angles.tibia, leg.tibiaLimits, limitHit);

    if (!reachable) {
        return IK_OUT_OF_REACH;
    }
    if (limitHit) {
        return IK_LIMIT_HIT;
    }
    return IK_OK;
}

Vec3 legFK(const Leg &leg, JointAngles angles) {
    float femurDeg = leg.femurZeroDeg + angles.femur;
    float kneeDeg = leg.kneeZeroDeg + angles.tibia;
    float tibiaDeg = femurDeg - (180.0f - kneeDeg);

    float femurRad = degToRad(femurDeg);
    float tibiaRad = degToRad(tibiaDeg);

    float r = leg.coxaLength
            + leg.femurLength * cosf(femurRad)
            + leg.tibiaLength * cosf(tibiaRad);
    float z = leg.femurLength * sinf(femurRad)
            + leg.tibiaLength * sinf(tibiaRad);

    float coxaRad = degToRad(angles.coxa);
    Vec3 footLeg = { r * cosf(coxaRad), r * sinf(coxaRad), z };

    return legToBody(leg, footLeg);
}
