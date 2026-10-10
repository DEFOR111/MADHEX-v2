#pragma once

#include <math.h>

const float MATHX_PI = 3.14159265f;

struct Vec3 {
    float x;
    float y;
    float z;
};

inline Vec3 vecAdd(Vec3 a, Vec3 b) {
    Vec3 r = { a.x + b.x, a.y + b.y, a.z + b.z };
    return r;
}

inline Vec3 vecSub(Vec3 a, Vec3 b) {
    Vec3 r = { a.x - b.x, a.y - b.y, a.z - b.z };
    return r;
}

inline Vec3 vecScale(Vec3 v, float k) {
    Vec3 r = { v.x * k, v.y * k, v.z * k };
    return r;
}

inline float vecLength(Vec3 v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

inline float vecDot(Vec3 a, Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline float degToRad(float deg) {
    return deg * (MATHX_PI / 180.0f);
}

inline float radToDeg(float rad) {
    return rad * (180.0f / MATHX_PI);
}

inline float clamp(float value, float minValue, float maxValue) {
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

inline Vec3 rotateZ(Vec3 v, float angleRad) {
    float c = cosf(angleRad);
    float s = sinf(angleRad);
    Vec3 r = {
        v.x * c - v.y * s,
        v.x * s + v.y * c,
        v.z
    };
    return r;
}
