#include <math.h>
#include <stdlib.h>
#include "imu.h"

const float IMU_MIN_GRAVITY = 5.0f;

ImuCalibration imuCalibrationFromByte(uint8_t calibStat) {
    ImuCalibration c;
    c.sys = (calibStat >> 6) & 0x03;
    c.gyro = (calibStat >> 4) & 0x03;
    c.accel = (calibStat >> 2) & 0x03;
    c.mag = calibStat & 0x03;
    return c;
}

bool imuCalibrationReadyToSave(ImuCalibration c) {
    return c.gyro == 3 && c.accel == 3 && c.mag == 3;
}

int16_t imuRaw16(uint8_t low, uint8_t high) {
    return (int16_t)(low | (high << 8));
}

float imuHeadingFromRaw(int16_t raw) {
    return raw / 16.0f;
}

Vec3 imuGravityFromRaw(int16_t x, int16_t y, int16_t z) {
    Vec3 g = { x / 100.0f, y / 100.0f, z / 100.0f };
    return g;
}

bool imuAxisMapIsValid(const int map[3]) {
    int axis[3];
    int signs = 1;
    for (int i = 0; i < 3; i++) {
        int a = abs(map[i]);
        if (a < 1 || a > 3) {
            return false;
        }
        axis[i] = a - 1;
        signs = signs * (map[i] > 0 ? 1 : -1);
    }
    if (axis[0] == axis[1] || axis[0] == axis[2] || axis[1] == axis[2]) {
        return false;
    }
    bool sameOrder = (axis[1] == (axis[0] + 1) % 3);
    int orderSign = sameOrder ? 1 : -1;
    if (orderSign * signs != 1) {
        return false;
    }
    return axis[2] == 2;
}

static float component(Vec3 v, int axis) {
    if (axis == 0) {
        return v.x;
    }
    if (axis == 1) {
        return v.y;
    }
    return v.z;
}

static float mapOne(Vec3 sensor, int mapValue) {
    float value = component(sensor, abs(mapValue) - 1);
    return (mapValue > 0) ? value : -value;
}

Vec3 imuSensorToRobot(Vec3 sensor, const int map[3]) {
    Vec3 robot = { mapOne(sensor, map[0]), mapOne(sensor, map[1]), mapOne(sensor, map[2]) };
    return robot;
}

bool imuTilt(Vec3 gravityRobot, float &rollDeg, float &pitchDeg) {
    rollDeg = 0.0f;
    pitchDeg = 0.0f;
    float length = vecLength(gravityRobot);
    if (length < IMU_MIN_GRAVITY) {
        return false;
    }
    Vec3 up = vecScale(gravityRobot, 1.0f / length);
    pitchDeg = radToDeg(atan2f(-up.y, up.z));
    rollDeg = radToDeg(atan2f(up.x, sqrtf(up.y * up.y + up.z * up.z)));
    if (pitchDeg == 0.0f) {
        pitchDeg = 0.0f;
    }
    if (rollDeg == 0.0f) {
        rollDeg = 0.0f;
    }
    return true;
}

float imuRobotHeading(float sensorHeadingDeg, const int map[3], float offsetDeg) {
    float heading = (map[2] > 0) ? sensorHeadingDeg : -sensorHeadingDeg;
    heading = fmodf(heading + offsetDeg, 360.0f);
    if (heading < 0.0f) {
        heading += 360.0f;
    }
    return heading;
}
