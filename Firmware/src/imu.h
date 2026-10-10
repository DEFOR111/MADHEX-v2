#pragma once

#include <stdint.h>
#include "mathx.h"

const uint8_t BNO055_REG_CHIP_ID = 0x00;
const uint8_t BNO055_CHIP_ID = 0xA0;
const uint8_t BNO055_REG_PAGE_ID = 0x07;
const uint8_t BNO055_REG_EULER_HEADING = 0x1A;
const uint8_t BNO055_REG_GRAVITY = 0x2E;
const uint8_t BNO055_REG_UNIT_SEL = 0x3B;
const uint8_t BNO055_REG_OPR_MODE = 0x3D;
const uint8_t BNO055_REG_PWR_MODE = 0x3E;
const uint8_t BNO055_REG_SYS_TRIGGER = 0x3F;
const uint8_t BNO055_REG_OFFSETS = 0x55;

const uint8_t BNO055_MODE_CONFIG = 0x00;
const uint8_t BNO055_MODE_NDOF = 0x0C;
const uint8_t BNO055_POWER_NORMAL = 0x00;

const unsigned long BNO055_TO_CONFIG_MS = 19;
const unsigned long BNO055_FROM_CONFIG_MS = 7;

const int BNO055_OFFSET_BYTES = 22;

const int BNO055_GRAVITY_BLOCK_BYTES = 8;

struct ImuCalibration {
    int sys;
    int gyro;
    int accel;
    int mag;
};

ImuCalibration imuCalibrationFromByte(uint8_t calibStat);

bool imuCalibrationReadyToSave(ImuCalibration c);

int16_t imuRaw16(uint8_t low, uint8_t high);

float imuHeadingFromRaw(int16_t raw);

Vec3 imuGravityFromRaw(int16_t x, int16_t y, int16_t z);

bool imuAxisMapIsValid(const int map[3]);

Vec3 imuSensorToRobot(Vec3 sensor, const int map[3]);

bool imuTilt(Vec3 gravityRobot, float &rollDeg, float &pitchDeg);

float imuRobotHeading(float sensorHeadingDeg, const int map[3], float offsetDeg);
