#pragma once

#include <stdint.h>
#include "mathx.h"

struct JointLimits {
    float minDeg;
    float maxDeg;
};

struct LegConfig {
    const char *name;
    const char *position;
    Vec3 p1;
    Vec3 p2;
    Vec3 p3;
    Vec3 p4;
    JointLimits coxa;
    JointLimits femur;
    JointLimits tibia;
};

const JointLimits COXA_LIMITS  = {  -50.0,  50.0 };
const JointLimits FEMUR_LIMITS = {  -90.0,  90.0 };
const JointLimits TIBIA_LIMITS = { -110.0, 110.0 };

const int LEG_COUNT = 6;

const LegConfig LEGS[LEG_COUNT] = {
    {
        "A", "right middle",
        {  -96.0,    0.0, 60.0 },
        { -141.0,    0.0, 60.0 },
        { -269.0,    0.0, 60.0 },
        { -405.0,    0.0, 20.0 },
        COXA_LIMITS, FEMUR_LIMITS, TIBIA_LIMITS
    },
    {
        "B", "right rear",
        {  -88.0,   81.5, 60.0 },
        { -126.5,  103.5, 60.0 },
        { -236.5,  167.0, 60.0 },
        { -355.0,  235.0, 20.0 },
        COXA_LIMITS, FEMUR_LIMITS, TIBIA_LIMITS
    },
    {
        "C", "left rear",
        {   88.0,   81.5, 60.0 },
        {  126.5,  103.5, 60.0 },
        {  236.5,  167.0, 60.0 },
        {  355.0,  235.0, 20.0 },
        COXA_LIMITS, FEMUR_LIMITS, TIBIA_LIMITS
    },
    {
        "D", "left middle",
        {   96.0,    0.0, 60.0 },
        {  141.0,    0.0, 60.0 },
        {  269.0,    0.0, 60.0 },
        {  405.0,    0.0, 20.0 },
        COXA_LIMITS, FEMUR_LIMITS, TIBIA_LIMITS
    },
    {
        "E", "left front",
        {   88.0,  -81.5, 60.0 },
        {  126.5, -103.5, 60.0 },
        {  236.5, -167.0, 60.0 },
        {  355.0, -235.0, 20.0 },
        COXA_LIMITS, FEMUR_LIMITS, TIBIA_LIMITS
    },
    {
        "F", "right front",
        {  -88.0,  -81.5, 60.0 },
        { -126.5, -103.5, 60.0 },
        { -236.5, -167.0, 60.0 },
        { -355.0, -235.0, 20.0 },
        COXA_LIMITS, FEMUR_LIMITS, TIBIA_LIMITS
    },
};

const int REFERENCE_LEG = 0;

const float LENGTH_WARNING_MM = 1.0;

const Vec3 STAND_FOOT_A = { -305.0, 0.0, 0.0 };

const float BODY_UNDERSIDE_Z = 30.0;

const float GAIT_STRIDE_MM = 80.0;
const float GAIT_LIFT_MM = 25.0;
const float GAIT_SWING_S = 0.5;
const float GAIT_RAMP_S = 1.0;
const float GAIT_TURN_DEG = 14.0;
const float GAIT_LIFT_FADE = 0.25;

const int SERVO_BUS1_TX_PIN = 4;
const int SERVO_BUS1_RX_PIN = 5;
const int SERVO_BUS2_TX_PIN = 17;
const int SERVO_BUS2_RX_PIN = 18;
const unsigned long SERVO_BAUD = 115200;

const uint8_t SERVO_ID[LEG_COUNT][3] = {
    {  1,  2,  3 },
    {  4,  5,  6 },
    {  7,  8,  9 },
    { 10, 11, 12 },
    { 13, 14, 15 },
    { 16, 17, 18 },
};

const int SERVO_BUS_OF_LEG[LEG_COUNT] = { 1, 1, 2, 2, 2, 1 };

const unsigned long SERVO_REPLY_TIMEOUT_MS = 20;
const unsigned long SERVO_ID_VERIFY_TIMEOUT_MS = 100;
const int SERVO_TEST_MOVE_MS = 2000;

const float TRIM_LIMIT_DEG = 30.0;
const int CALIBRATION_MOVE_MS = 1000;
const int TRIM_MOVE_MS = 200;

const unsigned long SAFETY_CHECK_PERIOD_MS = 1000;
const int SAFETY_LOW_VOLTAGE_MV = 6600;
const int SAFETY_LOW_VOLTAGE_COUNT = 2;
const int SAFETY_MAX_TEMP_C = 70;
const unsigned long SAFETY_REPLY_TIMEOUT_MS = 10;

const int I2C_SDA_PIN = 8;
const int I2C_SCL_PIN = 9;
const unsigned long I2C_CLOCK_HZ = 400000;

const uint8_t INA226_ADDRESS = 0x40;
const float INA226_SHUNT_OHM = 0.002;
const float INA226_CURRENT_LSB_A = 0.001;
const int INA226_AVG_CODE = 2;
const int INA226_BUS_CT_CODE = 4;
const int INA226_SHUNT_CT_CODE = 4;
const unsigned long POWER_READ_PERIOD_MS = 100;

const int BATTERY_NOT_CONNECTED_MV = 1000;
const int BATTERY_CRITICAL_MV = 6400;
const int BATTERY_LOW_MV = 6800;
const int BATTERY_HYSTERESIS_MV = 200;

const uint8_t IMU_ADDRESS = 0x28;
const int IMU_RESET_PIN = 10;
const int IMU_INT_PIN = 11;
const unsigned long IMU_BOOT_TIMEOUT_MS = 1000;
const unsigned long IMU_READ_PERIOD_MS = 20;

const int IMU_AXIS_MAP[3] = {
    1,
    2,
    3,
};
const float IMU_HEADING_OFFSET_DEG = 0.0;

const char WIFI_AP_SSID[] = "hexapod";
const char WIFI_AP_PASSWORD[] = "hexapod123";
const unsigned long WEB_LINK_TIMEOUT_MS = 500;
const unsigned long WEB_STATUS_PERIOD_MS = 200;
const int WEB_MAX_CLIENTS = 2;

const float LEVEL_MAX_DEG = 6.0;
const float LEVEL_GAIN_PER_S = 1.0;
const float LEVEL_MAX_RATE_DEG_S = 5.0;
const float LEVEL_FILTER_S = 0.3;
const float LEVEL_DEADBAND_DEG = 0.5;

const int VBATT_ADC_PIN = 1;
const float VBATT_DIVIDER_TOP_OHM = 100000.0;
const float VBATT_DIVIDER_BOTTOM_OHM = 33000.0;

const float HEIGHT_CHANGE_S = 1.5;

const float STAND_HEIGHT_MIN_MM = -20.0;
const float STAND_HEIGHT_MAX_MM = 30.0;
const float STAND_HEIGHT_RATE_MM_S = 20.0;

const unsigned long UPDATE_PERIOD_MS = 20;

const float SERVO_STARTUP_MOVE_S = 2.5;

const int STATUS_LED_PIN = 7;
const uint8_t STATUS_LED_BRIGHTNESS = 40;
const unsigned long LED_BLINK_SLOW_MS = 1000;
const unsigned long LED_BLINK_FAST_MS = 400;
const unsigned long LED_WHITE_PERIOD_MS = 2000;
const unsigned long LED_WHITE_FLASH_MS = 100;
