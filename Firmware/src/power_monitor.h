#pragma once

#include <stdint.h>

const uint8_t INA226_REG_CONFIG = 0x00;
const uint8_t INA226_REG_BUS_VOLTAGE = 0x02;
const uint8_t INA226_REG_POWER = 0x03;
const uint8_t INA226_REG_CURRENT = 0x04;
const uint8_t INA226_REG_CALIBRATION = 0x05;
const uint8_t INA226_REG_MANUFACTURER_ID = 0xFE;
const uint16_t INA226_MANUFACTURER_ID = 0x5449;

uint16_t ina226Calibration(float currentLsbA, float shuntOhm);

uint16_t ina226ConfigValue(int avgCode, int busCtCode, int shuntCtCode);

float ina226BusVoltageMv(uint16_t raw);
float ina226ShuntVoltageUv(uint16_t raw);
float ina226CurrentMa(uint16_t raw, float currentLsbA);
float ina226PowerMw(uint16_t raw, float currentLsbA);

float adcToBatteryMv(float adcMv);

enum BatteryState {
    BATTERY_NOT_CONNECTED,
    BATTERY_OK,
    BATTERY_LOW,
    BATTERY_CRITICAL
};

BatteryState batteryState(float voltageMv, BatteryState previous);

const char *batteryStateName(BatteryState state);
