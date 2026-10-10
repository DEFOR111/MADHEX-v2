#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "power_monitor_hw.h"

static bool ina226Present = false;
static unsigned long lastReadMs = 0;
static PowerReading latest = { false, false, 0.0f, 0.0f, 0.0f, BATTERY_NOT_CONNECTED };

static bool writeRegister(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(INA226_ADDRESS);
    Wire.write(reg);
    Wire.write((uint8_t)(value >> 8));
    Wire.write((uint8_t)(value & 0xFF));
    return Wire.endTransmission() == 0;
}

static bool readRegister(uint8_t reg, uint16_t &value) {
    Wire.beginTransmission(INA226_ADDRESS);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) {
        return false;
    }
    if (Wire.requestFrom((int)INA226_ADDRESS, 2) != 2) {
        return false;
    }
    uint8_t high = Wire.read();
    uint8_t low = Wire.read();
    value = (uint16_t)((high << 8) | low);
    return true;
}

void powerBegin() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_HZ);

    uint16_t id = 0;
    ina226Present = readRegister(INA226_REG_MANUFACTURER_ID, id) && id == INA226_MANUFACTURER_ID
        && writeRegister(INA226_REG_CONFIG,
                         ina226ConfigValue(INA226_AVG_CODE, INA226_BUS_CT_CODE, INA226_SHUNT_CT_CODE))
        && writeRegister(INA226_REG_CALIBRATION, ina226Calibration(INA226_CURRENT_LSB_A, INA226_SHUNT_OHM));

    if (ina226Present) {
        Serial.println("power: INA226 found at 0x40");
    } else {
        Serial.println("power: INA226 not found at 0x40, using the ADC on GPIO1 (voltage only)");
    }
}

void powerUpdate() {
    if (millis() - lastReadMs < POWER_READ_PERIOD_MS) {
        return;
    }
    lastReadMs = millis();

    PowerReading r = latest;
    r.valid = true;
    uint16_t bus;
    uint16_t current;
    uint16_t power;
    if (ina226Present && readRegister(INA226_REG_BUS_VOLTAGE, bus)
        && readRegister(INA226_REG_CURRENT, current) && readRegister(INA226_REG_POWER, power)) {
        r.fromIna226 = true;
        r.voltageMv = ina226BusVoltageMv(bus);
        r.currentMa = ina226CurrentMa(current, INA226_CURRENT_LSB_A);
        r.powerMw = ina226PowerMw(power, INA226_CURRENT_LSB_A);
    } else {
        if (ina226Present) {
            Serial.println("power: INA226 stopped answering, using the ADC on GPIO1 (voltage only)");
            ina226Present = false;
        }
        r.fromIna226 = false;
        r.voltageMv = adcToBatteryMv((float)analogReadMilliVolts(VBATT_ADC_PIN));
        r.currentMa = 0.0f;
        r.powerMw = 0.0f;
    }
    r.state = batteryState(r.voltageMv, latest.state);
    latest = r;
}

PowerReading powerLatest() {
    return latest;
}
