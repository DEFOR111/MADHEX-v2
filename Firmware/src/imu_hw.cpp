#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include "config.h"
#include "imu_hw.h"

const char IMU_NAMESPACE[] = "imu";
const char IMU_KEY_OFFSETS[] = "offsets";
const char IMU_KEY_VERSION[] = "version";
const uint8_t IMU_STORE_VERSION = 1;

static bool present = false;
static bool restored = false;
static unsigned long lastReadMs = 0;
static ImuReading latest = { false, false, 0.0f, 0.0f, 0.0f, { 0, 0, 0, 0 }, { 0.0f, 0.0f, 0.0f } };

static bool writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(IMU_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

static bool readRegisters(uint8_t reg, uint8_t out[], int count) {
    Wire.beginTransmission(IMU_ADDRESS);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) {
        return false;
    }
    if (Wire.requestFrom((int)IMU_ADDRESS, count) != count) {
        return false;
    }
    for (int i = 0; i < count; i++) {
        out[i] = Wire.read();
    }
    return true;
}

static bool restoreOffsets() {
    Preferences prefs;
    if (!prefs.begin(IMU_NAMESPACE, true)) {
        return false;
    }
    uint8_t offsets[BNO055_OFFSET_BYTES];
    bool ok = prefs.getUChar(IMU_KEY_VERSION, 0) == IMU_STORE_VERSION
        && prefs.getBytesLength(IMU_KEY_OFFSETS) == BNO055_OFFSET_BYTES
        && prefs.getBytes(IMU_KEY_OFFSETS, offsets, BNO055_OFFSET_BYTES) == BNO055_OFFSET_BYTES;
    prefs.end();
    for (int i = 0; ok && i < BNO055_OFFSET_BYTES; i++) {
        ok = writeRegister(BNO055_REG_OFFSETS + i, offsets[i]);
    }
    return ok;
}

void imuBegin() {
    pinMode(IMU_INT_PIN, INPUT);

    pinMode(IMU_RESET_PIN, OUTPUT);
    digitalWrite(IMU_RESET_PIN, LOW);
    delay(2);
    digitalWrite(IMU_RESET_PIN, HIGH);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_HZ);

    unsigned long start = millis();
    uint8_t id = 0;
    while (!(readRegisters(BNO055_REG_CHIP_ID, &id, 1) && id == BNO055_CHIP_ID)) {
        if (millis() - start > IMU_BOOT_TIMEOUT_MS) {
            Serial.println("imu: BNO055 not found at 0x28, the robot works without it");
            return;
        }
        delay(10);
    }
    unsigned long bootMs = millis() - start;
    delay(50);

    bool ok = writeRegister(BNO055_REG_PAGE_ID, 0)
        && writeRegister(BNO055_REG_PWR_MODE, BNO055_POWER_NORMAL)
        && writeRegister(BNO055_REG_SYS_TRIGGER, 0x00)
        && writeRegister(BNO055_REG_UNIT_SEL, 0x00);
    restored = ok && restoreOffsets();
    ok = ok && writeRegister(BNO055_REG_OPR_MODE, BNO055_MODE_NDOF);
    delay(BNO055_FROM_CONFIG_MS + 13);

    char line[160];
    if (!ok) {
        Serial.println("imu: BNO055 answered but the setup failed, the robot works without it");
        return;
    }
    present = true;
    int n = snprintf(line, sizeof(line), "imu: BNO055 found at 0x28 (%lu ms after reset), NDOF mode, internal oscillator, %s\r\n",
                     bootMs, restored ? "calibration restored from flash"
                                      : "no saved calibration (calibrate, then 'imu save')");
    Serial.write((const uint8_t *)line, n);
}

void imuUpdate() {
    if (!present || millis() - lastReadMs < IMU_READ_PERIOD_MS) {
        return;
    }
    lastReadMs = millis();

    uint8_t heading[2];
    uint8_t block[BNO055_GRAVITY_BLOCK_BYTES];
    if (!readRegisters(BNO055_REG_EULER_HEADING, heading, 2)
        || !readRegisters(BNO055_REG_GRAVITY, block, BNO055_GRAVITY_BLOCK_BYTES)) {
        Serial.println("imu: BNO055 stopped answering, the robot works without it");
        present = false;
        latest.valid = false;
        return;
    }

    ImuReading r;
    r.valid = true;
    r.gravitySensor = imuGravityFromRaw(imuRaw16(block[0], block[1]), imuRaw16(block[2], block[3]),
                                        imuRaw16(block[4], block[5]));
    r.calib = imuCalibrationFromByte(block[7]);
    r.tiltValid = imuTilt(imuSensorToRobot(r.gravitySensor, IMU_AXIS_MAP), r.rollDeg, r.pitchDeg);
    r.headingDeg = imuRobotHeading(imuHeadingFromRaw(imuRaw16(heading[0], heading[1])), IMU_AXIS_MAP,
                                   IMU_HEADING_OFFSET_DEG);
    latest = r;
}

bool imuPresent() {
    return present;
}

ImuReading imuLatest() {
    return latest;
}

const char *imuSaveCalibration() {
    if (!present) {
        return "no BNO055";
    }
    if (!latest.valid || !imuCalibrationReadyToSave(latest.calib)) {
        return "not calibrated yet (gyro, accel and mag must all be 3)";
    }
    uint8_t offsets[BNO055_OFFSET_BYTES];
    bool ok = writeRegister(BNO055_REG_OPR_MODE, BNO055_MODE_CONFIG);
    delay(BNO055_TO_CONFIG_MS + 6);
    ok = ok && readRegisters(BNO055_REG_OFFSETS, offsets, BNO055_OFFSET_BYTES);
    bool back = writeRegister(BNO055_REG_OPR_MODE, BNO055_MODE_NDOF);
    delay(BNO055_FROM_CONFIG_MS + 3);
    if (!ok || !back) {
        return "reading the offsets from the BNO055 failed";
    }

    Preferences prefs;
    if (!prefs.begin(IMU_NAMESPACE, false)) {
        return "opening the flash failed";
    }
    bool saved = prefs.putBytes(IMU_KEY_OFFSETS, offsets, BNO055_OFFSET_BYTES) == BNO055_OFFSET_BYTES
        && prefs.putUChar(IMU_KEY_VERSION, IMU_STORE_VERSION) == 1;
    prefs.end();
    return saved ? NULL : "writing to flash failed";
}
