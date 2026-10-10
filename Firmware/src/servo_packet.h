#pragma once

#include <stdint.h>

const uint8_t SERVO_HEADER = 0x55;
const uint8_t SERVO_BROADCAST_ID = 254;

const uint8_t SERVO_MOVE_TIME_WRITE = 1;
const uint8_t SERVO_MOVE_STOP = 12;
const uint8_t SERVO_ID_WRITE = 13;
const uint8_t SERVO_ID_READ = 14;
const uint8_t SERVO_TEMP_READ = 26;
const uint8_t SERVO_VIN_READ = 27;
const uint8_t SERVO_POS_READ = 28;

const int SERVO_POSITION_MIN = 0;
const int SERVO_POSITION_MAX = 1000;
const int SERVO_POSITION_ZERO = 500;
const float SERVO_DEG_PER_UNIT = 0.24f;
const int SERVO_TIME_MAX_MS = 30000;
const int SERVO_ID_MAX = 253;

const int SERVO_MAX_PACKET = 10;

uint8_t servoChecksum(const uint8_t packet[]);

int servoBuildMoveTime(uint8_t id, int position, int timeMs, uint8_t out[]);

int servoBuildRead(uint8_t id, uint8_t command, uint8_t out[]);

int servoBuildIdWrite(uint8_t id, uint8_t newId, uint8_t out[]);

int servoBuildIdRead(uint8_t id, uint8_t out[]);

struct ServoReceiver {
    uint8_t packet[SERVO_MAX_PACKET];
    int count;
};

void servoReceiverReset(ServoReceiver &r);

bool servoReceiverFeed(ServoReceiver &r, uint8_t byte);

bool servoReplyId(const uint8_t packet[], uint8_t &id);

bool servoReplyVoltage(const uint8_t packet[], int &millivolts);

bool servoReplyTemperature(const uint8_t packet[], int &celsius);

bool servoReplyPosition(const uint8_t packet[], int &position);

int servoBuildMoveStop(uint8_t id, uint8_t out[]);

int servoAngleToPosition(float angleDeg);

float servoPositionToAngle(int position);
