#include <math.h>
#include "servo_packet.h"

uint8_t servoChecksum(const uint8_t packet[]) {
    uint8_t length = packet[3];
    unsigned int sum = 0;
    for (int i = 2; i < length + 2; i++) {
        sum = sum + packet[i];
    }
    return (uint8_t)(~sum & 0xFF);
}

int servoBuildMoveTime(uint8_t id, int position, int timeMs, uint8_t out[]) {
    if (position < SERVO_POSITION_MIN) {
        position = SERVO_POSITION_MIN;
    }
    if (position > SERVO_POSITION_MAX) {
        position = SERVO_POSITION_MAX;
    }
    if (timeMs < 0) {
        timeMs = 0;
    }
    if (timeMs > SERVO_TIME_MAX_MS) {
        timeMs = SERVO_TIME_MAX_MS;
    }

    out[0] = SERVO_HEADER;
    out[1] = SERVO_HEADER;
    out[2] = id;
    out[3] = 7;
    out[4] = SERVO_MOVE_TIME_WRITE;
    out[5] = position & 0xFF;
    out[6] = (position >> 8) & 0xFF;
    out[7] = timeMs & 0xFF;
    out[8] = (timeMs >> 8) & 0xFF;
    out[9] = servoChecksum(out);
    return 10;
}

int servoBuildRead(uint8_t id, uint8_t command, uint8_t out[]) {
    out[0] = SERVO_HEADER;
    out[1] = SERVO_HEADER;
    out[2] = id;
    out[3] = 3;
    out[4] = command;
    out[5] = servoChecksum(out);
    return 6;
}

int servoBuildIdWrite(uint8_t id, uint8_t newId, uint8_t out[]) {
    out[0] = SERVO_HEADER;
    out[1] = SERVO_HEADER;
    out[2] = id;
    out[3] = 4;
    out[4] = SERVO_ID_WRITE;
    out[5] = newId;
    out[6] = servoChecksum(out);
    return 7;
}

int servoBuildIdRead(uint8_t id, uint8_t out[]) {
    return servoBuildRead(id, SERVO_ID_READ, out);
}

void servoReceiverReset(ServoReceiver &r) {
    r.count = 0;
}

bool servoReceiverFeed(ServoReceiver &r, uint8_t byte) {
    if (r.count < 2) {
        if (byte == SERVO_HEADER) {
            r.packet[r.count] = byte;
            r.count++;
        } else {
            r.count = 0;
        }
        return false;
    }

    r.packet[r.count] = byte;
    r.count++;

    if (r.count == 4) {
        int length = r.packet[3];
        if (length < 3 || length + 3 > SERVO_MAX_PACKET) {
            servoReceiverReset(r);
            return false;
        }
    }

    if (r.count >= 4 && r.count == r.packet[3] + 3) {
        if (servoChecksum(r.packet) == r.packet[r.count - 1]) {
            return true;
        }
        servoReceiverReset(r);
    }
    return false;
}

bool servoReplyId(const uint8_t packet[], uint8_t &id) {
    if (packet[3] != 4 || packet[4] != SERVO_ID_READ) {
        return false;
    }
    id = packet[5];
    return true;
}

bool servoReplyVoltage(const uint8_t packet[], int &millivolts) {
    if (packet[3] != 5 || packet[4] != SERVO_VIN_READ) {
        return false;
    }
    millivolts = packet[5] | (packet[6] << 8);
    return true;
}

bool servoReplyTemperature(const uint8_t packet[], int &celsius) {
    if (packet[3] != 4 || packet[4] != SERVO_TEMP_READ) {
        return false;
    }
    celsius = packet[5];
    return true;
}

bool servoReplyPosition(const uint8_t packet[], int &position) {
    if (packet[3] != 5 || packet[4] != SERVO_POS_READ) {
        return false;
    }
    position = (int16_t)(packet[5] | (packet[6] << 8));
    return true;
}

int servoBuildMoveStop(uint8_t id, uint8_t out[]) {
    return servoBuildRead(id, SERVO_MOVE_STOP, out);
}

int servoAngleToPosition(float angleDeg) {
    if (!isfinite(angleDeg)) {
        return SERVO_POSITION_ZERO;
    }
    long position = lroundf(SERVO_POSITION_ZERO + angleDeg / SERVO_DEG_PER_UNIT);
    if (position < SERVO_POSITION_MIN) {
        position = SERVO_POSITION_MIN;
    }
    if (position > SERVO_POSITION_MAX) {
        position = SERVO_POSITION_MAX;
    }
    return (int)position;
}

float servoPositionToAngle(int position) {
    return (position - SERVO_POSITION_ZERO) * SERVO_DEG_PER_UNIT;
}
