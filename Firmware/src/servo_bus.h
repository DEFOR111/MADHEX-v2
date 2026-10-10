#pragma once

#include <stdint.h>
#include "servo_packet.h"

void servoBusBegin();

void servoBusSend(int bus, const uint8_t packet[], int length);

void servoBusWaitSent(int bus);

void servoMoveTime(int bus, uint8_t id, float angleDeg, int timeMs);

enum ServoReplyStatus {
    SERVO_REPLY_OK,
    SERVO_NO_ECHO,
    SERVO_ECHO_WRONG,
    SERVO_NO_REPLY,
    SERVO_PENDING
};

struct ServoPendingRequest {
    bool active;
    int bus;
    uint8_t request[SERVO_MAX_PACKET];
    int requestLength;
    int echoed;
    ServoReceiver receiver;
    unsigned long startMs;
};

void servoRequestStart(ServoPendingRequest &p, int bus, const uint8_t request[], int requestLength);

ServoReplyStatus servoRequestPoll(ServoPendingRequest &p, unsigned long timeoutMs);

ServoReplyStatus servoBusRequest(int bus, const uint8_t request[], int requestLength,
                                 uint8_t reply[], unsigned long timeoutMs);

const char *servoReplyStatusText(ServoReplyStatus status);
