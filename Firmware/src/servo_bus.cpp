#include <Arduino.h>
#include "driver/uart.h"
#include "config.h"
#include "servo_bus.h"
#include "servo_packet.h"

static HardwareSerial &busPort(int bus) {
    if (bus == 1) {
        return Serial1;
    }
    return Serial2;
}

static void dropReceived(HardwareSerial &port) {
    while (port.available() > 0) {
        port.read();
    }
}

void servoBusBegin() {
    Serial1.begin(SERVO_BAUD, SERIAL_8N1, SERVO_BUS1_RX_PIN, SERVO_BUS1_TX_PIN);
    uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_TXD_INV);

    Serial2.begin(SERVO_BAUD, SERIAL_8N1, SERVO_BUS2_RX_PIN, SERVO_BUS2_TX_PIN);
    uart_set_line_inverse(UART_NUM_2, UART_SIGNAL_TXD_INV);
}

void servoBusSend(int bus, const uint8_t packet[], int length) {
    HardwareSerial &port = busPort(bus);
    dropReceived(port);
    port.write(packet, length);
}

void servoBusWaitSent(int bus) {
    busPort(bus).flush();
}

void servoMoveTime(int bus, uint8_t id, float angleDeg, int timeMs) {
    uint8_t packet[SERVO_MAX_PACKET];
    int length = servoBuildMoveTime(id, servoAngleToPosition(angleDeg), timeMs, packet);
    servoBusSend(bus, packet, length);
}

void servoRequestStart(ServoPendingRequest &p, int bus, const uint8_t request[], int requestLength) {
    p.bus = bus;
    p.requestLength = requestLength;
    for (int i = 0; i < requestLength; i++) {
        p.request[i] = request[i];
    }
    p.echoed = 0;
    servoReceiverReset(p.receiver);

    HardwareSerial &port = busPort(bus);
    dropReceived(port);
    port.write(request, requestLength);
    p.startMs = millis();
    p.active = true;
}

ServoReplyStatus servoRequestPoll(ServoPendingRequest &p, unsigned long timeoutMs) {
    if (!p.active) {
        return SERVO_NO_REPLY;
    }
    HardwareSerial &port = busPort(p.bus);
    while (port.available() > 0) {
        uint8_t b = (uint8_t)port.read();
        if (p.echoed < p.requestLength) {
            if (b != p.request[p.echoed]) {
                p.active = false;
                return SERVO_ECHO_WRONG;
            }
            p.echoed++;
        } else if (servoReceiverFeed(p.receiver, b)) {
            if (p.receiver.packet[4] == p.request[4]) {
                p.active = false;
                return SERVO_REPLY_OK;
            }
            servoReceiverReset(p.receiver);
        }
    }
    if (millis() - p.startMs > timeoutMs) {
        p.active = false;
        return (p.echoed < p.requestLength) ? SERVO_NO_ECHO : SERVO_NO_REPLY;
    }
    return SERVO_PENDING;
}

ServoReplyStatus servoBusRequest(int bus, const uint8_t request[], int requestLength,
                                 uint8_t reply[], unsigned long timeoutMs) {
    ServoPendingRequest p;
    servoRequestStart(p, bus, request, requestLength);
    ServoReplyStatus status = SERVO_PENDING;
    while (status == SERVO_PENDING) {
        status = servoRequestPoll(p, timeoutMs);
    }
    if (status == SERVO_REPLY_OK) {
        for (int i = 0; i < p.receiver.count; i++) {
            reply[i] = p.receiver.packet[i];
        }
    }
    return status;
}

const char *servoReplyStatusText(ServoReplyStatus status) {
    if (status == SERVO_REPLY_OK) {
        return "ok";
    }
    if (status == SERVO_NO_ECHO) {
        return "no echo (check bus wiring / TX)";
    }
    if (status == SERVO_ECHO_WRONG) {
        return "echo wrong (check TX inversion)";
    }
    if (status == SERVO_PENDING) {
        return "waiting";
    }
    return "no reply";
}
