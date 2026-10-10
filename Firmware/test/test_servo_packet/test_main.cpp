#include <math.h>
#include <unity.h>
#include "servo_packet.h"

void setUp(void) {}
void tearDown(void) {}

void test_move_time_example_from_task(void) {
    uint8_t packet[SERVO_MAX_PACKET];
    uint8_t expected[10] = { 0x55, 0x55, 0x01, 0x07, 0x01, 0xF4, 0x01, 0xE8, 0x03, 0x16 };
    TEST_ASSERT_EQUAL_INT(10, servoBuildMoveTime(1, 500, 1000, packet));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, packet, 10);
}

void test_move_time_other_values(void) {
    uint8_t packet[SERVO_MAX_PACKET];
    uint8_t expectedA[10] = { 0x55, 0x55, 0x05, 0x07, 0x01, 0xE8, 0x03, 0x14, 0x00, 0xF3 };
    servoBuildMoveTime(5, 1000, 20, packet);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expectedA, packet, 10);

    uint8_t expectedB[10] = { 0x55, 0x55, 0x12, 0x07, 0x01, 0x00, 0x00, 0x30, 0x75, 0x40 };
    servoBuildMoveTime(18, 0, 30000, packet);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expectedB, packet, 10);
}

void test_move_time_cuts_bad_values(void) {
    uint8_t packet[SERVO_MAX_PACKET];
    uint8_t reference[SERVO_MAX_PACKET];

    servoBuildMoveTime(5, 1200, 20, packet);
    servoBuildMoveTime(5, 1000, 20, reference);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(reference, packet, 10);

    servoBuildMoveTime(18, -5, 40000, packet);
    servoBuildMoveTime(18, 0, 30000, reference);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(reference, packet, 10);
}

void test_read_requests(void) {
    uint8_t packet[SERVO_MAX_PACKET];
    uint8_t pos[6] = { 0x55, 0x55, 0x01, 0x03, 0x1C, 0xDF };
    uint8_t vin[6] = { 0x55, 0x55, 0x01, 0x03, 0x1B, 0xE0 };
    uint8_t temp[6] = { 0x55, 0x55, 0x01, 0x03, 0x1A, 0xE1 };
    uint8_t broadcastPos[6] = { 0x55, 0x55, 0xFE, 0x03, 0x1C, 0xE2 };

    TEST_ASSERT_EQUAL_INT(6, servoBuildRead(1, SERVO_POS_READ, packet));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(pos, packet, 6);
    servoBuildRead(1, SERVO_VIN_READ, packet);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(vin, packet, 6);
    servoBuildRead(1, SERVO_TEMP_READ, packet);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(temp, packet, 6);
    servoBuildRead(SERVO_BROADCAST_ID, SERVO_POS_READ, packet);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(broadcastPos, packet, 6);
}

void test_command_numbers_match_protocol(void) {
    TEST_ASSERT_EQUAL_UINT8(1, SERVO_MOVE_TIME_WRITE);
    TEST_ASSERT_EQUAL_UINT8(26, SERVO_TEMP_READ);
    TEST_ASSERT_EQUAL_UINT8(27, SERVO_VIN_READ);
    TEST_ASSERT_EQUAL_UINT8(28, SERVO_POS_READ);
    TEST_ASSERT_EQUAL_UINT8(254, SERVO_BROADCAST_ID);
}

void test_checksum_changes_with_any_byte(void) {
    uint8_t packet[SERVO_MAX_PACKET];
    servoBuildMoveTime(1, 500, 1000, packet);
    uint8_t original = servoChecksum(packet);
    TEST_ASSERT_EQUAL_HEX8(0x16, original);
    for (int i = 2; i < 9; i++) {
        packet[i] = packet[i] + 1;
        TEST_ASSERT_NOT_EQUAL(original, servoChecksum(packet));
        packet[i] = packet[i] - 1;
    }
}

void test_id_command_numbers_match_protocol(void) {
    TEST_ASSERT_EQUAL_UINT8(13, SERVO_ID_WRITE);
    TEST_ASSERT_EQUAL_UINT8(14, SERVO_ID_READ);
}

void test_id_write_packet(void) {
    uint8_t packet[SERVO_MAX_PACKET];
    uint8_t to7[7] = { 0x55, 0x55, 0x01, 0x04, 0x0D, 0x07, 0xE6 };
    uint8_t to18[7] = { 0x55, 0x55, 0x01, 0x04, 0x0D, 0x12, 0xDB };
    TEST_ASSERT_EQUAL_INT(7, servoBuildIdWrite(1, 7, packet));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(to7, packet, 7);
    servoBuildIdWrite(1, 18, packet);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(to18, packet, 7);
}

void test_id_read_packet(void) {
    uint8_t packet[SERVO_MAX_PACKET];
    uint8_t broadcast[6] = { 0x55, 0x55, 0xFE, 0x03, 0x0E, 0xF0 };
    uint8_t to7[6] = { 0x55, 0x55, 0x07, 0x03, 0x0E, 0xE7 };
    TEST_ASSERT_EQUAL_INT(6, servoBuildIdRead(SERVO_BROADCAST_ID, packet));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(broadcast, packet, 6);
    servoBuildIdRead(7, packet);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(to7, packet, 6);
}

static int feedAll(ServoReceiver &r, const uint8_t bytes[], int count) {
    for (int i = 0; i < count; i++) {
        if (servoReceiverFeed(r, bytes[i])) {
            return i;
        }
    }
    return -1;
}

void test_receiver_gets_id_answer(void) {
    uint8_t answer[7] = { 0x55, 0x55, 0x07, 0x04, 0x0E, 0x07, 0xDF };
    ServoReceiver r;
    servoReceiverReset(r);
    TEST_ASSERT_EQUAL_INT(6, feedAll(r, answer, 7));
    uint8_t id = 0;
    TEST_ASSERT_TRUE(servoReplyId(r.packet, id));
    TEST_ASSERT_EQUAL_UINT8(7, id);
}

void test_receiver_skips_garbage_before_header(void) {
    uint8_t bytes[10] = { 0x00, 0x55, 0x13, 0x55, 0x55, 0x01, 0x04, 0x0E, 0x01, 0xEB };
    ServoReceiver r;
    servoReceiverReset(r);
    TEST_ASSERT_EQUAL_INT(9, feedAll(r, bytes, 10));
    uint8_t id = 0;
    TEST_ASSERT_TRUE(servoReplyId(r.packet, id));
    TEST_ASSERT_EQUAL_UINT8(1, id);
}

void test_receiver_rejects_bad_checksum(void) {
    uint8_t damaged[7] = { 0x55, 0x55, 0x07, 0x04, 0x0E, 0x07, 0xDE };
    ServoReceiver r;
    servoReceiverReset(r);
    TEST_ASSERT_EQUAL_INT(-1, feedAll(r, damaged, 7));
    TEST_ASSERT_EQUAL_INT(0, r.count);
}

void test_receiver_rejects_impossible_length(void) {
    uint8_t bytes[5] = { 0x55, 0x55, 0x07, 0x20, 0x0E };
    ServoReceiver r;
    servoReceiverReset(r);
    TEST_ASSERT_EQUAL_INT(-1, feedAll(r, bytes, 5));
    TEST_ASSERT_EQUAL_INT(0, r.count);
}

void test_echo_of_request_is_not_an_id_answer(void) {
    uint8_t request[SERVO_MAX_PACKET];
    servoBuildIdRead(SERVO_BROADCAST_ID, request);
    ServoReceiver r;
    servoReceiverReset(r);
    TEST_ASSERT_EQUAL_INT(5, feedAll(r, request, 6));
    uint8_t id = 0;
    TEST_ASSERT_FALSE(servoReplyId(r.packet, id));
}

void test_receiver_gets_position_answer(void) {
    uint8_t answer[8] = { 0x55, 0x55, 0x03, 0x05, 0x1C, 0x64, 0x02, 0x75 };
    ServoReceiver r;
    servoReceiverReset(r);
    TEST_ASSERT_EQUAL_INT(7, feedAll(r, answer, 8));
    int position = 0;
    TEST_ASSERT_TRUE(servoReplyPosition(r.packet, position));
    TEST_ASSERT_EQUAL_INT(612, position);

    uint8_t negative[8] = { 0x55, 0x55, 0x03, 0x05, 0x1C, 0xEC, 0xFF, 0xF0 };
    servoReceiverReset(r);
    TEST_ASSERT_EQUAL_INT(7, feedAll(r, negative, 8));
    TEST_ASSERT_TRUE(servoReplyPosition(r.packet, position));
    TEST_ASSERT_EQUAL_INT(-20, position);
}

void test_voltage_answer_is_not_a_position(void) {
    uint8_t vin[8] = { 0x55, 0x55, 0x03, 0x05, SERVO_VIN_READ, 0xE8, 0x1C, 0x00 };
    int position = 12345;
    TEST_ASSERT_FALSE(servoReplyPosition(vin, position));
    TEST_ASSERT_EQUAL_INT(12345, position);
}

void test_angle_to_position(void) {
    TEST_ASSERT_EQUAL_INT(500, servoAngleToPosition(0.0f));
    TEST_ASSERT_EQUAL_INT(501, servoAngleToPosition(0.24f));
    TEST_ASSERT_EQUAL_INT(638, servoAngleToPosition(33.098f));
    TEST_ASSERT_EQUAL_INT(154, servoAngleToPosition(-83.101f));
    TEST_ASSERT_EQUAL_INT(1000, servoAngleToPosition(120.0f));
    TEST_ASSERT_EQUAL_INT(0, servoAngleToPosition(-120.0f));
    TEST_ASSERT_EQUAL_INT(1000, servoAngleToPosition(200.0f));
    TEST_ASSERT_EQUAL_INT(0, servoAngleToPosition(-200.0f));
    TEST_ASSERT_EQUAL_INT(500, servoAngleToPosition(NAN));
    TEST_ASSERT_EQUAL_INT(500, servoAngleToPosition(INFINITY));
}

void test_position_to_angle(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, servoPositionToAngle(500));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 120.0f, servoPositionToAngle(1000));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -120.0f, servoPositionToAngle(0));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 33.12f, servoPositionToAngle(638));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_move_time_example_from_task);
    RUN_TEST(test_move_time_other_values);
    RUN_TEST(test_move_time_cuts_bad_values);
    RUN_TEST(test_read_requests);
    RUN_TEST(test_command_numbers_match_protocol);
    RUN_TEST(test_checksum_changes_with_any_byte);
    RUN_TEST(test_id_command_numbers_match_protocol);
    RUN_TEST(test_id_write_packet);
    RUN_TEST(test_id_read_packet);
    RUN_TEST(test_receiver_gets_id_answer);
    RUN_TEST(test_receiver_skips_garbage_before_header);
    RUN_TEST(test_receiver_rejects_bad_checksum);
    RUN_TEST(test_receiver_rejects_impossible_length);
    RUN_TEST(test_echo_of_request_is_not_an_id_answer);
    RUN_TEST(test_receiver_gets_position_answer);
    RUN_TEST(test_voltage_answer_is_not_a_position);
    RUN_TEST(test_angle_to_position);
    RUN_TEST(test_position_to_angle);
    return UNITY_END();
}
