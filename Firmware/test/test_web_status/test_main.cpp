#include <unity.h>
#include <string.h>
#include "web_status.h"

void setUp(void) {}
void tearDown(void) {}

static WebStatus example() {
    WebStatus s;
    s.state = "WALK";
    s.gait = "ripple";
    s.servos = "ON";
    s.heightMm = 10.0f;
    s.heightMinMm = -20.0f;
    s.heightMaxMm = 30.0f;
    s.levelOn = true;
    s.batteryValid = true;
    s.batteryV = 7.42f;
    s.batteryState = "OK";
    s.currentValid = true;
    s.currentA = 1.5f;
    s.imuValid = true;
    s.rollDeg = -1.25f;
    s.pitchDeg = 0.5f;
    s.led.color = LED_CYAN;
    s.led.blinkMs = 0;
    s.led.whiteFlash = false;
    s.loop.updatesPerSecond = 50;
    s.loop.maxLoopUs = 812;
    s.loop.maxGapUs = 20400;
    return s;
}

void test_status_json(void) {
    char out[512];
    int n = webStatusJson(example(), out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING(
        "{\"type\":\"status\",\"state\":\"WALK\",\"gait\":\"ripple\",\"servos\":\"ON\","
        "\"height\":10,\"height_min\":-20,\"height_max\":30,\"level\":true,"
        "\"batt_v\":7.42,\"batt_state\":\"OK\",\"current_a\":1.500,\"roll\":-1.2,\"pitch\":0.5,"
        "\"led\":{\"color\":\"#00ffff\",\"blink_ms\":0,\"flash\":false},"
        "\"loop\":{\"ups\":50,\"max_loop_us\":812,\"max_gap_ms\":20.4}}", out);
    TEST_ASSERT_EQUAL_INT((int)strlen(out), n);
}

void test_status_json_unknown_values(void) {
    WebStatus s = example();
    s.batteryValid = false;
    s.currentValid = false;
    s.imuValid = false;
    s.led.color = LED_RED;
    s.led.blinkMs = 400;
    s.led.whiteFlash = true;
    char out[512];
    TEST_ASSERT_TRUE(webStatusJson(s, out, sizeof(out)) > 0);
    TEST_ASSERT_NOT_NULL(strstr(out, "\"batt_v\":null,\"batt_state\":null,\"current_a\":null,\"roll\":null,\"pitch\":null,"));
    TEST_ASSERT_NOT_NULL(strstr(out, "\"led\":{\"color\":\"#ff0000\",\"blink_ms\":400,\"flash\":true}"));
}

void test_status_json_does_not_overflow(void) {
    char out[64];
    memset(out, 'x', sizeof(out));
    TEST_ASSERT_EQUAL_INT(0, webStatusJson(example(), out, 40));
    TEST_ASSERT_EQUAL_CHAR('x', out[40]);
}

void test_reply_json_escapes(void) {
    char out[128];
    int n = webReplyJson("error: use 'walk' \"now\"\r\npath C:\\x\tend", out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("{\"type\":\"reply\",\"text\":\"error: use 'walk' \\\"now\\\"\\npath C:\\\\xend\"}", out);
    TEST_ASSERT_EQUAL_INT((int)strlen(out), n);
    TEST_ASSERT_EQUAL_INT(0, webReplyJson("a long answer that does not fit", out, 30));
    webReplyJson("up: standing up\r\n", out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("{\"type\":\"reply\",\"text\":\"up: standing up\"}", out);
}

void test_dead_man(void) {
    TEST_ASSERT_FALSE(webLinkLost(true, 499, 500));
    TEST_ASSERT_FALSE(webLinkLost(true, 500, 500));
    TEST_ASSERT_TRUE(webLinkLost(true, 501, 500));
    TEST_ASSERT_FALSE(webLinkLost(false, 5000, 500));
}

void test_walk_commands(void) {
    TEST_ASSERT_TRUE(webIsWalkCommand("walk 0.30 -0.80 0.00"));
    TEST_ASSERT_TRUE(webIsWalkCommand("walk"));
    TEST_ASSERT_TRUE(webIsWalkCommand("w"));
    TEST_ASSERT_TRUE(webIsWalkCommand("  q"));
    TEST_ASSERT_FALSE(webIsWalkCommand("x"));
    TEST_ASSERT_FALSE(webIsWalkCommand("wave"));
    TEST_ASSERT_FALSE(webIsWalkCommand("walking"));
    TEST_ASSERT_FALSE(webIsWalkCommand("stop"));
    TEST_ASSERT_FALSE(webIsWalkCommand(""));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_status_json);
    RUN_TEST(test_status_json_unknown_values);
    RUN_TEST(test_status_json_does_not_overflow);
    RUN_TEST(test_reply_json_escapes);
    RUN_TEST(test_dead_man);
    RUN_TEST(test_walk_commands);
    return UNITY_END();
}
