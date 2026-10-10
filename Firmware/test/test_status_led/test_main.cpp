#include <unity.h>
#include "config.h"
#include "status_color.h"

void setUp(void) {}
void tearDown(void) {}

static LedInputs normal(RobotState state) {
    LedInputs in;
    in.state = state;
    in.safetyFault = false;
    in.batteryValid = true;
    in.battery = BATTERY_OK;
    in.servoOutputOff = false;
    return in;
}

static const char *colorAt(const LedInputs &in, unsigned long ms) {
    return ledColorName(statusLedColor(in, ms));
}

void test_state_colors(void) {
    TEST_ASSERT_EQUAL_STRING("dim blue", colorAt(normal(STATE_SIT), 0));
    TEST_ASSERT_EQUAL_STRING("green", colorAt(normal(STATE_STAND), 0));
    TEST_ASSERT_EQUAL_STRING("cyan", colorAt(normal(STATE_WALK), 0));
    TEST_ASSERT_EQUAL_STRING("cyan", colorAt(normal(STATE_STANDING_UP), 0));
    TEST_ASSERT_EQUAL_STRING("cyan", colorAt(normal(STATE_SITTING_DOWN), 0));
}

void test_battery_low_blinks_yellow(void) {
    LedInputs in = normal(STATE_WALK);
    in.battery = BATTERY_LOW;
    TEST_ASSERT_EQUAL_STRING("yellow", colorAt(in, 0));
    TEST_ASSERT_EQUAL_STRING("yellow", colorAt(in, 499));
    TEST_ASSERT_EQUAL_STRING("off", colorAt(in, 500));
    TEST_ASSERT_EQUAL_STRING("off", colorAt(in, 999));
    TEST_ASSERT_EQUAL_STRING("yellow", colorAt(in, 1000));
}

void test_errors_blink_red(void) {
    LedInputs estop = normal(STATE_ESTOP);
    TEST_ASSERT_EQUAL_STRING("red", colorAt(estop, 0));
    TEST_ASSERT_EQUAL_STRING("red", colorAt(estop, 199));
    TEST_ASSERT_EQUAL_STRING("off", colorAt(estop, 200));
    TEST_ASSERT_EQUAL_STRING("red", colorAt(estop, 400));

    LedInputs fault = normal(STATE_SITTING_DOWN);
    fault.safetyFault = true;
    TEST_ASSERT_EQUAL_STRING("red", colorAt(fault, 50));
    TEST_ASSERT_EQUAL_STRING("off", colorAt(fault, 250));

    LedInputs critical = normal(STATE_STAND);
    critical.battery = BATTERY_CRITICAL;
    TEST_ASSERT_EQUAL_STRING("red", colorAt(critical, 50));
}

void test_usb_only_is_purple(void) {
    LedInputs in = normal(STATE_SIT);
    in.battery = BATTERY_NOT_CONNECTED;
    TEST_ASSERT_EQUAL_STRING("purple", colorAt(in, 0));
    TEST_ASSERT_EQUAL_STRING("purple", colorAt(in, 777));
}

void test_priority(void) {
    LedInputs in = normal(STATE_ESTOP);
    in.battery = BATTERY_NOT_CONNECTED;
    TEST_ASSERT_EQUAL_STRING("red", colorAt(in, 0));

    in = normal(STATE_WALK);
    in.battery = BATTERY_LOW;
    TEST_ASSERT_EQUAL_STRING("yellow", colorAt(in, 0));

    in = normal(STATE_STAND);
    in.batteryValid = false;
    in.battery = BATTERY_NOT_CONNECTED;
    TEST_ASSERT_EQUAL_STRING("green", colorAt(in, 0));
}

void test_white_flash_when_output_off(void) {
    LedInputs in = normal(STATE_SIT);
    in.servoOutputOff = true;
    TEST_ASSERT_EQUAL_STRING("white", colorAt(in, 0));
    TEST_ASSERT_EQUAL_STRING("white", colorAt(in, 99));
    TEST_ASSERT_EQUAL_STRING("dim blue", colorAt(in, 100));
    TEST_ASSERT_EQUAL_STRING("dim blue", colorAt(in, 1999));
    TEST_ASSERT_EQUAL_STRING("white", colorAt(in, 2050));

    in.battery = BATTERY_NOT_CONNECTED;
    TEST_ASSERT_EQUAL_STRING("white", colorAt(in, 4000));
    TEST_ASSERT_EQUAL_STRING("purple", colorAt(in, 4100));
}

static int changesIn10s(const LedInputs &in) {
    int changes = 0;
    LedColor last = statusLedColor(in, 0);
    for (unsigned long ms = 1; ms < 10000; ms++) {
        LedColor c = statusLedColor(in, ms);
        if (!ledColorEqual(c, last)) {
            changes++;
            last = c;
        }
    }
    return changes;
}

void test_led_updates_are_rare(void) {
    TEST_ASSERT_EQUAL_INT(0, changesIn10s(normal(STATE_WALK)));

    LedInputs off = normal(STATE_SIT);
    off.servoOutputOff = true;
    TEST_ASSERT_EQUAL_INT(9, changesIn10s(off));

    LedInputs low = normal(STATE_STAND);
    low.battery = BATTERY_LOW;
    TEST_ASSERT_EQUAL_INT(19, changesIn10s(low));

    LedInputs estop = normal(STATE_ESTOP);
    TEST_ASSERT_EQUAL_INT(49, changesIn10s(estop));
}

void test_pattern_for_the_web_page(void) {
    LedPattern p = statusLedPattern(normal(STATE_STAND));
    TEST_ASSERT_EQUAL_STRING("green", ledColorName(p.color));
    TEST_ASSERT_EQUAL_UINT32(0, p.blinkMs);
    TEST_ASSERT_FALSE(p.whiteFlash);

    LedInputs low = normal(STATE_WALK);
    low.battery = BATTERY_LOW;
    low.servoOutputOff = true;
    p = statusLedPattern(low);
    TEST_ASSERT_EQUAL_STRING("yellow", ledColorName(p.color));
    TEST_ASSERT_EQUAL_UINT32(LED_BLINK_SLOW_MS, p.blinkMs);
    TEST_ASSERT_TRUE(p.whiteFlash);

    p = statusLedPattern(normal(STATE_ESTOP));
    TEST_ASSERT_EQUAL_STRING("red", ledColorName(p.color));
    TEST_ASSERT_EQUAL_UINT32(LED_BLINK_FAST_MS, p.blinkMs);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_state_colors);
    RUN_TEST(test_battery_low_blinks_yellow);
    RUN_TEST(test_errors_blink_red);
    RUN_TEST(test_usb_only_is_purple);
    RUN_TEST(test_priority);
    RUN_TEST(test_white_flash_when_output_off);
    RUN_TEST(test_led_updates_are_rare);
    RUN_TEST(test_pattern_for_the_web_page);
    return UNITY_END();
}
