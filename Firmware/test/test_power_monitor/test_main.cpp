#include <unity.h>
#include "config.h"
#include "power_monitor.h"

void setUp(void) {}
void tearDown(void) {}

void test_calibration_2560(void) {
    TEST_ASSERT_EQUAL_UINT16(2560, ina226Calibration(0.001f, 0.002f));
    TEST_ASSERT_EQUAL_UINT16(2560, ina226Calibration(INA226_CURRENT_LSB_A, INA226_SHUNT_OHM));
    TEST_ASSERT_EQUAL_HEX16(0x0A00, ina226Calibration(INA226_CURRENT_LSB_A, INA226_SHUNT_OHM));
}

void test_config_register(void) {
    TEST_ASSERT_EQUAL_HEX16(0x4127, ina226ConfigValue(0, 4, 4));
    TEST_ASSERT_EQUAL_HEX16(0x4527, ina226ConfigValue(INA226_AVG_CODE, INA226_BUS_CT_CODE,
                                                      INA226_SHUNT_CT_CODE));
}

void test_bus_voltage(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 11980.0f, ina226BusVoltageMv(0x2570));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 7420.0f, ina226BusVoltageMv(5936));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, ina226BusVoltageMv(0));
}

void test_shunt_voltage(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 20000.0f, ina226ShuntVoltageUv(0x1F40));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -2.5f, ina226ShuntVoltageUv(0xFFFF));
}

void test_current(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10000.0f, ina226CurrentMa(0x2710, 0.001f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1500.0f, ina226CurrentMa(1500, 0.001f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -1.0f, ina226CurrentMa(0xFFFF, 0.001f));
}

void test_power(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 119800.0f, ina226PowerMw(0x12B8, 0.001f));
}

void test_adc_backup(void) {
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 8399.0f, adcToBatteryMv(2084.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, adcToBatteryMv(0.0f));
}

void test_plain_thresholds(void) {
    BatteryState start = BATTERY_NOT_CONNECTED;
    TEST_ASSERT_EQUAL_STRING("NOT_CONNECTED", batteryStateName(batteryState(50.0f, start)));
    TEST_ASSERT_EQUAL_STRING("CRITICAL", batteryStateName(batteryState(6300.0f, start)));
    TEST_ASSERT_EQUAL_STRING("LOW", batteryStateName(batteryState(6700.0f, start)));
    TEST_ASSERT_EQUAL_STRING("OK", batteryStateName(batteryState(7400.0f, start)));
}

void test_hysteresis(void) {
    BatteryState s = batteryState(7400.0f, BATTERY_NOT_CONNECTED);
    s = batteryState(6750.0f, s);
    TEST_ASSERT_EQUAL_STRING("LOW", batteryStateName(s));
    s = batteryState(6900.0f, s);
    TEST_ASSERT_EQUAL_STRING("LOW", batteryStateName(s));
    s = batteryState(7000.0f, s);
    TEST_ASSERT_EQUAL_STRING("OK", batteryStateName(s));

    s = batteryState(6350.0f, s);
    TEST_ASSERT_EQUAL_STRING("CRITICAL", batteryStateName(s));
    s = batteryState(6500.0f, s);
    TEST_ASSERT_EQUAL_STRING("CRITICAL", batteryStateName(s));
    s = batteryState(6650.0f, s);
    TEST_ASSERT_EQUAL_STRING("LOW", batteryStateName(s));
}

void test_no_flicker_around_threshold(void) {
    BatteryState s = batteryState(6790.0f, BATTERY_OK);
    TEST_ASSERT_EQUAL_STRING("LOW", batteryStateName(s));
    float bounce[6] = { 6810.0f, 6790.0f, 6850.0f, 6780.0f, 6900.0f, 6820.0f };
    for (int i = 0; i < 6; i++) {
        s = batteryState(bounce[i], s);
        TEST_ASSERT_EQUAL_STRING("LOW", batteryStateName(s));
    }
}

void test_not_connected_is_immediate(void) {
    BatteryState s = batteryState(6300.0f, BATTERY_NOT_CONNECTED);
    s = batteryState(20.0f, s);
    TEST_ASSERT_EQUAL_STRING("NOT_CONNECTED", batteryStateName(s));
    s = batteryState(8300.0f, s);
    TEST_ASSERT_EQUAL_STRING("OK", batteryStateName(s));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_calibration_2560);
    RUN_TEST(test_config_register);
    RUN_TEST(test_bus_voltage);
    RUN_TEST(test_shunt_voltage);
    RUN_TEST(test_current);
    RUN_TEST(test_power);
    RUN_TEST(test_adc_backup);
    RUN_TEST(test_plain_thresholds);
    RUN_TEST(test_hysteresis);
    RUN_TEST(test_no_flicker_around_threshold);
    RUN_TEST(test_not_connected_is_immediate);
    return UNITY_END();
}
