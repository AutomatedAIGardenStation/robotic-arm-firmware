#include <unity.h>
#include "CoordinateMapper.h"

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_is_in_range_x(void) {
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::AXIS_X, 0.0f));
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::AXIS_X, 500.0f));
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::AXIS_X, 1000.0f));
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(CoordinateMapper::AXIS_X, -1.0f));
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(CoordinateMapper::AXIS_X, 1001.0f));
}

void test_is_in_range_y(void) {
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::AXIS_Y, 0.0f));
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::AXIS_Y, 1000.0f));
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(CoordinateMapper::AXIS_Y, -0.1f));
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(CoordinateMapper::AXIS_Y, 1001.0f));
}

void test_is_in_range_z(void) {
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::AXIS_Z, 0.0f));
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::AXIS_Z, 500.0f));
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(CoordinateMapper::AXIS_Z, -1.0f));
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(CoordinateMapper::AXIS_Z, 501.0f));
}

void test_is_in_range_pitch(void) {
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::WRIST_PITCH, 0.0f));
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::WRIST_PITCH, 90.0f));
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::WRIST_PITCH, -90.0f));
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(CoordinateMapper::WRIST_PITCH, 91.0f));
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(CoordinateMapper::WRIST_PITCH, -91.0f));
}

void test_is_in_range_roll(void) {
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::WRIST_ROLL, 0.0f));
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::WRIST_ROLL, 90.0f));
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::WRIST_ROLL, -90.0f));
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(CoordinateMapper::WRIST_ROLL, 91.0f));
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(CoordinateMapper::WRIST_ROLL, -91.0f));
}

void test_steps_from_mm(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, CoordinateMapper::steps_from_mm(CoordinateMapper::AXIS_X, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(400.0f, CoordinateMapper::steps_from_mm(CoordinateMapper::AXIS_X, 10.0f));
    TEST_ASSERT_EQUAL_FLOAT(4000.0f, CoordinateMapper::steps_from_mm(CoordinateMapper::AXIS_Y, 100.0f));
    TEST_ASSERT_EQUAL_FLOAT(2000.0f, CoordinateMapper::steps_from_mm(CoordinateMapper::AXIS_Z, 50.0f));
}

void test_steps_from_degrees(void) {
    // Zero degree input
    TEST_ASSERT_EQUAL_FLOAT(0.0f, CoordinateMapper::steps_from_degrees(CoordinateMapper::WRIST_PITCH, 0.0f));

    // Positive angles (3200 steps/rev = 360 deg -> 800 steps/90 deg)
    TEST_ASSERT_EQUAL_FLOAT(800.0f, CoordinateMapper::steps_from_degrees(CoordinateMapper::WRIST_PITCH, 90.0f));
    TEST_ASSERT_EQUAL_FLOAT(800.0f, CoordinateMapper::steps_from_degrees(CoordinateMapper::WRIST_ROLL, 90.0f));

    // Negative angle returns negative steps for joints that support it
    TEST_ASSERT_EQUAL_FLOAT(-800.0f, CoordinateMapper::steps_from_degrees(CoordinateMapper::WRIST_PITCH, -90.0f));
}

void test_unknown_axis(void) {
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(99, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, CoordinateMapper::steps_from_degrees(99, 90.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, CoordinateMapper::steps_from_mm(99, 90.0f));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_is_in_range_x);
    RUN_TEST(test_is_in_range_y);
    RUN_TEST(test_is_in_range_z);
    RUN_TEST(test_is_in_range_pitch);
    RUN_TEST(test_is_in_range_roll);
    RUN_TEST(test_steps_from_mm);
    RUN_TEST(test_steps_from_degrees);
    RUN_TEST(test_unknown_axis);

    return UNITY_END();
}
