#include <unity.h>
#include "CoordinateMapper.h"

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_is_in_range_j1(void) {
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_YAW, 0.0f));
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_YAW, 180.0f));
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_YAW, -180.0f));
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_YAW, 181.0f));
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_YAW, -181.0f));
}

void test_is_in_range_j2(void) {
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_SHOULDER, 0.0f));
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_SHOULDER, 180.0f));
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_SHOULDER, 90.0f));
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_SHOULDER, -0.1f));
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_SHOULDER, 181.0f));
}

void test_is_in_range_j3(void) {
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_ELBOW, 0.0f));
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_ELBOW, 150.0f));
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_ELBOW, -1.0f));
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_ELBOW, 151.0f));
}

void test_is_in_range_j4(void) {
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_WRIST_ROLL, 0.0f));
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_WRIST_ROLL, 180.0f));
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_WRIST_ROLL, -180.0f));
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_WRIST_ROLL, 181.0f));
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_WRIST_ROLL, -181.0f));
}

void test_is_in_range_j5(void) {
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_WRIST_PITCH, 0.0f));
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_WRIST_PITCH, 90.0f));
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_WRIST_PITCH, -90.0f));
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_WRIST_PITCH, 91.0f));
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_WRIST_PITCH, -91.0f));
}

void test_is_in_range_j6(void) {
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_GRIPPER, 0.0f));
    TEST_ASSERT_TRUE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_GRIPPER, 90.0f));
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_GRIPPER, -1.0f));
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(CoordinateMapper::JOINT_GRIPPER, 91.0f));
}

void test_steps_from_degrees(void) {
    // With default SPR=200, microstepping=16, gear_ratio=1, full rev = 3200 steps
    // 360 deg = 3200 steps
    // 180 deg = 1600 steps
    // 90 deg = 800 steps
    // 0 deg = 0 steps

    // Zero degree input
    TEST_ASSERT_EQUAL_FLOAT(0.0f, CoordinateMapper::steps_from_degrees(CoordinateMapper::JOINT_YAW, 0.0f));

    // Positive angles
    TEST_ASSERT_EQUAL_FLOAT(1600.0f, CoordinateMapper::steps_from_degrees(CoordinateMapper::JOINT_YAW, 180.0f));
    TEST_ASSERT_EQUAL_FLOAT(800.0f, CoordinateMapper::steps_from_degrees(CoordinateMapper::JOINT_SHOULDER, 90.0f));
    TEST_ASSERT_EQUAL_FLOAT(800.0f, CoordinateMapper::steps_from_degrees(CoordinateMapper::JOINT_ELBOW, 90.0f));
    TEST_ASSERT_EQUAL_FLOAT(400.0f, CoordinateMapper::steps_from_degrees(CoordinateMapper::JOINT_WRIST_ROLL, 45.0f));
    TEST_ASSERT_EQUAL_FLOAT(800.0f, CoordinateMapper::steps_from_degrees(CoordinateMapper::JOINT_WRIST_PITCH, 90.0f));
    TEST_ASSERT_EQUAL_FLOAT(800.0f, CoordinateMapper::steps_from_degrees(CoordinateMapper::JOINT_GRIPPER, 90.0f));

    // Negative angle returns negative steps for joints that support it
    TEST_ASSERT_EQUAL_FLOAT(-1600.0f, CoordinateMapper::steps_from_degrees(CoordinateMapper::JOINT_YAW, -180.0f));
    TEST_ASSERT_EQUAL_FLOAT(-800.0f, CoordinateMapper::steps_from_degrees(CoordinateMapper::JOINT_WRIST_PITCH, -90.0f));
}

void test_unknown_joint(void) {
    TEST_ASSERT_FALSE(CoordinateMapper::is_in_range(99, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, CoordinateMapper::steps_from_degrees(99, 90.0f));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_is_in_range_j1);
    RUN_TEST(test_is_in_range_j2);
    RUN_TEST(test_is_in_range_j3);
    RUN_TEST(test_is_in_range_j4);
    RUN_TEST(test_is_in_range_j5);
    RUN_TEST(test_is_in_range_j6);
    RUN_TEST(test_steps_from_degrees);
    RUN_TEST(test_unknown_joint);

    return UNITY_END();
}
