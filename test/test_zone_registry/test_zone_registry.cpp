#include <unity.h>
#include "../../lib/motion/ZoneRegistry.h"
#include "../../config/Config.h"

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_lookup_hit(void) {
    float angles[6];
    bool result = resolve_zone("home", angles);
    TEST_ASSERT_TRUE(result);
    float expected_home[] = ZONE_HOME_ANGLES;
    for (int i = 0; i < 6; i++) {
        TEST_ASSERT_EQUAL_FLOAT(expected_home[i], angles[i]);
    }
}

void test_lookup_miss(void) {
    float angles[6];
    bool result = resolve_zone("unknown", angles);
    TEST_ASSERT_FALSE(result);
}

void test_all_default_zones(void) {
    float angles[6];

    // inspect
    TEST_ASSERT_TRUE(resolve_zone("inspect", angles));
    float expected_inspect[] = ZONE_INSPECT_ANGLES;
    for (int i = 0; i < 6; i++) {
        TEST_ASSERT_EQUAL_FLOAT(expected_inspect[i], angles[i]);
    }

    // harvest
    TEST_ASSERT_TRUE(resolve_zone("harvest", angles));
    float expected_harvest[] = ZONE_HARVEST_ANGLES;
    for (int i = 0; i < 6; i++) {
        TEST_ASSERT_EQUAL_FLOAT(expected_harvest[i], angles[i]);
    }

    // deposit
    TEST_ASSERT_TRUE(resolve_zone("deposit", angles));
    float expected_deposit[] = ZONE_DEPOSIT_ANGLES;
    for (int i = 0; i < 6; i++) {
        TEST_ASSERT_EQUAL_FLOAT(expected_deposit[i], angles[i]);
    }
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_lookup_hit);
    RUN_TEST(test_lookup_miss);
    RUN_TEST(test_all_default_zones);
    return UNITY_END();
}
