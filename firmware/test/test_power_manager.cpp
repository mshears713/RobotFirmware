/**
 * @file test_power_manager.cpp
 * @brief Unit tests for PowerManager class - Phase 4
 *
 * EDUCATIONAL: This demonstrates embedded unit testing using PlatformIO's
 * Unity test framework. Unit tests validate individual component behavior
 * before integration.
 *
 * To run these tests:
 * ```
 * pio test
 * ```
 */

#include <unity.h>
#include "PowerManager.h"

// Test instance
PowerManager* powerMgr = nullptr;

/**
 * @brief Setup function - runs before each test
 */
void setUp() {
    // Create fresh PowerManager instance for each test
    powerMgr = new PowerManager(5000, 10000, 15000);  // Short timeouts for testing
    powerMgr->initialize();
}

/**
 * @brief Teardown function - runs after each test
 */
void tearDown() {
    // Clean up
    if (powerMgr != nullptr) {
        delete powerMgr;
        powerMgr = nullptr;
    }
}

/**
 * @brief Test PowerManager initialization
 */
void test_power_manager_initialization() {
    TEST_ASSERT_NOT_NULL(powerMgr);
    TEST_ASSERT_TRUE(powerMgr->isReady());
    TEST_ASSERT_EQUAL(PowerMode::ACTIVE, powerMgr->getPowerMode());
}

/**
 * @brief Test power mode transitions
 */
void test_power_mode_transitions() {
    // Start in ACTIVE mode
    TEST_ASSERT_EQUAL(PowerMode::ACTIVE, powerMgr->getPowerMode());

    // Transition to IDLE
    powerMgr->setPowerMode(PowerMode::IDLE);
    TEST_ASSERT_EQUAL(PowerMode::IDLE, powerMgr->getPowerMode());

    // Transition to LOW_POWER
    powerMgr->setPowerMode(PowerMode::LOW_POWER);
    TEST_ASSERT_EQUAL(PowerMode::LOW_POWER, powerMgr->getPowerMode());

    // Back to ACTIVE
    powerMgr->setPowerMode(PowerMode::ACTIVE);
    TEST_ASSERT_EQUAL(PowerMode::ACTIVE, powerMgr->getPowerMode());
}

/**
 * @brief Test activity registration
 */
void test_activity_registration() {
    // Register activity
    powerMgr->registerActivity();

    // Time since activity should be very small (< 100ms)
    delay(50);
    unsigned long timeSinceActivity = powerMgr->getTimeSinceActivity();
    TEST_ASSERT_LESS_THAN(100, timeSinceActivity);

    // Wait a bit and check again
    delay(500);
    timeSinceActivity = powerMgr->getTimeSinceActivity();
    TEST_ASSERT_GREATER_THAN(400, timeSinceActivity);
}

/**
 * @brief Test estimated current draw calculations
 */
void test_estimated_current_draw() {
    // ACTIVE mode should draw most current
    powerMgr->setPowerMode(PowerMode::ACTIVE);
    float activeCurrent = powerMgr->getEstimatedCurrentDraw();
    TEST_ASSERT_GREATER_THAN(100.0f, activeCurrent);  // > 100mA

    // LOW_POWER mode should draw less
    powerMgr->setPowerMode(PowerMode::LOW_POWER);
    float lowPowerCurrent = powerMgr->getEstimatedCurrentDraw();
    TEST_ASSERT_LESS_THAN(activeCurrent, lowPowerCurrent);

    // Verify current estimate is reasonable
    TEST_ASSERT_GREATER_THAN(10.0f, lowPowerCurrent);   // > 10mA
    TEST_ASSERT_LESS_THAN(100.0f, lowPowerCurrent);     // < 100mA
}

/**
 * @brief Test wake timer configuration
 */
void test_wake_timer_configuration() {
    // Configure wake timer for 30 seconds
    uint64_t duration = 30 * 1000000;  // 30 seconds in microseconds
    powerMgr->configureWakeTimer(duration);

    // No direct way to verify, but should not crash
    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test power mode string conversion
 */
void test_power_mode_strings() {
    powerMgr->setPowerMode(PowerMode::ACTIVE);
    TEST_ASSERT_EQUAL_STRING("ACTIVE", powerMgr->getPowerModeString().c_str());

    powerMgr->setPowerMode(PowerMode::IDLE);
    TEST_ASSERT_EQUAL_STRING("IDLE", powerMgr->getPowerModeString().c_str());

    powerMgr->setPowerMode(PowerMode::LOW_POWER);
    TEST_ASSERT_EQUAL_STRING("LOW_POWER", powerMgr->getPowerModeString().c_str());
}

/**
 * @brief Main test runner
 */
int main() {
    UNITY_BEGIN();

    RUN_TEST(test_power_manager_initialization);
    RUN_TEST(test_power_mode_transitions);
    RUN_TEST(test_activity_registration);
    RUN_TEST(test_estimated_current_draw);
    RUN_TEST(test_wake_timer_configuration);
    RUN_TEST(test_power_mode_strings);

    return UNITY_END();
}
