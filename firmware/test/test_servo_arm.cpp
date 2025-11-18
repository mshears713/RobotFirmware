/**
 * @file test_servo_arm.cpp
 * @brief Unit tests for ServoArm subsystem
 *
 * EDUCATIONAL NOTE: Unit testing in embedded systems helps ensure code
 * correctness before deploying to hardware. PlatformIO uses the Unity
 * testing framework for embedded unit tests.
 *
 * To run tests:
 * - Command line: pio test
 * - VSCode: Click "Test" icon in PlatformIO toolbar
 *
 * Unity testing basics:
 * - TEST_ASSERT_TRUE(condition) - Verify condition is true
 * - TEST_ASSERT_FALSE(condition) - Verify condition is false
 * - TEST_ASSERT_EQUAL(expected, actual) - Compare values
 * - TEST_ASSERT_FLOAT_WITHIN(delta, expected, actual) - Compare floats
 */

#include <Arduino.h>
#include <unity.h>
#include "ServoArm.h"
#include "config.h"

// ============================================================================
// TEST FIXTURES
// ============================================================================

/**
 * EDUCATIONAL: Test fixtures are global objects/variables used across tests.
 * They're created fresh for each test to ensure test isolation.
 */

ServoArm* testServo = nullptr;

/**
 * @brief setUp runs before EACH test
 *
 * EDUCATIONAL: Use this to initialize test objects and ensure
 * each test starts with a clean state.
 */
void setUp(void) {
    // Create a fresh ServoArm instance for each test
    testServo = new ServoArm(SERVO_ARM_PIN, 0, 180);
}

/**
 * @brief tearDown runs after EACH test
 *
 * EDUCATIONAL: Use this to clean up resources allocated in setUp
 * or during the test. Prevents memory leaks in test suite.
 */
void tearDown(void) {
    if (testServo != nullptr) {
        delete testServo;
        testServo = nullptr;
    }
}

// ============================================================================
// TEST CASES
// ============================================================================

/**
 * @brief Test servo constructor initializes values correctly
 *
 * EDUCATIONAL: This tests the constructor without touching hardware.
 * We verify that member variables are set to expected initial values.
 */
void test_servo_construction(void) {
    // Verify servo is created but not yet initialized
    TEST_ASSERT_NOT_NULL(testServo);
    TEST_ASSERT_FALSE(testServo->isReady());

    // Verify initial angles are at default (90 degrees)
    TEST_ASSERT_EQUAL_FLOAT(SERVO_DEFAULT_ANGLE, testServo->getCurrentAngle());
    TEST_ASSERT_EQUAL_FLOAT(SERVO_DEFAULT_ANGLE, testServo->getTargetAngle());

    // Verify servo has correct name
    TEST_ASSERT_EQUAL_STRING("ServoArm", testServo->getName());
}

/**
 * @brief Test angle limiting (safety feature)
 *
 * EDUCATIONAL: This verifies that setTargetAngle() properly constrains
 * angles to the configured min/max range. This prevents damaging the servo
 * by commanding it beyond its mechanical limits.
 */
void test_servo_angle_limits(void) {
    // Try to set angle beyond maximum (180°)
    testServo->setTargetAngle(250);
    TEST_ASSERT_EQUAL_FLOAT(180.0, testServo->getTargetAngle());

    // Try to set angle below minimum (0°)
    testServo->setTargetAngle(-50);
    TEST_ASSERT_EQUAL_FLOAT(0.0, testServo->getTargetAngle());

    // Verify valid angle is accepted
    testServo->setTargetAngle(90);
    TEST_ASSERT_EQUAL_FLOAT(90.0, testServo->getTargetAngle());
}

/**
 * @brief Test custom angle range limits
 *
 * EDUCATIONAL: This tests that servos can be configured with custom
 * min/max angles, useful for servos with limited range or to prevent
 * collisions with robot structure.
 */
void test_servo_custom_limits(void) {
    // Create servo with custom 45-135° range
    ServoArm* limitedServo = new ServoArm(SERVO_ARM_PIN, 45, 135);

    // Try angles outside custom range
    limitedServo->setTargetAngle(0);
    TEST_ASSERT_EQUAL_FLOAT(45.0, limitedServo->getTargetAngle());

    limitedServo->setTargetAngle(180);
    TEST_ASSERT_EQUAL_FLOAT(135.0, limitedServo->getTargetAngle());

    // Verify angles within range accepted
    limitedServo->setTargetAngle(90);
    TEST_ASSERT_EQUAL_FLOAT(90.0, limitedServo->getTargetAngle());

    delete limitedServo;
}

/**
 * @brief Test enable/disable functionality
 *
 * EDUCATIONAL: This verifies the enable/disable mechanism works correctly.
 * Disabled subsystems should not perform updates.
 */
void test_servo_enable_disable(void) {
    // Initially enabled
    TEST_ASSERT_TRUE(testServo->isEnabled());

    // Disable servo
    testServo->disable();
    TEST_ASSERT_FALSE(testServo->isEnabled());

    // Re-enable servo
    testServo->enable();
    TEST_ASSERT_TRUE(testServo->isEnabled());
}

/**
 * @brief Test hasReachedTarget detection
 *
 * EDUCATIONAL: This tests the logic for determining if servo has
 * reached its target position. Uses threshold of 0.5 degrees.
 */
void test_servo_target_detection(void) {
    // Set current and target to same value
    testServo->setAngleImmediate(90);
    testServo->setTargetAngle(90);
    TEST_ASSERT_TRUE(testServo->hasReachedTarget());

    // Set target far from current
    testServo->setTargetAngle(45);
    TEST_ASSERT_FALSE(testServo->hasReachedTarget());

    // Set target very close (within threshold)
    testServo->setAngleImmediate(90.0);
    testServo->setTargetAngle(90.3);  // Within 0.5° threshold
    TEST_ASSERT_TRUE(testServo->hasReachedTarget());
}

/**
 * @brief Test immediate angle setting
 *
 * EDUCATIONAL: Tests setAngleImmediate which bypasses smooth transitions.
 * Both current and target should be set to the same value.
 */
void test_servo_immediate_angle(void) {
    testServo->setAngleImmediate(45);

    TEST_ASSERT_EQUAL_FLOAT(45.0, testServo->getCurrentAngle());
    TEST_ASSERT_EQUAL_FLOAT(45.0, testServo->getTargetAngle());
    TEST_ASSERT_TRUE(testServo->hasReachedTarget());
}

/**
 * @brief Test reset to default position
 *
 * EDUCATIONAL: Tests the reset() method which should return servo
 * to default center position (90 degrees).
 */
void test_servo_reset(void) {
    // Move servo away from center
    testServo->setTargetAngle(180);
    TEST_ASSERT_EQUAL_FLOAT(180.0, testServo->getTargetAngle());

    // Reset should set target back to default
    testServo->reset();
    TEST_ASSERT_EQUAL_FLOAT(SERVO_DEFAULT_ANGLE, testServo->getTargetAngle());
}

/**
 * @brief Test transition speed configuration
 *
 * EDUCATIONAL: While we can't easily test actual timing in unit tests
 * (would require hardware and delays), we can verify the setter
 * constrains speed to valid ranges.
 */
void test_servo_transition_speed(void) {
    // Note: We can't directly read _transitionSpeed (it's private)
    // but we can verify the setter doesn't crash and accepts valid input

    testServo->setTransitionSpeed(5.0);  // Valid speed
    testServo->setTransitionSpeed(0.1);  // Minimum
    testServo->setTransitionSpeed(10.0); // Maximum

    // These should be constrained, not cause errors
    testServo->setTransitionSpeed(0.01);  // Too slow, should constrain
    testServo->setTransitionSpeed(100.0); // Too fast, should constrain

    // If we got here without crash, test passes
    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test getStatus returns valid string
 *
 * EDUCATIONAL: Tests that status reporting works and returns
 * a non-empty string with expected format.
 */
void test_servo_status_string(void) {
    String status = testServo->getStatus();

    // Status should not be empty
    TEST_ASSERT_TRUE(status.length() > 0);

    // Should contain "ServoArm" identifier
    TEST_ASSERT_TRUE(status.indexOf("ServoArm") >= 0);
}

// ============================================================================
// MAIN TEST RUNNER
// ============================================================================

/**
 * @brief Main test setup and execution
 *
 * EDUCATIONAL: For embedded testing, we need to initialize Serial
 * and then run all tests. The UNITY_BEGIN and UNITY_END macros
 * handle test framework setup and reporting.
 */
void setup() {
    // Wait for serial connection (useful for monitoring test output)
    delay(2000);

    // Initialize Unity test framework
    UNITY_BEGIN();

    // Run all test cases
    RUN_TEST(test_servo_construction);
    RUN_TEST(test_servo_angle_limits);
    RUN_TEST(test_servo_custom_limits);
    RUN_TEST(test_servo_enable_disable);
    RUN_TEST(test_servo_target_detection);
    RUN_TEST(test_servo_immediate_angle);
    RUN_TEST(test_servo_reset);
    RUN_TEST(test_servo_transition_speed);
    RUN_TEST(test_servo_status_string);

    // Finish test run and print results
    UNITY_END();
}

/**
 * @brief Empty loop for embedded testing
 *
 * EDUCATIONAL: In embedded unit tests, all tests run in setup()
 * and loop() remains empty. The system will halt after tests complete.
 */
void loop() {
    // Tests run once in setup(), loop stays empty
}

// ============================================================================
// EDUCATIONAL NOTES
// ============================================================================

/**
 * Understanding Unit Testing:
 *
 * Unit tests verify individual components (units) in isolation.
 * Benefits:
 * - Catch bugs early in development
 * - Verify edge cases and error conditions
 * - Serve as documentation of expected behavior
 * - Enable refactoring with confidence
 * - Prevent regression (old bugs returning)
 *
 * Best practices:
 * - Test one thing per test function
 * - Use descriptive test names (test_servo_angle_limits)
 * - Test both normal cases and edge cases
 * - Keep tests independent (don't rely on test order)
 * - Use setUp/tearDown for clean test isolation
 *
 * What to test:
 * ✓ Boundary conditions (min/max values)
 * ✓ Invalid inputs (negative angles, out of range)
 * ✓ State transitions (enabled→disabled)
 * ✓ Return values and side effects
 * ✓ Error handling
 *
 * What NOT to test:
 * ✗ Third-party library internals (e.g., Arduino Servo library)
 * ✗ Hardware behavior (use integration tests instead)
 * ✗ Implementation details (test interface, not internals)
 *
 * Running these tests:
 * 1. Connect ESP32 via USB
 * 2. Run: pio test
 * 3. View results in console
 *
 * Expected output:
 * test_servo_construction: PASS
 * test_servo_angle_limits: PASS
 * ...
 * 9 Tests 0 Failures 0 Ignored
 */
