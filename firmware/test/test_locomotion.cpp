/**
 * @file test_locomotion.cpp
 * @brief Unit tests for Locomotion subsystem
 *
 * EDUCATIONAL NOTE: These tests verify motor control logic without requiring
 * actual hardware. We test state management, speed constraints, and direction
 * control.
 */

#include <Arduino.h>
#include <unity.h>
#include "Locomotion.h"
#include "config.h"

// Test fixtures
Locomotion* testLocomotion = nullptr;

void setUp(void) {
    testLocomotion = new Locomotion(MOTOR_A_PWM_PIN, MOTOR_A_DIR_PIN,
                                    MOTOR_B_PWM_PIN, MOTOR_B_DIR_PIN);
}

void tearDown(void) {
    if (testLocomotion != nullptr) {
        delete testLocomotion;
        testLocomotion = nullptr;
    }
}

// ============================================================================
// TEST CASES
// ============================================================================

void test_locomotion_construction(void) {
    TEST_ASSERT_NOT_NULL(testLocomotion);
    TEST_ASSERT_FALSE(testLocomotion->isReady());
    TEST_ASSERT_EQUAL_STRING("Locomotion", testLocomotion->getName());
}

void test_locomotion_enable_disable(void) {
    // Initially enabled but motors should be stopped
    TEST_ASSERT_TRUE(testLocomotion->isEnabled());

    testLocomotion->disable();
    TEST_ASSERT_FALSE(testLocomotion->isEnabled());
    TEST_ASSERT_FALSE(testLocomotion->areMotorsEnabled());

    testLocomotion->enable();
    TEST_ASSERT_TRUE(testLocomotion->isEnabled());
}

void test_locomotion_motor_speed_limits(void) {
    // Motors aren't enabled by default, check state tracking
    testLocomotion->setMotorSpeed(Locomotion::MOTOR_A, 200, true);
    TEST_ASSERT_EQUAL_UINT8(200, testLocomotion->getMotorASpeed());

    testLocomotion->setMotorSpeed(Locomotion::MOTOR_B, 128, true);
    TEST_ASSERT_EQUAL_UINT8(128, testLocomotion->getMotorBSpeed());
}

void test_locomotion_motor_direction(void) {
    testLocomotion->setMotorSpeed(Locomotion::MOTOR_A, 100, true);
    TEST_ASSERT_TRUE(testLocomotion->getMotorADirection());

    testLocomotion->setMotorSpeed(Locomotion::MOTOR_A, 100, false);
    TEST_ASSERT_FALSE(testLocomotion->getMotorADirection());
}

void test_locomotion_both_motors(void) {
    testLocomotion->setMotorSpeed(Locomotion::BOTH_MOTORS, 150, true);

    TEST_ASSERT_EQUAL_UINT8(150, testLocomotion->getMotorASpeed());
    TEST_ASSERT_EQUAL_UINT8(150, testLocomotion->getMotorBSpeed());
    TEST_ASSERT_TRUE(testLocomotion->getMotorADirection());
    TEST_ASSERT_TRUE(testLocomotion->getMotorBDirection());
}

void test_locomotion_stop_motors(void) {
    testLocomotion->setMotorSpeed(Locomotion::BOTH_MOTORS, 200, true);
    testLocomotion->stopMotors();

    TEST_ASSERT_EQUAL_UINT8(0, testLocomotion->getMotorASpeed());
    TEST_ASSERT_EQUAL_UINT8(0, testLocomotion->getMotorBSpeed());
}

void test_locomotion_move_forward(void) {
    testLocomotion->moveForward(180);

    TEST_ASSERT_EQUAL_UINT8(180, testLocomotion->getMotorASpeed());
    TEST_ASSERT_EQUAL_UINT8(180, testLocomotion->getMotorBSpeed());
    TEST_ASSERT_TRUE(testLocomotion->getMotorADirection());
    TEST_ASSERT_TRUE(testLocomotion->getMotorBDirection());
}

void test_locomotion_move_backward(void) {
    testLocomotion->moveBackward(150);

    TEST_ASSERT_EQUAL_UINT8(150, testLocomotion->getMotorASpeed());
    TEST_ASSERT_EQUAL_UINT8(150, testLocomotion->getMotorBSpeed());
    TEST_ASSERT_FALSE(testLocomotion->getMotorADirection());
    TEST_ASSERT_FALSE(testLocomotion->getMotorBDirection());
}

void test_locomotion_status_string(void) {
    String status = testLocomotion->getStatus();
    TEST_ASSERT_TRUE(status.length() > 0);
    TEST_ASSERT_TRUE(status.indexOf("Locomotion") >= 0);
}

// ============================================================================
// MAIN TEST RUNNER
// ============================================================================

void setup() {
    delay(2000);

    UNITY_BEGIN();

    RUN_TEST(test_locomotion_construction);
    RUN_TEST(test_locomotion_enable_disable);
    RUN_TEST(test_locomotion_motor_speed_limits);
    RUN_TEST(test_locomotion_motor_direction);
    RUN_TEST(test_locomotion_both_motors);
    RUN_TEST(test_locomotion_stop_motors);
    RUN_TEST(test_locomotion_move_forward);
    RUN_TEST(test_locomotion_move_backward);
    RUN_TEST(test_locomotion_status_string);

    UNITY_END();
}

void loop() {
    // Tests run once in setup()
}
