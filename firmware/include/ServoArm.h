/**
 * @file ServoArm.h
 * @brief Servo motor control subsystem for robotic arm movements
 *
 * EDUCATIONAL NOTE: This class demonstrates PWM-based servo control.
 * Servos are position-controlled actuators commonly used in robotics.
 *
 * How servos work:
 * - They expect a 50Hz PWM signal (20ms period)
 * - Pulse width determines position:
 *   * 1.0ms pulse = 0 degrees (full left)
 *   * 1.5ms pulse = 90 degrees (center)
 *   * 2.0ms pulse = 180 degrees (full right)
 * - The servo's internal circuitry maintains the position
 *
 * The Arduino Servo library handles the PWM timing details for us.
 */

#ifndef SERVO_ARM_H
#define SERVO_ARM_H

#include <Arduino.h>
#include <Servo.h>
#include "RobotSubsystem.h"
#include "config.h"

/**
 * @class ServoArm
 * @brief Controls a servo motor with smooth transitions and safety limits
 *
 * EDUCATIONAL: This class inherits from RobotSubsystem and implements all
 * required methods. It adds servo-specific functionality like angle control,
 * smooth transitions, and position limits.
 */
class ServoArm : public RobotSubsystem {
public:
    /**
     * @brief Construct a new ServoArm object
     *
     * EDUCATIONAL: The constructor sets up configuration but doesn't touch
     * hardware yet. Actual hardware initialization happens in initialize().
     * This separation is important in embedded systems to control when
     * hardware resources are allocated.
     *
     * @param pin GPIO pin number for servo PWM signal
     * @param minAngle Minimum allowed angle in degrees (default 0)
     * @param maxAngle Maximum allowed angle in degrees (default 180)
     */
    ServoArm(int pin,
             float minAngle = SERVO_MIN_ANGLE,
             float maxAngle = SERVO_MAX_ANGLE)
        : _pin(pin),
          _minAngle(minAngle),
          _maxAngle(maxAngle),
          _currentAngle(SERVO_DEFAULT_ANGLE),
          _targetAngle(SERVO_DEFAULT_ANGLE),
          _transitionSpeed(SERVO_TRANSITION_SPEED)
    {
        // Constructor initializes member variables only
        // No hardware interaction yet!
    }

    /**
     * @brief Initialize the servo hardware
     *
     * EDUCATIONAL: This method attaches the servo to its GPIO pin and moves
     * it to the default position. The Servo library configures the pin for
     * PWM output and starts generating the 50Hz signal.
     *
     * @return true if servo attached successfully, false otherwise
     */
    bool initialize() override {
        Serial.print("[ServoArm] Initializing on pin ");
        Serial.println(_pin);

        // Attach servo to pin
        // EDUCATIONAL: attach() configures the pin for PWM and returns true on success
        if (_servo.attach(_pin)) {
            // Move to default position
            _servo.write(_currentAngle);

            _isInitialized = true;
            Serial.println("[ServoArm] ✓ Initialization successful");
            return true;
        } else {
            Serial.println("[ServoArm] ✗ Initialization FAILED - could not attach servo");
            return false;
        }
    }

    /**
     * @brief Update servo position (called every loop iteration)
     *
     * EDUCATIONAL: This implements smooth servo movement by gradually
     * transitioning from current position to target position. This prevents
     * jerky motion and reduces mechanical stress on the servo.
     *
     * The update rate is controlled by MAIN_LOOP_INTERVAL in config.h.
     * At 20ms interval with 2°/step speed, full 180° sweep takes 1.8 seconds.
     */
    void update() override {
        if (!_isInitialized || !_isEnabled) {
            return;  // Skip update if not ready
        }

        // Calculate difference between current and target angles
        float delta = _targetAngle - _currentAngle;

        // EDUCATIONAL: Only update if we're not already at the target
        // Using a small threshold (0.5°) prevents oscillation
        if (abs(delta) > 0.5f) {
            // Calculate step size (limited by transition speed)
            // EDUCATIONAL: constrain() limits the value to a range.
            // This ensures we don't overshoot and don't move too fast.
            float step = constrain(delta, -_transitionSpeed, _transitionSpeed);

            // Update current angle
            _currentAngle += step;

            // Write new position to servo
            // EDUCATIONAL: write() accepts integer degrees (0-180)
            _servo.write((int)_currentAngle);
        }
    }

    /**
     * @brief Check if servo is ready to operate
     *
     * @return true if initialized and servo is attached
     */
    bool isReady() override {
        return _isInitialized && _servo.attached();
    }

    /**
     * @brief Get current status string
     *
     * @return Status string with current and target angles
     */
    String getStatus() override {
        if (!_isInitialized) {
            return "ServoArm: Not initialized";
        }

        char buffer[64];
        snprintf(buffer, sizeof(buffer),
                 "ServoArm: %.1f° → %.1f° (pin %d)",
                 _currentAngle, _targetAngle, _pin);
        return String(buffer);
    }

    /**
     * @brief Get subsystem name
     *
     * @return Subsystem identifier
     */
    const char* getName() const override {
        return "ServoArm";
    }

    /**
     * @brief Set target angle for servo to move to
     *
     * EDUCATIONAL: This method sets the target angle but doesn't move the
     * servo immediately. The update() method will gradually transition to
     * this angle over multiple loop iterations.
     *
     * @param angle Target angle in degrees
     */
    void setTargetAngle(float angle) {
        // EDUCATIONAL: Constrain angle to safe operating range
        // This prevents trying to move servo beyond its mechanical limits
        _targetAngle = constrain(angle, _minAngle, _maxAngle);

        if (DEBUG_VERBOSE) {
            Serial.print("[ServoArm] Target angle set to ");
            Serial.print(_targetAngle);
            Serial.println("°");
        }
    }

    /**
     * @brief Get current servo angle
     *
     * @return Current angle in degrees
     */
    float getCurrentAngle() const {
        return _currentAngle;
    }

    /**
     * @brief Get target servo angle
     *
     * @return Target angle in degrees
     */
    float getTargetAngle() const {
        return _targetAngle;
    }

    /**
     * @brief Check if servo has reached target position
     *
     * EDUCATIONAL: Useful for coordination - wait until servo reaches
     * position before starting next movement.
     *
     * @return true if current angle is within 0.5° of target
     */
    bool hasReachedTarget() const {
        return abs(_targetAngle - _currentAngle) <= 0.5f;
    }

    /**
     * @brief Set transition speed
     *
     * EDUCATIONAL: Higher speed = faster movement but more jerky.
     * Lower speed = smoother but slower. Typical range: 1-5 degrees/update.
     *
     * @param speed Degrees per update cycle
     */
    void setTransitionSpeed(float speed) {
        // EDUCATIONAL: Limit speed to reasonable range
        _transitionSpeed = constrain(speed, 0.1f, 10.0f);
    }

    /**
     * @brief Immediately move servo to angle (no smooth transition)
     *
     * EDUCATIONAL: Use this sparingly. Instant movements can cause:
     * - Mechanical stress on servo gears
     * - Current spikes
     * - Jerky robot motion
     * Prefer setTargetAngle() for normal operation.
     *
     * @param angle Angle to move to immediately
     */
    void setAngleImmediate(float angle) {
        _targetAngle = constrain(angle, _minAngle, _maxAngle);
        _currentAngle = _targetAngle;
        _servo.write((int)_currentAngle);

        if (DEBUG_VERBOSE) {
            Serial.print("[ServoArm] Immediate move to ");
            Serial.print(_currentAngle);
            Serial.println("°");
        }
    }

    /**
     * @brief Reset servo to default center position
     */
    void reset() override {
        setTargetAngle(SERVO_DEFAULT_ANGLE);
    }

    /**
     * @brief Disable servo (stops PWM signal)
     *
     * EDUCATIONAL: When disabled, the servo stops receiving PWM and loses
     * holding torque. It can be manually moved. Useful for:
     * - Power saving
     * - Manual positioning
     * - Safety (allows passive movement)
     */
    void disable() override {
        RobotSubsystem::disable();
        if (_isInitialized) {
            _servo.detach();
        }
    }

    /**
     * @brief Re-enable servo after disable()
     */
    void enable() override {
        RobotSubsystem::enable();
        if (_isInitialized) {
            _servo.attach(_pin);
            _servo.write((int)_currentAngle);
        }
    }

private:
    /**
     * EDUCATIONAL: Private members are internal implementation details.
     * They cannot be accessed from outside the class, which prevents
     * accidental misuse and allows us to change implementation later.
     */

    Servo _servo;              ///< Arduino Servo library object
    int _pin;                  ///< GPIO pin number
    float _currentAngle;       ///< Current servo position in degrees
    float _targetAngle;        ///< Target servo position in degrees
    float _minAngle;           ///< Minimum allowed angle (safety limit)
    float _maxAngle;           ///< Maximum allowed angle (safety limit)
    float _transitionSpeed;    ///< Degrees to move per update cycle
};

#endif // SERVO_ARM_H
