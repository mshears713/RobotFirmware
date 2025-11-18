/**
 * @file Locomotion.h
 * @brief Motor control subsystem for robot movement
 *
 * EDUCATIONAL NOTE: This class demonstrates PWM motor control for robotics.
 * DC motors are the most common actuators for robot locomotion because:
 * - High power-to-weight ratio
 * - Continuous rotation
 * - Variable speed control via PWM
 * - Relatively inexpensive
 *
 * PWM Motor Control Fundamentals:
 * - PWM (Pulse Width Modulation) controls average voltage to motor
 * - Duty cycle (0-100%) determines speed
 *   * 0% = Motor off (0V average)
 *   * 50% = Half speed (1.65V average on 3.3V logic)
 *   * 100% = Full speed (3.3V average)
 * - Direction controlled by H-Bridge circuit:
 *   * DIR pin HIGH = Forward
 *   * DIR pin LOW = Reverse
 *
 * Motor Driver Hardware:
 * - ESP32 GPIO cannot drive motors directly (insufficient current)
 * - Use motor driver IC (L298N, DRV8833, TB6612FNG, etc.)
 * - Motor driver provides:
 *   * High current capability (1-2A per motor)
 *   * H-bridge for direction control
 *   * Protection (thermal shutdown, overcurrent)
 *
 * Typical Wiring:
 * ESP32 PWM Pin → Motor Driver ENA (speed control)
 * ESP32 DIR Pin → Motor Driver IN1/IN2 (direction control)
 * Motor Driver OUT1/OUT2 → DC Motor
 * Motor Driver VCC → Battery/Power supply (6-12V)
 * Motor Driver GND → Common ground with ESP32
 */

#ifndef LOCOMOTION_H
#define LOCOMOTION_H

#include <Arduino.h>
#include "RobotSubsystem.h"
#include "config.h"

/**
 * @class Locomotion
 * @brief Controls DC motors for robot movement using PWM
 *
 * EDUCATIONAL: This class implements differential drive control, where
 * two motors (left and right) are controlled independently. By varying
 * the speed and direction of each motor, the robot can:
 * - Move forward (both motors forward at same speed)
 * - Move backward (both motors reverse at same speed)
 * - Turn left (right motor faster than left)
 * - Turn right (left motor faster than right)
 * - Rotate in place (motors opposite directions)
 */
class Locomotion : public RobotSubsystem {
public:
    /**
     * @brief Motor selection enum
     *
     * EDUCATIONAL: Using enums instead of magic numbers makes code readable.
     */
    enum Motor {
        MOTOR_A = 0,
        MOTOR_B = 1,
        BOTH_MOTORS = 2
    };

    /**
     * @brief Construct a new Locomotion object
     *
     * EDUCATIONAL: Constructor sets pin configuration. Motor drivers need:
     * - PWM pin for speed (uses ESP32's LEDC PWM peripheral)
     * - Direction pin for forward/reverse
     *
     * @param motorA_pwmPin PWM pin for Motor A
     * @param motorA_dirPin Direction pin for Motor A
     * @param motorB_pwmPin PWM pin for Motor B
     * @param motorB_dirPin Direction pin for Motor B
     */
    Locomotion(int motorA_pwmPin = MOTOR_A_PWM_PIN,
               int motorA_dirPin = MOTOR_A_DIR_PIN,
               int motorB_pwmPin = MOTOR_B_PWM_PIN,
               int motorB_dirPin = MOTOR_B_DIR_PIN)
        : _motorA_pwmPin(motorA_pwmPin),
          _motorA_dirPin(motorA_dirPin),
          _motorB_pwmPin(motorB_pwmPin),
          _motorB_dirPin(motorB_dirPin),
          _motorA_speed(0),
          _motorB_speed(0),
          _motorA_direction(true),
          _motorB_direction(true),
          _motorsEnabled(false)
    {
        // Constructor initializes configuration only
    }

    /**
     * @brief Initialize PWM and direction pins
     *
     * EDUCATIONAL: ESP32 has 16 independent PWM channels (LEDC peripheral).
     * We configure 2 channels, one for each motor. PWM parameters:
     * - Frequency: 5kHz (good for motors, inaudible)
     * - Resolution: 8-bit (0-255 speed values)
     *
     * Higher PWM frequency = smoother motor operation but more switching losses
     * Lower frequency = more efficient but may cause audible whine
     *
     * @return true if initialization successful
     */
    bool initialize() override {
        Serial.println("[Locomotion] Initializing motor control...");

        // Configure direction pins as outputs
        pinMode(_motorA_dirPin, OUTPUT);
        pinMode(_motorB_dirPin, OUTPUT);

        // Set initial direction (forward)
        digitalWrite(_motorA_dirPin, HIGH);
        digitalWrite(_motorB_dirPin, HIGH);

        Serial.print("[Locomotion] Direction pins configured: Motor A=");
        Serial.print(_motorA_dirPin);
        Serial.print(", Motor B=");
        Serial.println(_motorB_dirPin);

        // Configure PWM for Motor A
        // EDUCATIONAL: ledcSetup() configures a PWM channel
        // Parameters: channel, frequency, resolution
        const int PWM_CHANNEL_A = 0;
        const int PWM_CHANNEL_B = 1;
        const int PWM_FREQUENCY = 5000;    // 5kHz
        const int PWM_RESOLUTION = 8;      // 8-bit (0-255)

        ledcSetup(PWM_CHANNEL_A, PWM_FREQUENCY, PWM_RESOLUTION);
        ledcAttachPin(_motorA_pwmPin, PWM_CHANNEL_A);
        ledcWrite(PWM_CHANNEL_A, 0);  // Start at 0 speed

        Serial.print("[Locomotion] Motor A PWM: pin=");
        Serial.print(_motorA_pwmPin);
        Serial.print(", channel=");
        Serial.print(PWM_CHANNEL_A);
        Serial.print(", freq=");
        Serial.print(PWM_FREQUENCY);
        Serial.println("Hz");

        // Configure PWM for Motor B
        ledcSetup(PWM_CHANNEL_B, PWM_FREQUENCY, PWM_RESOLUTION);
        ledcAttachPin(_motorB_pwmPin, PWM_CHANNEL_B);
        ledcWrite(PWM_CHANNEL_B, 0);  // Start at 0 speed

        Serial.print("[Locomotion] Motor B PWM: pin=");
        Serial.print(_motorB_pwmPin);
        Serial.print(", channel=");
        Serial.print(PWM_CHANNEL_B);
        Serial.print(", freq=");
        Serial.print(PWM_FREQUENCY);
        Serial.println("Hz");

        // Store PWM channels for later use
        _pwmChannelA = PWM_CHANNEL_A;
        _pwmChannelB = PWM_CHANNEL_B;

        // Motors start disabled for safety
        _motorsEnabled = false;

        _isInitialized = true;
        Serial.println("[Locomotion] ✓ Motor control initialized");
        Serial.println("[Locomotion] Motors disabled (call enable() to activate)");

        return true;
    }

    /**
     * @brief Update motor control (called every loop iteration)
     *
     * EDUCATIONAL: This could implement:
     * - Acceleration/deceleration ramping (smoother starts/stops)
     * - Current monitoring
     * - Encoder feedback (closed-loop speed control)
     *
     * For now, it's a placeholder for future enhancements.
     */
    void update() override {
        if (!_isInitialized) {
            return;
        }

        // Future: Implement acceleration ramping, safety checks, etc.
    }

    /**
     * @brief Check if subsystem is ready
     *
     * @return true if initialized
     */
    bool isReady() override {
        return _isInitialized;
    }

    /**
     * @brief Get current status string
     *
     * @return Status with motor speeds and directions
     */
    String getStatus() override {
        if (!_isInitialized) {
            return "Locomotion: Not initialized";
        }

        char buffer[128];
        snprintf(buffer, sizeof(buffer),
                 "Locomotion: A=%d%s, B=%d%s [%s]",
                 _motorA_speed,
                 _motorA_direction ? "F" : "R",
                 _motorB_speed,
                 _motorB_direction ? "F" : "R",
                 _motorsEnabled ? "ENABLED" : "disabled");
        return String(buffer);
    }

    /**
     * @brief Get subsystem name
     *
     * @return Subsystem identifier
     */
    const char* getName() const override {
        return "Locomotion";
    }

    /**
     * @brief Enable motor outputs
     *
     * EDUCATIONAL: Safety feature - motors start disabled and must be
     * explicitly enabled. This prevents accidental movement during testing.
     */
    void enable() override {
        RobotSubsystem::enable();
        _motorsEnabled = true;
        Serial.println("[Locomotion] Motors ENABLED");
    }

    /**
     * @brief Disable motor outputs (stop motors)
     *
     * EDUCATIONAL: Emergency stop function. Sets all motors to zero speed.
     */
    void disable() override {
        RobotSubsystem::disable();
        _motorsEnabled = false;
        stopMotors();
        Serial.println("[Locomotion] Motors DISABLED");
    }

    /**
     * @brief Set motor speed and direction
     *
     * EDUCATIONAL: This is the core motor control function.
     * Speed is 0-255, where:
     * - 0 = Stopped
     * - 128 = Half speed
     * - 255 = Full speed
     *
     * @param motor Which motor to control (MOTOR_A, MOTOR_B, BOTH_MOTORS)
     * @param speed Speed value 0-255
     * @param forward true for forward, false for reverse
     */
    void setMotorSpeed(Motor motor, uint8_t speed, bool forward = true) {
        if (!_isInitialized) {
            Serial.println("[Locomotion] ✗ Cannot set speed - not initialized");
            return;
        }

        // Constrain speed to valid range
        speed = constrain(speed, 0, 255);

        // Update Motor A
        if (motor == MOTOR_A || motor == BOTH_MOTORS) {
            _motorA_speed = speed;
            _motorA_direction = forward;

            digitalWrite(_motorA_dirPin, forward ? HIGH : LOW);

            if (_motorsEnabled) {
                ledcWrite(_pwmChannelA, speed);
            } else {
                ledcWrite(_pwmChannelA, 0);  // Keep stopped if disabled
            }

            if (DEBUG_VERBOSE) {
                Serial.print("[Locomotion] Motor A: speed=");
                Serial.print(speed);
                Serial.print(", direction=");
                Serial.println(forward ? "FORWARD" : "REVERSE");
            }
        }

        // Update Motor B
        if (motor == MOTOR_B || motor == BOTH_MOTORS) {
            _motorB_speed = speed;
            _motorB_direction = forward;

            digitalWrite(_motorB_dirPin, forward ? HIGH : LOW);

            if (_motorsEnabled) {
                ledcWrite(_pwmChannelB, speed);
            } else {
                ledcWrite(_pwmChannelB, 0);  // Keep stopped if disabled
            }

            if (DEBUG_VERBOSE) {
                Serial.print("[Locomotion] Motor B: speed=");
                Serial.print(speed);
                Serial.print(", direction=");
                Serial.println(forward ? "FORWARD" : "REVERSE");
            }
        }
    }

    /**
     * @brief Stop all motors immediately
     *
     * EDUCATIONAL: Emergency stop function. Sets PWM duty cycle to 0.
     */
    void stopMotors() {
        _motorA_speed = 0;
        _motorB_speed = 0;

        ledcWrite(_pwmChannelA, 0);
        ledcWrite(_pwmChannelB, 0);

        if (DEBUG_VERBOSE) {
            Serial.println("[Locomotion] All motors STOPPED");
        }
    }

    /**
     * @brief Move robot forward
     *
     * EDUCATIONAL: High-level movement command. Both motors forward at same speed.
     *
     * @param speed Speed value 0-255
     */
    void moveForward(uint8_t speed) {
        setMotorSpeed(BOTH_MOTORS, speed, true);
        if (DEBUG_VERBOSE) {
            Serial.print("[Locomotion] Moving FORWARD at speed ");
            Serial.println(speed);
        }
    }

    /**
     * @brief Move robot backward
     *
     * @param speed Speed value 0-255
     */
    void moveBackward(uint8_t speed) {
        setMotorSpeed(BOTH_MOTORS, speed, false);
        if (DEBUG_VERBOSE) {
            Serial.print("[Locomotion] Moving BACKWARD at speed ");
            Serial.println(speed);
        }
    }

    /**
     * @brief Turn robot left
     *
     * EDUCATIONAL: For differential drive, left turn is achieved by:
     * - Slowing or stopping left motor
     * - Maintaining or increasing right motor speed
     *
     * For sharp turns, left motor can go reverse.
     *
     * @param speed Base speed for turn (right motor)
     * @param turnRate How sharp to turn (0=gentle, 255=spin in place)
     */
    void turnLeft(uint8_t speed, uint8_t turnRate = 128) {
        uint8_t rightSpeed = speed;
        uint8_t leftSpeed = speed - turnRate;

        if (leftSpeed > 128) {  // Underflow (went negative)
            // Very sharp turn - left motor reverses
            setMotorSpeed(MOTOR_A, abs((int8_t)leftSpeed), false);  // Reverse
        } else {
            setMotorSpeed(MOTOR_A, leftSpeed, true);  // Forward but slower
        }

        setMotorSpeed(MOTOR_B, rightSpeed, true);

        if (DEBUG_VERBOSE) {
            Serial.print("[Locomotion] Turning LEFT: L=");
            Serial.print(leftSpeed);
            Serial.print(", R=");
            Serial.println(rightSpeed);
        }
    }

    /**
     * @brief Turn robot right
     *
     * @param speed Base speed for turn (left motor)
     * @param turnRate How sharp to turn (0=gentle, 255=spin in place)
     */
    void turnRight(uint8_t speed, uint8_t turnRate = 128) {
        uint8_t leftSpeed = speed;
        uint8_t rightSpeed = speed - turnRate;

        setMotorSpeed(MOTOR_A, leftSpeed, true);

        if (rightSpeed > 128) {  // Underflow
            setMotorSpeed(MOTOR_B, abs((int8_t)rightSpeed), false);  // Reverse
        } else {
            setMotorSpeed(MOTOR_B, rightSpeed, true);  // Forward but slower
        }

        if (DEBUG_VERBOSE) {
            Serial.print("[Locomotion] Turning RIGHT: L=");
            Serial.print(leftSpeed);
            Serial.print(", R=");
            Serial.println(rightSpeed);
        }
    }

    /**
     * @brief Rotate robot in place
     *
     * EDUCATIONAL: Zero-radius turn by running motors in opposite directions.
     *
     * @param speed Rotation speed 0-255
     * @param clockwise true for clockwise, false for counter-clockwise
     */
    void rotateInPlace(uint8_t speed, bool clockwise = true) {
        if (clockwise) {
            setMotorSpeed(MOTOR_A, speed, true);   // Left forward
            setMotorSpeed(MOTOR_B, speed, false);  // Right reverse
        } else {
            setMotorSpeed(MOTOR_A, speed, false);  // Left reverse
            setMotorSpeed(MOTOR_B, speed, true);   // Right forward
        }

        if (DEBUG_VERBOSE) {
            Serial.print("[Locomotion] Rotating ");
            Serial.print(clockwise ? "CLOCKWISE" : "COUNTER-CLOCKWISE");
            Serial.print(" at speed ");
            Serial.println(speed);
        }
    }

    /**
     * @brief Get Motor A current speed
     *
     * @return Speed value 0-255
     */
    uint8_t getMotorASpeed() const {
        return _motorA_speed;
    }

    /**
     * @brief Get Motor B current speed
     *
     * @return Speed value 0-255
     */
    uint8_t getMotorBSpeed() const {
        return _motorB_speed;
    }

    /**
     * @brief Get Motor A direction
     *
     * @return true if forward, false if reverse
     */
    bool getMotorADirection() const {
        return _motorA_direction;
    }

    /**
     * @brief Get Motor B direction
     *
     * @return true if forward, false if reverse
     */
    bool getMotorBDirection() const {
        return _motorB_direction;
    }

    /**
     * @brief Check if motors are enabled
     *
     * @return true if enabled
     */
    bool areMotorsEnabled() const {
        return _motorsEnabled;
    }

private:
    // Pin assignments
    int _motorA_pwmPin;         ///< Motor A PWM pin
    int _motorA_dirPin;         ///< Motor A direction pin
    int _motorB_pwmPin;         ///< Motor B PWM pin
    int _motorB_dirPin;         ///< Motor B direction pin

    // PWM channels
    int _pwmChannelA;           ///< LEDC channel for Motor A
    int _pwmChannelB;           ///< LEDC channel for Motor B

    // Current state
    uint8_t _motorA_speed;      ///< Motor A speed (0-255)
    uint8_t _motorB_speed;      ///< Motor B speed (0-255)
    bool _motorA_direction;     ///< Motor A direction (true=forward)
    bool _motorB_direction;     ///< Motor B direction (true=forward)
    bool _motorsEnabled;        ///< Motors enabled flag
};

#endif // LOCOMOTION_H
