/**
 * @file CommandProcessor.h
 * @brief Command parsing and execution for robot control
 *
 * EDUCATIONAL NOTE: This class acts as the "command interpreter" for the robot.
 * It receives JSON commands from the web API and translates them into
 * actions on the robot's subsystems.
 *
 * Command Pattern:
 * - Encapsulates requests as objects
 * - Decouples sender (UI) from receiver (robot subsystems)
 * - Enables queuing, logging, and undo operations
 *
 * Supported Commands:
 * - set_servo_angle: Move servo to specific angle
 * - set_motor_speed: Control motor speed and direction
 * - move_forward/backward: High-level locomotion
 * - turn_left/right: Turning maneuvers
 * - rotate: Rotate in place
 * - stop_motors: Emergency stop
 * - enable_motors/disable_motors: Motor safety
 *
 * Phase 4 Commands:
 * - set_power_mode: Change power management mode
 * - clear_errors: Clear error log
 * - perform_health_check: Run system health check
 * - set_operational_mode: Change operational mode (idle/active/etc)
 */

#ifndef COMMAND_PROCESSOR_H
#define COMMAND_PROCESSOR_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "ServoArm.h"
#include "Locomotion.h"
#include "PowerManager.h"
#include "ErrorManager.h"

/**
 * @class CommandProcessor
 * @brief Processes and executes JSON commands on robot subsystems
 *
 * EDUCATIONAL: This demonstrates command pattern implementation.
 * Benefits:
 * - Centralized command logic
 * - Easy to add new commands
 * - Command validation in one place
 * - Can add command history/logging
 */
class CommandProcessor {
public:
    /**
     * @brief Construct CommandProcessor
     *
     * @param servo Pointer to ServoArm subsystem
     * @param locomotion Pointer to Locomotion subsystem
     * @param powerManager Pointer to PowerManager (Phase 4)
     * @param errorManager Pointer to ErrorManager (Phase 4)
     */
    CommandProcessor(
        ServoArm* servo,
        Locomotion* locomotion,
        PowerManager* powerManager = nullptr,
        ErrorManager* errorManager = nullptr
    )
        : _servo(servo),
          _locomotion(locomotion),
          _powerManager(powerManager),
          _errorManager(errorManager),
          _commandCount(0),
          _errorCount(0),
          _currentMode("active")
    {
        // Constructor stores subsystem pointers
    }

    /**
     * @brief Process and execute JSON command
     *
     * EDUCATIONAL: This is the main entry point for command execution.
     * It parses the JSON, validates the command, and executes it.
     *
     * @param commandJson JSON document containing command
     * @return true if command executed successfully
     */
    bool processCommand(const JsonDocument& commandJson) {
        _commandCount++;

        // Extract command name
        const char* command = commandJson["command"];
        if (command == nullptr) {
            Serial.println("[CommandProcessor] ✗ Missing 'command' field");
            _errorCount++;
            return false;
        }

        Serial.print("[CommandProcessor] Processing command: ");
        Serial.println(command);

        // Route to appropriate handler
        bool success = false;

        if (strcmp(command, "set_servo_angle") == 0) {
            success = handleSetServoAngle(commandJson);
        }
        else if (strcmp(command, "set_motor_speed") == 0) {
            success = handleSetMotorSpeed(commandJson);
        }
        else if (strcmp(command, "move_forward") == 0) {
            success = handleMoveForward(commandJson);
        }
        else if (strcmp(command, "move_backward") == 0) {
            success = handleMoveBackward(commandJson);
        }
        else if (strcmp(command, "turn_left") == 0) {
            success = handleTurnLeft(commandJson);
        }
        else if (strcmp(command, "turn_right") == 0) {
            success = handleTurnRight(commandJson);
        }
        else if (strcmp(command, "rotate") == 0) {
            success = handleRotate(commandJson);
        }
        else if (strcmp(command, "stop_motors") == 0) {
            success = handleStopMotors(commandJson);
        }
        else if (strcmp(command, "enable_motors") == 0) {
            success = handleEnableMotors(commandJson);
        }
        else if (strcmp(command, "disable_motors") == 0) {
            success = handleDisableMotors(commandJson);
        }
        // Phase 4 commands
        else if (strcmp(command, "set_power_mode") == 0) {
            success = handleSetPowerMode(commandJson);
        }
        else if (strcmp(command, "set_operational_mode") == 0) {
            success = handleSetOperationalMode(commandJson);
        }
        else if (strcmp(command, "clear_errors") == 0) {
            success = handleClearErrors(commandJson);
        }
        else if (strcmp(command, "perform_health_check") == 0) {
            success = handlePerformHealthCheck(commandJson);
        }
        else {
            Serial.print("[CommandProcessor] ✗ Unknown command: ");
            Serial.println(command);
            _errorCount++;
            return false;
        }

        if (!success) {
            _errorCount++;
        }

        return success;
    }

    /**
     * @brief Get total command count
     */
    unsigned long getCommandCount() const {
        return _commandCount;
    }

    /**
     * @brief Get error count
     */
    unsigned long getErrorCount() const {
        return _errorCount;
    }

    /**
     * @brief Get current operational mode
     */
    String getOperationalMode() const {
        return _currentMode;
    }

    /**
     * @brief Set operational mode
     */
    void setOperationalMode(const String& mode) {
        _currentMode = mode;
    }

private:
    /**
     * @brief Handle set_servo_angle command
     *
     * Format: {"command": "set_servo_angle", "value": 90}
     */
    bool handleSetServoAngle(const JsonDocument& cmd) {
        if (_servo == nullptr) {
            Serial.println("[CommandProcessor] ✗ Servo not available");
            return false;
        }

        if (!cmd.containsKey("value")) {
            Serial.println("[CommandProcessor] ✗ Missing 'value' parameter");
            return false;
        }

        float angle = cmd["value"];

        _servo->setTargetAngle(angle);

        Serial.print("[CommandProcessor] ✓ Servo angle set to ");
        Serial.print(angle);
        Serial.println("°");

        return true;
    }

    /**
     * @brief Handle set_motor_speed command
     *
     * Format: {"command": "set_motor_speed", "motor": "A", "speed": 150, "forward": true}
     */
    bool handleSetMotorSpeed(const JsonDocument& cmd) {
        if (_locomotion == nullptr) {
            Serial.println("[CommandProcessor] ✗ Locomotion not available");
            return false;
        }

        if (!cmd.containsKey("motor") || !cmd.containsKey("speed")) {
            Serial.println("[CommandProcessor] ✗ Missing parameters");
            return false;
        }

        const char* motorStr = cmd["motor"];
        uint8_t speed = cmd["speed"];
        bool forward = cmd["forward"] | true;  // Default to forward

        Locomotion::Motor motor;
        if (strcmp(motorStr, "A") == 0) {
            motor = Locomotion::MOTOR_A;
        } else if (strcmp(motorStr, "B") == 0) {
            motor = Locomotion::MOTOR_B;
        } else if (strcmp(motorStr, "BOTH") == 0) {
            motor = Locomotion::BOTH_MOTORS;
        } else {
            Serial.println("[CommandProcessor] ✗ Invalid motor selection");
            return false;
        }

        _locomotion->setMotorSpeed(motor, speed, forward);

        Serial.print("[CommandProcessor] ✓ Motor ");
        Serial.print(motorStr);
        Serial.print(" set to speed ");
        Serial.print(speed);
        Serial.print(" ");
        Serial.println(forward ? "forward" : "reverse");

        return true;
    }

    /**
     * @brief Handle move_forward command
     *
     * Format: {"command": "move_forward", "speed": 150}
     */
    bool handleMoveForward(const JsonDocument& cmd) {
        if (_locomotion == nullptr) {
            Serial.println("[CommandProcessor] ✗ Locomotion not available");
            return false;
        }

        uint8_t speed = cmd["speed"] | 128;  // Default to medium speed

        _locomotion->moveForward(speed);

        Serial.print("[CommandProcessor] ✓ Moving forward at speed ");
        Serial.println(speed);

        return true;
    }

    /**
     * @brief Handle move_backward command
     *
     * Format: {"command": "move_backward", "speed": 150}
     */
    bool handleMoveBackward(const JsonDocument& cmd) {
        if (_locomotion == nullptr) {
            Serial.println("[CommandProcessor] ✗ Locomotion not available");
            return false;
        }

        uint8_t speed = cmd["speed"] | 128;

        _locomotion->moveBackward(speed);

        Serial.print("[CommandProcessor] ✓ Moving backward at speed ");
        Serial.println(speed);

        return true;
    }

    /**
     * @brief Handle turn_left command
     *
     * Format: {"command": "turn_left", "speed": 120, "turn_rate": 64}
     */
    bool handleTurnLeft(const JsonDocument& cmd) {
        if (_locomotion == nullptr) {
            Serial.println("[CommandProcessor] ✗ Locomotion not available");
            return false;
        }

        uint8_t speed = cmd["speed"] | 120;
        uint8_t turnRate = cmd["turn_rate"] | 64;

        _locomotion->turnLeft(speed, turnRate);

        Serial.print("[CommandProcessor] ✓ Turning left (speed=");
        Serial.print(speed);
        Serial.print(", rate=");
        Serial.print(turnRate);
        Serial.println(")");

        return true;
    }

    /**
     * @brief Handle turn_right command
     *
     * Format: {"command": "turn_right", "speed": 120, "turn_rate": 64}
     */
    bool handleTurnRight(const JsonDocument& cmd) {
        if (_locomotion == nullptr) {
            Serial.println("[CommandProcessor] ✗ Locomotion not available");
            return false;
        }

        uint8_t speed = cmd["speed"] | 120;
        uint8_t turnRate = cmd["turn_rate"] | 64;

        _locomotion->turnRight(speed, turnRate);

        Serial.print("[CommandProcessor] ✓ Turning right (speed=");
        Serial.print(speed);
        Serial.print(", rate=");
        Serial.print(turnRate);
        Serial.println(")");

        return true;
    }

    /**
     * @brief Handle rotate command
     *
     * Format: {"command": "rotate", "speed": 128, "clockwise": true}
     */
    bool handleRotate(const JsonDocument& cmd) {
        if (_locomotion == nullptr) {
            Serial.println("[CommandProcessor] ✗ Locomotion not available");
            return false;
        }

        uint8_t speed = cmd["speed"] | 128;
        bool clockwise = cmd["clockwise"] | true;

        _locomotion->rotateInPlace(speed, clockwise);

        Serial.print("[CommandProcessor] ✓ Rotating ");
        Serial.print(clockwise ? "clockwise" : "counter-clockwise");
        Serial.print(" at speed ");
        Serial.println(speed);

        return true;
    }

    /**
     * @brief Handle stop_motors command
     *
     * Format: {"command": "stop_motors"}
     */
    bool handleStopMotors(const JsonDocument& cmd) {
        if (_locomotion == nullptr) {
            Serial.println("[CommandProcessor] ✗ Locomotion not available");
            return false;
        }

        _locomotion->stopMotors();

        Serial.println("[CommandProcessor] ✓ Motors stopped");

        return true;
    }

    /**
     * @brief Handle enable_motors command
     *
     * Format: {"command": "enable_motors"}
     */
    bool handleEnableMotors(const JsonDocument& cmd) {
        if (_locomotion == nullptr) {
            Serial.println("[CommandProcessor] ✗ Locomotion not available");
            return false;
        }

        _locomotion->enable();

        Serial.println("[CommandProcessor] ✓ Motors enabled");

        return true;
    }

    /**
     * @brief Handle disable_motors command
     *
     * Format: {"command": "disable_motors"}
     */
    bool handleDisableMotors(const JsonDocument& cmd) {
        if (_locomotion == nullptr) {
            Serial.println("[CommandProcessor] ✗ Locomotion not available");
            return false;
        }

        _locomotion->disable();

        Serial.println("[CommandProcessor] ✓ Motors disabled");

        return true;
    }

    // ========================================================================
    // PHASE 4: POWER MANAGEMENT COMMANDS
    // ========================================================================

    /**
     * @brief Handle set_power_mode command
     *
     * Format: {"command": "set_power_mode", "mode": "low_power"}
     * Modes: active, idle, low_power, light_sleep, deep_sleep
     */
    bool handleSetPowerMode(const JsonDocument& cmd) {
        if (_powerManager == nullptr) {
            Serial.println("[CommandProcessor] ✗ PowerManager not available");
            return false;
        }

        if (!cmd.containsKey("mode")) {
            Serial.println("[CommandProcessor] ✗ Missing 'mode' parameter");
            return false;
        }

        const char* modeStr = cmd["mode"];
        PowerMode mode;

        if (strcmp(modeStr, "active") == 0) {
            mode = PowerMode::ACTIVE;
        } else if (strcmp(modeStr, "idle") == 0) {
            mode = PowerMode::IDLE;
        } else if (strcmp(modeStr, "low_power") == 0) {
            mode = PowerMode::LOW_POWER;
        } else if (strcmp(modeStr, "light_sleep") == 0) {
            mode = PowerMode::LIGHT_SLEEP;
        } else if (strcmp(modeStr, "deep_sleep") == 0) {
            mode = PowerMode::DEEP_SLEEP;
        } else {
            Serial.print("[CommandProcessor] ✗ Invalid power mode: ");
            Serial.println(modeStr);
            return false;
        }

        _powerManager->setPowerMode(mode);

        Serial.print("[CommandProcessor] ✓ Power mode set to ");
        Serial.println(modeStr);

        return true;
    }

    /**
     * @brief Handle set_operational_mode command
     *
     * Format: {"command": "set_operational_mode", "mode": "active"}
     * Modes: idle, active, maintenance, error
     */
    bool handleSetOperationalMode(const JsonDocument& cmd) {
        if (!cmd.containsKey("mode")) {
            Serial.println("[CommandProcessor] ✗ Missing 'mode' parameter");
            return false;
        }

        const char* mode = cmd["mode"];
        _currentMode = String(mode);

        Serial.print("[CommandProcessor] ✓ Operational mode set to ");
        Serial.println(mode);

        // Register activity with power manager
        if (_powerManager != nullptr) {
            _powerManager->registerActivity();
        }

        return true;
    }

    // ========================================================================
    // PHASE 4: ERROR MANAGEMENT COMMANDS
    // ========================================================================

    /**
     * @brief Handle clear_errors command
     *
     * Format: {"command": "clear_errors"}
     */
    bool handleClearErrors(const JsonDocument& cmd) {
        if (_errorManager == nullptr) {
            Serial.println("[CommandProcessor] ✗ ErrorManager not available");
            return false;
        }

        _errorManager->clearAllErrors();

        Serial.println("[CommandProcessor] ✓ Error log cleared");

        return true;
    }

    /**
     * @brief Handle perform_health_check command
     *
     * Format: {"command": "perform_health_check"}
     */
    bool handlePerformHealthCheck(const JsonDocument& cmd) {
        if (_errorManager == nullptr) {
            Serial.println("[CommandProcessor] ✗ ErrorManager not available");
            return false;
        }

        _errorManager->performSystemHealthCheck();

        Serial.println("[CommandProcessor] ✓ Health check completed");

        return true;
    }

    // Private member variables
    ServoArm* _servo;               ///< Pointer to servo subsystem
    Locomotion* _locomotion;        ///< Pointer to locomotion subsystem
    PowerManager* _powerManager;    ///< Pointer to power manager (Phase 4)
    ErrorManager* _errorManager;    ///< Pointer to error manager (Phase 4)
    unsigned long _commandCount;    ///< Total commands processed
    unsigned long _errorCount;      ///< Failed commands count
    String _currentMode;            ///< Current operational mode
};

#endif // COMMAND_PROCESSOR_H
