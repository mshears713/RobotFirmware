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
 * - set_mode: Change operational mode
 */

#ifndef COMMAND_PROCESSOR_H
#define COMMAND_PROCESSOR_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "ServoArm.h"
#include "Locomotion.h"

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
     */
    CommandProcessor(ServoArm* servo, Locomotion* locomotion)
        : _servo(servo),
          _locomotion(locomotion),
          _commandCount(0),
          _errorCount(0)
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

    // Private member variables
    ServoArm* _servo;               ///< Pointer to servo subsystem
    Locomotion* _locomotion;        ///< Pointer to locomotion subsystem
    unsigned long _commandCount;    ///< Total commands processed
    unsigned long _errorCount;      ///< Failed commands count
};

#endif // COMMAND_PROCESSOR_H
