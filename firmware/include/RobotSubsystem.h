/**
 * @file RobotSubsystem.h
 * @brief Abstract base class for all robot subsystems
 *
 * EDUCATIONAL NOTE: This demonstrates the Template Method design pattern and
 * polymorphism in embedded systems. By defining a common interface, we can
 * manage different hardware components (servos, sensors, motors) uniformly
 * in the main loop.
 *
 * Benefits of this architecture:
 * 1. Modularity: Each subsystem is self-contained
 * 2. Testability: Subsystems can be tested independently
 * 3. Extensibility: New subsystems follow the same pattern
 * 4. Maintainability: Clear separation of concerns
 */

#ifndef ROBOT_SUBSYSTEM_H
#define ROBOT_SUBSYSTEM_H

#include <Arduino.h>

/**
 * @class RobotSubsystem
 * @brief Abstract base class providing common interface for robot hardware subsystems
 *
 * EDUCATIONAL: In C++, an abstract class contains at least one pure virtual
 * function (marked with '= 0'). This class cannot be instantiated directly;
 * instead, concrete classes must inherit from it and implement all pure virtual methods.
 *
 * Each subsystem (servo, sensor, motor) will inherit from this class and
 * implement these methods according to its specific hardware requirements.
 */
class RobotSubsystem {
public:
    /**
     * @brief Virtual destructor
     *
     * EDUCATIONAL: When using polymorphism (pointers to base class pointing
     * to derived objects), the destructor must be virtual to ensure proper
     * cleanup of derived class resources when the object is deleted.
     */
    virtual ~RobotSubsystem() {}

    /**
     * @brief Initialize the hardware subsystem
     *
     * EDUCATIONAL: This method is called once during setup() to configure
     * hardware pins, initialize sensors, attach servos, etc.
     * It should perform all one-time initialization and return success/failure.
     *
     * @return true if initialization succeeded, false otherwise
     *
     * Implementation example:
     * @code
     * bool MySubsystem::initialize() {
     *     pinMode(_pin, OUTPUT);
     *     _isInitialized = true;
     *     return true;
     * }
     * @endcode
     */
    virtual bool initialize() = 0;

    /**
     * @brief Update the subsystem state (called every loop iteration)
     *
     * EDUCATIONAL: This method is called repeatedly in the main loop to:
     * - Read sensor values
     * - Update actuator positions
     * - Process state transitions
     * - Handle timing-based operations
     *
     * This should be NON-BLOCKING! Never use delay() inside update().
     * Use millis() for timing instead.
     *
     * Implementation example:
     * @code
     * void MySubsystem::update() {
     *     if (!_isInitialized) return;
     *
     *     unsigned long now = millis();
     *     if (now - _lastUpdate >= UPDATE_INTERVAL) {
     *         // Do periodic work here
     *         _lastUpdate = now;
     *     }
     * }
     * @endcode
     */
    virtual void update() = 0;

    /**
     * @brief Check if subsystem is ready to operate
     *
     * EDUCATIONAL: This method allows the main system to verify that
     * a subsystem is properly initialized and functioning. Useful for:
     * - Pre-flight checks
     * - Error detection
     * - Status reporting
     *
     * @return true if subsystem is initialized and operational, false otherwise
     */
    virtual bool isReady() = 0;

    /**
     * @brief Get current status as a human-readable string
     *
     * EDUCATIONAL: This method provides diagnostic information for:
     * - Serial debugging output
     * - Telemetry reporting
     * - User interface display
     *
     * @return String describing current subsystem status
     *
     * Implementation example:
     * @code
     * String MySubsystem::getStatus() {
     *     if (!_isInitialized) return "Not initialized";
     *     return "Active, value=" + String(_currentValue);
     * }
     * @endcode
     */
    virtual String getStatus() = 0;

    /**
     * @brief Get the subsystem name/identifier
     *
     * EDUCATIONAL: Useful for logging and debugging to identify which
     * subsystem is reporting information.
     *
     * @return Subsystem name (e.g., "ServoArm", "I2CSensor", "Locomotion")
     */
    virtual const char* getName() const = 0;

    /**
     * @brief Enable the subsystem
     *
     * EDUCATIONAL: Some subsystems may need to be enabled/disabled dynamically.
     * For example, disabling motors when in idle mode to save power.
     * Default implementation does nothing; override if needed.
     */
    virtual void enable() {
        _isEnabled = true;
    }

    /**
     * @brief Disable the subsystem
     */
    virtual void disable() {
        _isEnabled = false;
    }

    /**
     * @brief Check if subsystem is enabled
     *
     * @return true if enabled, false otherwise
     */
    bool isEnabled() const {
        return _isEnabled;
    }

    /**
     * @brief Reset the subsystem to initial state
     *
     * EDUCATIONAL: Useful for error recovery or mode changes.
     * Default implementation does nothing; override if needed.
     */
    virtual void reset() {
        // Override in derived classes if reset behavior needed
    }

protected:
    /**
     * EDUCATIONAL: Protected members are accessible to derived classes
     * but not to external code. This encapsulates internal state while
     * allowing subclasses to use these common fields.
     */

    /**
     * @brief Flag indicating if subsystem has been initialized
     */
    bool _isInitialized = false;

    /**
     * @brief Flag indicating if subsystem is enabled
     */
    bool _isEnabled = true;

    /**
     * @brief Timestamp of last update() call
     *
     * EDUCATIONAL: Used for non-blocking timing. By tracking when we last
     * performed an action, we can implement periodic tasks without delay().
     * Compare (millis() - _lastUpdate) against a desired interval.
     */
    unsigned long _lastUpdate = 0;

    /**
     * @brief Helper function to check if interval has elapsed
     *
     * EDUCATIONAL: This utility function simplifies non-blocking timing.
     * Instead of writing the same millis() comparison everywhere, use this.
     *
     * @param interval Time interval in milliseconds
     * @return true if the specified interval has elapsed since _lastUpdate
     *
     * Usage example:
     * @code
     * if (hasIntervalElapsed(1000)) {
     *     // Do something every 1 second
     *     _lastUpdate = millis();  // Reset timer
     * }
     * @endcode
     */
    bool hasIntervalElapsed(unsigned long interval) const {
        return (millis() - _lastUpdate) >= interval;
    }

    /**
     * @brief Update the last update timestamp
     *
     * EDUCATIONAL: Call this after performing a periodic action to reset
     * the interval timer.
     */
    void updateTimestamp() {
        _lastUpdate = millis();
    }
};

#endif // ROBOT_SUBSYSTEM_H
