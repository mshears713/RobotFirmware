/**
 * @file PowerManager.h
 * @brief Low-power mode management for ESP32
 *
 * ESP32 Robotics Firmware - Phase 4: Power Management
 *
 * EDUCATIONAL NOTE: The ESP32 supports multiple low-power modes to conserve energy:
 * 1. Active Mode: Normal operation, all peripherals active (~160-260mA)
 * 2. Modem Sleep: Wi-Fi/BT off, CPU active (~30-68mA)
 * 3. Light Sleep: CPU paused, peripherals off, quick wake (~0.8mA)
 * 4. Deep Sleep: Complete shutdown, only RTC active (~10μA)
 *
 * This class manages transitions between power states based on activity levels.
 */

#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <Arduino.h>
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "RobotSubsystem.h"

/**
 * @brief Power modes for the robot system
 */
enum class PowerMode {
    ACTIVE,        // All systems operational
    IDLE,          // Reduced activity, Wi-Fi active
    LOW_POWER,     // Modem sleep, minimal activity
    LIGHT_SLEEP,   // Light sleep between activities
    DEEP_SLEEP     // Deep sleep (wake via timer or GPIO)
};

/**
 * @brief Wake-up reasons for logging
 */
enum class WakeReason {
    NONE,
    TIMER,
    GPIO,
    TOUCHPAD,
    ULP,
    UNKNOWN
};

/**
 * @brief PowerManager class for energy-efficient operation
 *
 * EDUCATIONAL: This class demonstrates how embedded systems manage power
 * consumption to extend battery life. It monitors system activity and
 * automatically transitions to low-power modes when appropriate.
 */
class PowerManager : public RobotSubsystem {
private:
    PowerMode _currentMode;
    PowerMode _previousMode;

    unsigned long _lastActivityTime;
    unsigned long _idleTimeout;          // Time before entering idle mode
    unsigned long _lowPowerTimeout;      // Time before entering low-power mode
    unsigned long _sleepTimeout;         // Time before entering sleep mode

    bool _wifiEnabled;
    unsigned long _wakeTimerDuration;    // Duration for timer-based wake
    gpio_num_t _wakeGpioPin;             // GPIO pin for wake-up

    int _activityCounter;
    WakeReason _lastWakeReason;

    // Internal helper methods
    void enterIdleMode();
    void enterLowPowerMode();
    void enterLightSleep();
    void enterDeepSleep();
    void exitSleepMode();
    WakeReason getWakeReason();

public:
    /**
     * @brief Constructor
     * @param idleTimeout_ms Time in ms before entering idle mode (default 30s)
     * @param lowPowerTimeout_ms Time in ms before low-power mode (default 60s)
     * @param sleepTimeout_ms Time in ms before sleep mode (default 5 minutes)
     */
    PowerManager(
        unsigned long idleTimeout_ms = 30000,
        unsigned long lowPowerTimeout_ms = 60000,
        unsigned long sleepTimeout_ms = 300000
    );

    /**
     * @brief Initialize power management system
     * @return true if successful
     */
    bool initialize() override;

    /**
     * @brief Update power state based on activity
     *
     * EDUCATIONAL: This is called every loop iteration to monitor
     * inactivity and trigger appropriate power-saving modes.
     */
    void update() override;

    /**
     * @brief Check if power manager is ready
     * @return true if initialized
     */
    bool isReady() override;

    /**
     * @brief Get current status string
     * @return Status description
     */
    String getStatus() override;

    /**
     * @brief Register activity to prevent sleep
     *
     * EDUCATIONAL: Call this whenever something happens (command received,
     * sensor reading, user input) to reset the inactivity timer.
     */
    void registerActivity();

    /**
     * @brief Manually set power mode
     * @param mode Desired power mode
     */
    void setPowerMode(PowerMode mode);

    /**
     * @brief Get current power mode
     * @return Current mode
     */
    PowerMode getPowerMode() const { return _currentMode; }

    /**
     * @brief Get power mode as string
     * @return Mode name
     */
    String getPowerModeString() const;

    /**
     * @brief Configure wake-up timer
     * @param duration_us Wake-up duration in microseconds
     *
     * EDUCATIONAL: This sets a timer that will wake the ESP32 from
     * sleep mode after the specified duration. Useful for periodic
     * sensor readings or status updates.
     */
    void configureWakeTimer(uint64_t duration_us);

    /**
     * @brief Configure GPIO wake-up
     * @param pin GPIO pin number to use for wake-up
     * @param level Level to trigger wake (0=LOW, 1=HIGH)
     *
     * EDUCATIONAL: This configures a GPIO pin to wake the ESP32
     * from sleep when it reaches the specified level. Perfect for
     * button presses or external interrupt signals.
     */
    void configureWakeGpio(gpio_num_t pin, int level);

    /**
     * @brief Get time since last activity
     * @return Milliseconds since last activity
     */
    unsigned long getTimeSinceActivity() const;

    /**
     * @brief Get last wake reason
     * @return Wake reason enum
     */
    WakeReason getLastWakeReason() const { return _lastWakeReason; }

    /**
     * @brief Get wake reason as string
     * @return Wake reason description
     */
    String getWakeReasonString() const;

    /**
     * @brief Enable/disable Wi-Fi power management
     * @param enable true to enable Wi-Fi sleep modes
     *
     * EDUCATIONAL: Wi-Fi consumes significant power. When enabled,
     * the modem will sleep between transmissions to save energy.
     */
    void enableWifiPowerSave(bool enable);

    /**
     * @brief Get estimated current draw in milliamps
     * @return Approximate current consumption
     *
     * EDUCATIONAL: This provides rough estimates of power consumption
     * based on the current operating mode. Useful for battery life calculations.
     */
    float getEstimatedCurrentDraw() const;
};

#endif // POWER_MANAGER_H
