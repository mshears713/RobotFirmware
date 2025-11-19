/**
 * @file PowerManager.cpp
 * @brief Implementation of PowerManager class
 *
 * ESP32 Robotics Firmware - Phase 4: Power Management
 */

#include "PowerManager.h"
#include "driver/rtc_io.h"

// ============================================================================
// CONSTRUCTOR
// ============================================================================

PowerManager::PowerManager(
    unsigned long idleTimeout_ms,
    unsigned long lowPowerTimeout_ms,
    unsigned long sleepTimeout_ms
) : _currentMode(PowerMode::ACTIVE),
    _previousMode(PowerMode::ACTIVE),
    _lastActivityTime(0),
    _idleTimeout(idleTimeout_ms),
    _lowPowerTimeout(lowPowerTimeout_ms),
    _sleepTimeout(sleepTimeout_ms),
    _wifiEnabled(true),
    _wakeTimerDuration(0),
    _wakeGpioPin(GPIO_NUM_0),
    _activityCounter(0),
    _lastWakeReason(WakeReason::NONE)
{
    // Constructor initializes all member variables
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool PowerManager::initialize() {
    Serial.println("[PowerManager] Initializing power management...");

    // Check wake-up reason
    _lastWakeReason = getWakeReason();

    if (_lastWakeReason != WakeReason::NONE) {
        Serial.print("[PowerManager] Woke from sleep: ");
        Serial.println(getWakeReasonString());
    }

    // Register initial activity
    registerActivity();

    _isInitialized = true;
    Serial.println("[PowerManager] ✓ Power management ready");
    Serial.print("[PowerManager]   Idle timeout: ");
    Serial.print(_idleTimeout / 1000);
    Serial.println("s");
    Serial.print("[PowerManager]   Low-power timeout: ");
    Serial.print(_lowPowerTimeout / 1000);
    Serial.println("s");
    Serial.print("[PowerManager]   Sleep timeout: ");
    Serial.print(_sleepTimeout / 1000);
    Serial.println("s");

    return true;
}

// ============================================================================
// UPDATE FUNCTION
// ============================================================================

void PowerManager::update() {
    if (!_isInitialized) return;

    unsigned long inactiveTime = getTimeSinceActivity();

    // EDUCATIONAL: State machine for power management
    // Check inactivity time and transition to appropriate mode
    switch (_currentMode) {
        case PowerMode::ACTIVE:
            // Transition to IDLE after idle timeout
            if (inactiveTime >= _idleTimeout) {
                enterIdleMode();
            }
            break;

        case PowerMode::IDLE:
            // Transition to LOW_POWER after low-power timeout
            if (inactiveTime >= _lowPowerTimeout) {
                enterLowPowerMode();
            }
            break;

        case PowerMode::LOW_POWER:
            // Transition to LIGHT_SLEEP after sleep timeout
            if (inactiveTime >= _sleepTimeout) {
                // Note: Light sleep is initiated manually via setPowerMode
                // to avoid automatic sleep during critical operations
                Serial.println("[PowerManager] Sleep timeout reached");
                Serial.println("[PowerManager] Call setPowerMode(LIGHT_SLEEP) to enter sleep");
            }
            break;

        case PowerMode::LIGHT_SLEEP:
        case PowerMode::DEEP_SLEEP:
            // These modes are blocking - shouldn't reach here
            break;
    }

    _lastUpdate = millis();
}

// ============================================================================
// POWER MODE MANAGEMENT
// ============================================================================

void PowerManager::enterIdleMode() {
    Serial.println("[PowerManager] Entering IDLE mode");
    Serial.println("[PowerManager]   Reducing activity, Wi-Fi active");

    _previousMode = _currentMode;
    _currentMode = PowerMode::IDLE;

    // Could reduce CPU frequency here for additional power savings
    // setCpuFrequencyMhz(80);  // Reduce from 240MHz to 80MHz
}

void PowerManager::enterLowPowerMode() {
    Serial.println("[PowerManager] Entering LOW_POWER mode");
    Serial.println("[PowerManager]   Modem sleep enabled");

    _previousMode = _currentMode;
    _currentMode = PowerMode::LOW_POWER;

    // EDUCATIONAL: Enable Wi-Fi modem sleep to save power
    // The modem will sleep between DTIM beacon intervals
    if (_wifiEnabled) {
        esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
        Serial.println("[PowerManager]   Wi-Fi modem sleep: ENABLED");
    }
}

void PowerManager::enterLightSleep() {
    Serial.println("[PowerManager] Entering LIGHT_SLEEP mode");
    Serial.print("[PowerManager]   Wake timer: ");

    _previousMode = _currentMode;
    _currentMode = PowerMode::LIGHT_SLEEP;

    // EDUCATIONAL: Light sleep pauses CPU but maintains RAM
    // Wake-up is fast (~3ms) compared to deep sleep (~100ms)

    if (_wakeTimerDuration > 0) {
        Serial.print(_wakeTimerDuration / 1000000);
        Serial.println("s");
        esp_sleep_enable_timer_wakeup(_wakeTimerDuration);
    } else {
        Serial.println("not configured");
    }

    // Configure GPIO wake if set
    if (_wakeGpioPin != GPIO_NUM_0) {
        Serial.print("[PowerManager]   Wake GPIO: ");
        Serial.println(_wakeGpioPin);
    }

    Serial.println("[PowerManager] Entering sleep NOW...");
    Serial.flush();  // Ensure all serial data is sent

    // Enter light sleep
    esp_light_sleep_start();

    // EXECUTION RESUMES HERE AFTER WAKE-UP
    exitSleepMode();
}

void PowerManager::enterDeepSleep() {
    Serial.println("[PowerManager] Entering DEEP_SLEEP mode");
    Serial.print("[PowerManager]   Wake timer: ");

    _previousMode = _currentMode;
    _currentMode = PowerMode::DEEP_SLEEP;

    // EDUCATIONAL: Deep sleep shuts down CPU and most peripherals
    // Only RTC and ULP coprocessor remain active
    // Wake-up requires full reboot (~100-200ms)

    if (_wakeTimerDuration > 0) {
        Serial.print(_wakeTimerDuration / 1000000);
        Serial.println("s");
        esp_sleep_enable_timer_wakeup(_wakeTimerDuration);
    } else {
        Serial.println("not configured");
    }

    // Configure GPIO wake if set
    if (_wakeGpioPin != GPIO_NUM_0) {
        Serial.print("[PowerManager]   Wake GPIO: ");
        Serial.println(_wakeGpioPin);

        // Configure RTC GPIO for wake
        rtc_gpio_pullup_en(_wakeGpioPin);
        rtc_gpio_pulldown_dis(_wakeGpioPin);
    }

    Serial.println("[PowerManager] Entering deep sleep NOW...");
    Serial.println("[PowerManager] System will restart on wake-up");
    Serial.flush();

    // Enter deep sleep (this function does not return)
    esp_deep_sleep_start();
}

void PowerManager::exitSleepMode() {
    // Determine wake reason
    _lastWakeReason = getWakeReason();

    Serial.println();
    Serial.println("[PowerManager] ⏰ WAKE UP!");
    Serial.print("[PowerManager]   Wake reason: ");
    Serial.println(getWakeReasonString());

    // Return to active mode
    _currentMode = PowerMode::ACTIVE;

    // Disable Wi-Fi modem sleep
    if (_wifiEnabled) {
        esp_wifi_set_ps(WIFI_PS_NONE);
    }

    // Register activity
    registerActivity();
}

void PowerManager::setPowerMode(PowerMode mode) {
    if (mode == _currentMode) return;

    Serial.print("[PowerManager] Mode change: ");
    Serial.print(getPowerModeString());
    Serial.print(" → ");

    switch (mode) {
        case PowerMode::ACTIVE:
            _currentMode = PowerMode::ACTIVE;
            // Disable Wi-Fi sleep
            if (_wifiEnabled) {
                esp_wifi_set_ps(WIFI_PS_NONE);
            }
            registerActivity();
            break;

        case PowerMode::IDLE:
            enterIdleMode();
            break;

        case PowerMode::LOW_POWER:
            enterLowPowerMode();
            break;

        case PowerMode::LIGHT_SLEEP:
            enterLightSleep();
            // Note: Function returns after wake-up
            break;

        case PowerMode::DEEP_SLEEP:
            enterDeepSleep();
            // Note: This function does not return
            break;
    }

    Serial.println(getPowerModeString());
}

// ============================================================================
// ACTIVITY TRACKING
// ============================================================================

void PowerManager::registerActivity() {
    _lastActivityTime = millis();
    _activityCounter++;

    // If we're in a reduced power mode, return to ACTIVE
    if (_currentMode != PowerMode::ACTIVE) {
        _previousMode = _currentMode;
        _currentMode = PowerMode::ACTIVE;

        // Disable Wi-Fi sleep
        if (_wifiEnabled) {
            esp_wifi_set_ps(WIFI_PS_NONE);
        }
    }
}

unsigned long PowerManager::getTimeSinceActivity() const {
    return millis() - _lastActivityTime;
}

// ============================================================================
// WAKE CONFIGURATION
// ============================================================================

void PowerManager::configureWakeTimer(uint64_t duration_us) {
    _wakeTimerDuration = duration_us;

    Serial.print("[PowerManager] Wake timer configured: ");
    Serial.print(duration_us / 1000000);
    Serial.println("s");
}

void PowerManager::configureWakeGpio(gpio_num_t pin, int level) {
    _wakeGpioPin = pin;

    // EDUCATIONAL: Configure GPIO wake-up for light and deep sleep
    // ext0 wake source uses a single pin
    esp_sleep_enable_ext0_wakeup(pin, level);

    Serial.print("[PowerManager] Wake GPIO configured: GPIO");
    Serial.print(pin);
    Serial.print(" on ");
    Serial.println(level == 1 ? "HIGH" : "LOW");
}

WakeReason PowerManager::getWakeReason() {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0:
        case ESP_SLEEP_WAKEUP_EXT1:
            return WakeReason::GPIO;

        case ESP_SLEEP_WAKEUP_TIMER:
            return WakeReason::TIMER;

        case ESP_SLEEP_WAKEUP_TOUCHPAD:
            return WakeReason::TOUCHPAD;

        case ESP_SLEEP_WAKEUP_ULP:
            return WakeReason::ULP;

        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            return WakeReason::NONE;
    }
}

String PowerManager::getWakeReasonString() const {
    switch (_lastWakeReason) {
        case WakeReason::TIMER:
            return "Timer";
        case WakeReason::GPIO:
            return "GPIO/Button";
        case WakeReason::TOUCHPAD:
            return "Touchpad";
        case WakeReason::ULP:
            return "ULP Coprocessor";
        case WakeReason::NONE:
            return "Power-on/Reset";
        default:
            return "Unknown";
    }
}

// ============================================================================
// WIFI POWER MANAGEMENT
// ============================================================================

void PowerManager::enableWifiPowerSave(bool enable) {
    _wifiEnabled = enable;

    if (enable) {
        // Enable Wi-Fi modem sleep in low-power modes
        if (_currentMode == PowerMode::LOW_POWER) {
            esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
        }
        Serial.println("[PowerManager] Wi-Fi power save: ENABLED");
    } else {
        // Disable Wi-Fi sleep
        esp_wifi_set_ps(WIFI_PS_NONE);
        Serial.println("[PowerManager] Wi-Fi power save: DISABLED");
    }
}

// ============================================================================
// STATUS AND REPORTING
// ============================================================================

bool PowerManager::isReady() {
    return _isInitialized;
}

String PowerManager::getStatus() {
    String status = "PowerManager: ";
    status += getPowerModeString();
    status += " (";
    status += String(getTimeSinceActivity() / 1000);
    status += "s idle)";
    return status;
}

String PowerManager::getPowerModeString() const {
    switch (_currentMode) {
        case PowerMode::ACTIVE:     return "ACTIVE";
        case PowerMode::IDLE:       return "IDLE";
        case PowerMode::LOW_POWER:  return "LOW_POWER";
        case PowerMode::LIGHT_SLEEP: return "LIGHT_SLEEP";
        case PowerMode::DEEP_SLEEP: return "DEEP_SLEEP";
        default:                    return "UNKNOWN";
    }
}

float PowerManager::getEstimatedCurrentDraw() const {
    // EDUCATIONAL: Approximate current consumption estimates
    // Based on ESP32 datasheet typical values
    switch (_currentMode) {
        case PowerMode::ACTIVE:
            return 160.0f;  // ~160mA with Wi-Fi active

        case PowerMode::IDLE:
            return 100.0f;  // ~100mA reduced activity

        case PowerMode::LOW_POWER:
            return 30.0f;   // ~30mA with modem sleep

        case PowerMode::LIGHT_SLEEP:
            return 0.8f;    // ~0.8mA in light sleep

        case PowerMode::DEEP_SLEEP:
            return 0.01f;   // ~10µA in deep sleep

        default:
            return 160.0f;
    }
}
