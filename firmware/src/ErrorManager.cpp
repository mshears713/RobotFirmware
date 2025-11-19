/**
 * @file ErrorManager.cpp
 * @brief Implementation of ErrorManager class
 *
 * ESP32 Robotics Firmware - Phase 4: Error Management
 */

#include "ErrorManager.h"
#include "WiFi.h"
#include "esp_system.h"
#include "ArduinoJson.h"

// ============================================================================
// CONSTRUCTOR
// ============================================================================

ErrorManager::ErrorManager()
    : _logIndex(0),
      _logCount(0),
      _currentError(ErrorCode::NONE),
      _currentSeverity(ErrorSeverity::INFO),
      _currentMessage(""),
      _totalErrors(0),
      _totalWarnings(0),
      _totalRecoveries(0),
      _failedRecoveries(0),
      _lastErrorTime(0),
      _autoRecoveryEnabled(true),
      _maxRetries(3)
{
    // Initialize error log array
    for (int i = 0; i < MAX_ERROR_LOG_ENTRIES; i++) {
        _errorLog[i].timestamp = 0;
        _errorLog[i].code = ErrorCode::NONE;
        _errorLog[i].severity = ErrorSeverity::INFO;
        _errorLog[i].message = "";
        _errorLog[i].subsystem = "";
        _errorLog[i].recovered = false;
    }
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool ErrorManager::initialize() {
    Serial.println("[ErrorManager] Initializing error management...");

    clearAllErrors();

    _isInitialized = true;
    Serial.println("[ErrorManager] ✓ Error management ready");
    Serial.print("[ErrorManager]   Auto-recovery: ");
    Serial.println(_autoRecoveryEnabled ? "ENABLED" : "DISABLED");
    Serial.print("[ErrorManager]   Max retries: ");
    Serial.println(_maxRetries);

    return true;
}

// ============================================================================
// UPDATE
// ============================================================================

void ErrorManager::update() {
    if (!_isInitialized) return;

    // EDUCATIONAL: Periodically check for system-level errors
    // This runs in the background to catch issues that subsystems
    // might not report themselves

    // Check for low memory
    if (checkLowMemory()) {
        reportWarning("Low memory detected", "System");
    }

    _lastUpdate = millis();
}

// ============================================================================
// STATUS
// ============================================================================

bool ErrorManager::isReady() {
    return _isInitialized;
}

String ErrorManager::getStatus() {
    String status = "ErrorManager: ";

    if (hasError()) {
        status += severityToString(_currentSeverity);
        status += " (";
        status += errorCodeToString(_currentError);
        status += ")";
    } else {
        status += "OK - ";
        status += String(_totalErrors);
        status += " errors, ";
        status += String(_totalWarnings);
        status += " warnings";
    }

    return status;
}

// ============================================================================
// ERROR REPORTING
// ============================================================================

void ErrorManager::reportError(
    ErrorCode code,
    ErrorSeverity severity,
    const String& message,
    const String& subsystem
) {
    // EDUCATIONAL: Error debouncing - prevent spam from repeated errors
    unsigned long now = millis();
    if (code == _currentError && (now - _lastErrorTime) < ERROR_DEBOUNCE_TIME) {
        return;  // Same error reported too quickly, ignore
    }

    _lastErrorTime = now;

    // Update current error state
    _currentError = code;
    _currentSeverity = severity;
    _currentMessage = message;

    // Update statistics
    if (severity == ErrorSeverity::WARNING) {
        _totalWarnings++;
    } else if (severity >= ErrorSeverity::ERROR) {
        _totalErrors++;
    }

    // Add to log
    addToLog(code, severity, message, subsystem);

    // Print to Serial
    Serial.println();
    Serial.println("========================================");
    Serial.print("  ");

    switch (severity) {
        case ErrorSeverity::INFO:
            Serial.print("ℹ INFO");
            break;
        case ErrorSeverity::WARNING:
            Serial.print("⚠ WARNING");
            break;
        case ErrorSeverity::ERROR:
            Serial.print("❌ ERROR");
            break;
        case ErrorSeverity::CRITICAL:
            Serial.print("🔥 CRITICAL");
            break;
    }

    Serial.print(" [");
    Serial.print(subsystem);
    Serial.println("]");
    Serial.print("  Code: ");
    Serial.println((int)code);
    Serial.print("  Message: ");
    Serial.println(message);
    Serial.println("========================================");
    Serial.println();

    // Attempt automatic recovery if enabled
    if (_autoRecoveryEnabled && severity >= ErrorSeverity::ERROR) {
        Serial.println("[ErrorManager] Attempting automatic recovery...");
        if (attemptRecovery(code, subsystem)) {
            Serial.println("[ErrorManager] ✓ Recovery successful");
            _totalRecoveries++;
        } else {
            Serial.println("[ErrorManager] ✗ Recovery failed");
            _failedRecoveries++;
        }
    }
}

void ErrorManager::reportInfo(const String& message, const String& subsystem) {
    reportError(ErrorCode::NONE, ErrorSeverity::INFO, message, subsystem);
}

void ErrorManager::reportWarning(const String& message, const String& subsystem) {
    reportError(ErrorCode::UNKNOWN_ERROR, ErrorSeverity::WARNING, message, subsystem);
}

void ErrorManager::clearError() {
    _currentError = ErrorCode::NONE;
    _currentSeverity = ErrorSeverity::INFO;
    _currentMessage = "";

    Serial.println("[ErrorManager] Error cleared");
}

void ErrorManager::clearAllErrors() {
    _logIndex = 0;
    _logCount = 0;
    _currentError = ErrorCode::NONE;
    _currentSeverity = ErrorSeverity::INFO;
    _currentMessage = "";
    _totalErrors = 0;
    _totalWarnings = 0;
    _totalRecoveries = 0;
    _failedRecoveries = 0;

    Serial.println("[ErrorManager] All errors cleared");
}

// ============================================================================
// ERROR RECOVERY
// ============================================================================

bool ErrorManager::attemptRecovery(ErrorCode code, const String& subsystem) {
    RecoveryAction action = getRecoveryAction(code);

    Serial.print("[ErrorManager] Recovery action: ");

    switch (action) {
        case RecoveryAction::NONE:
            Serial.println("None needed");
            return true;

        case RecoveryAction::RETRY:
            Serial.println("Retry operation");
            // The calling subsystem should retry
            return false;  // Caller must handle retry

        case RecoveryAction::RESET_SUBSYSTEM:
            Serial.print("Reset ");
            Serial.println(subsystem);
            // Subsystem should be reinitialized
            return false;  // Caller must handle reset

        case RecoveryAction::DISABLE_SUBSYSTEM:
            Serial.print("Disable ");
            Serial.println(subsystem);
            // Subsystem will be disabled
            return true;

        case RecoveryAction::SYSTEM_RESTART:
            Serial.println("System restart required");
            Serial.println("[ErrorManager] Restarting in 3 seconds...");
            delay(3000);
            esp_restart();
            return false;  // Won't reach here

        default:
            return false;
    }
}

RecoveryAction ErrorManager::getRecoveryAction(ErrorCode code) {
    // EDUCATIONAL: Define recovery strategies for different error types
    // This is where you customize how the system responds to each error

    switch (code) {
        // Wi-Fi errors - retry connection
        case ErrorCode::WIFI_CONNECTION_FAILED:
        case ErrorCode::WIFI_DISCONNECTED:
        case ErrorCode::WIFI_TIMEOUT:
            return RecoveryAction::RETRY;

        // Sensor errors - reset subsystem
        case ErrorCode::I2C_SENSOR_NOT_FOUND:
        case ErrorCode::I2C_COMM_ERROR:
        case ErrorCode::SPI_COMM_ERROR:
            return RecoveryAction::RESET_SUBSYSTEM;

        // Invalid sensor data - no action needed (might be temporary)
        case ErrorCode::SENSOR_DATA_INVALID:
        case ErrorCode::SENSOR_TIMEOUT:
            return RecoveryAction::NONE;

        // Servo/Motor errors - reset subsystem
        case ErrorCode::SERVO_ATTACH_FAILED:
        case ErrorCode::MOTOR_DRIVER_ERROR:
        case ErrorCode::PWM_CONFIG_ERROR:
            return RecoveryAction::RESET_SUBSYSTEM;

        // System errors - might need restart
        case ErrorCode::LOW_MEMORY:
            return RecoveryAction::NONE;  // Log and continue

        case ErrorCode::WATCHDOG_TIMEOUT:
        case ErrorCode::TEMPERATURE_HIGH:
            return RecoveryAction::SYSTEM_RESTART;

        case ErrorCode::POWER_LOW_BATTERY:
            return RecoveryAction::DISABLE_SUBSYSTEM;  // Enter low-power mode

        // Command errors - no recovery needed
        case ErrorCode::INVALID_COMMAND:
        case ErrorCode::COMMAND_PARSE_ERROR:
        case ErrorCode::COMMAND_EXECUTION_FAILED:
            return RecoveryAction::NONE;

        default:
            return RecoveryAction::NONE;
    }
}

bool ErrorManager::recoverFromError() {
    if (!hasError()) {
        return true;
    }

    String subsystem = "";
    if (_logCount > 0) {
        subsystem = getMostRecentError().subsystem;
    }

    return attemptRecovery(_currentError, subsystem);
}

// ============================================================================
// ERROR LOG MANAGEMENT
// ============================================================================

void ErrorManager::addToLog(
    ErrorCode code,
    ErrorSeverity severity,
    const String& message,
    const String& subsystem
) {
    // EDUCATIONAL: Circular buffer for error log
    // Newest entries overwrite oldest when log is full

    ErrorLogEntry& entry = _errorLog[_logIndex];
    entry.timestamp = millis();
    entry.code = code;
    entry.severity = severity;
    entry.message = message;
    entry.subsystem = subsystem;
    entry.recovered = false;

    _logIndex = (_logIndex + 1) % MAX_ERROR_LOG_ENTRIES;

    if (_logCount < MAX_ERROR_LOG_ENTRIES) {
        _logCount++;
    }
}

const ErrorLogEntry& ErrorManager::getLogEntry(int index) const {
    // Clamp index to valid range
    if (index < 0 || index >= _logCount) {
        index = 0;
    }

    return _errorLog[index];
}

const ErrorLogEntry& ErrorManager::getMostRecentError() const {
    if (_logCount == 0) {
        return _errorLog[0];  // Empty entry
    }

    int recentIndex = (_logIndex - 1 + MAX_ERROR_LOG_ENTRIES) % MAX_ERROR_LOG_ENTRIES;
    return _errorLog[recentIndex];
}

void ErrorManager::printErrorLog() {
    Serial.println();
    Serial.println("========================================");
    Serial.println("  ERROR LOG");
    Serial.println("========================================");

    if (_logCount == 0) {
        Serial.println("  No errors logged");
    } else {
        for (int i = 0; i < _logCount; i++) {
            const ErrorLogEntry& entry = _errorLog[i];

            Serial.print("  [");
            Serial.print(entry.timestamp / 1000);
            Serial.print("s] ");
            Serial.print(severityToString(entry.severity));
            Serial.print(" - ");
            Serial.print(entry.subsystem);
            Serial.print(": ");
            Serial.println(entry.message);
        }
    }

    Serial.println("========================================");
    Serial.print("  Total Errors: ");
    Serial.println(_totalErrors);
    Serial.print("  Total Warnings: ");
    Serial.println(_totalWarnings);
    Serial.print("  Recoveries: ");
    Serial.print(_totalRecoveries);
    Serial.print(" / ");
    Serial.println(_totalRecoveries + _failedRecoveries);
    Serial.println("========================================");
    Serial.println();
}

String ErrorManager::exportLogAsJson() {
    StaticJsonDocument<2048> doc;
    JsonArray errors = doc.createNestedArray("errors");

    for (int i = 0; i < _logCount; i++) {
        const ErrorLogEntry& entry = _errorLog[i];

        JsonObject error = errors.createNestedObject();
        error["timestamp"] = entry.timestamp;
        error["code"] = (int)entry.code;
        error["severity"] = severityToString(entry.severity);
        error["message"] = entry.message;
        error["subsystem"] = entry.subsystem;
        error["recovered"] = entry.recovered;
    }

    doc["total_errors"] = _totalErrors;
    doc["total_warnings"] = _totalWarnings;
    doc["total_recoveries"] = _totalRecoveries;
    doc["failed_recoveries"] = _failedRecoveries;
    doc["health"] = getSystemHealth();

    String output;
    serializeJson(doc, output);
    return output;
}

// ============================================================================
// STATISTICS
// ============================================================================

float ErrorManager::getSystemHealth() const {
    // EDUCATIONAL: Calculate system health score
    // 100% = perfect health, 0% = critical issues

    int totalIssues = _totalErrors + (_totalWarnings / 2);  // Warnings count half

    if (totalIssues == 0) {
        return 100.0f;
    }

    // Deduct points for errors, give credit for recoveries
    float health = 100.0f - (totalIssues * 5.0f) + (_totalRecoveries * 2.0f);

    // Clamp to valid range
    if (health < 0.0f) health = 0.0f;
    if (health > 100.0f) health = 100.0f;

    return health;
}

// ============================================================================
// SYSTEM HEALTH CHECKS
// ============================================================================

bool ErrorManager::checkLowMemory() {
    uint32_t freeHeap = ESP.getFreeHeap();
    const uint32_t LOW_MEMORY_THRESHOLD = 10000;  // 10KB threshold

    if (freeHeap < LOW_MEMORY_THRESHOLD) {
        return true;
    }

    return false;
}

bool ErrorManager::checkWiFiHealth() {
    if (WiFi.status() != WL_CONNECTED) {
        return false;
    }

    // Check signal strength
    int rssi = WiFi.RSSI();
    if (rssi < -80) {  // Weak signal
        reportWarning("Weak Wi-Fi signal: " + String(rssi) + " dBm", "WiFi");
    }

    return true;
}

void ErrorManager::performSystemHealthCheck() {
    Serial.println("[ErrorManager] Performing system health check...");

    // Check memory
    if (checkLowMemory()) {
        reportWarning("Low free memory", "System");
    }

    // Check WiFi
    if (!checkWiFiHealth()) {
        reportWarning("Wi-Fi not connected", "WiFi");
    }

    // Print summary
    Serial.print("[ErrorManager] System health: ");
    Serial.print(getSystemHealth(), 1);
    Serial.println("%");
}

// ============================================================================
// HELPER METHODS
// ============================================================================

String ErrorManager::errorCodeToString(ErrorCode code) {
    switch (code) {
        case ErrorCode::NONE: return "NONE";

        case ErrorCode::WIFI_CONNECTION_FAILED: return "WIFI_CONNECTION_FAILED";
        case ErrorCode::WIFI_DISCONNECTED: return "WIFI_DISCONNECTED";
        case ErrorCode::WIFI_WEAK_SIGNAL: return "WIFI_WEAK_SIGNAL";
        case ErrorCode::WIFI_TIMEOUT: return "WIFI_TIMEOUT";

        case ErrorCode::I2C_SENSOR_NOT_FOUND: return "I2C_SENSOR_NOT_FOUND";
        case ErrorCode::I2C_COMM_ERROR: return "I2C_COMM_ERROR";
        case ErrorCode::SPI_COMM_ERROR: return "SPI_COMM_ERROR";
        case ErrorCode::SENSOR_DATA_INVALID: return "SENSOR_DATA_INVALID";
        case ErrorCode::SENSOR_TIMEOUT: return "SENSOR_TIMEOUT";

        case ErrorCode::SERVO_ATTACH_FAILED: return "SERVO_ATTACH_FAILED";
        case ErrorCode::MOTOR_DRIVER_ERROR: return "MOTOR_DRIVER_ERROR";
        case ErrorCode::PWM_CONFIG_ERROR: return "PWM_CONFIG_ERROR";

        case ErrorCode::LOW_MEMORY: return "LOW_MEMORY";
        case ErrorCode::WATCHDOG_TIMEOUT: return "WATCHDOG_TIMEOUT";
        case ErrorCode::POWER_LOW_BATTERY: return "POWER_LOW_BATTERY";
        case ErrorCode::TEMPERATURE_HIGH: return "TEMPERATURE_HIGH";

        case ErrorCode::INVALID_COMMAND: return "INVALID_COMMAND";
        case ErrorCode::COMMAND_PARSE_ERROR: return "COMMAND_PARSE_ERROR";
        case ErrorCode::COMMAND_EXECUTION_FAILED: return "COMMAND_EXECUTION_FAILED";

        case ErrorCode::UNKNOWN_ERROR: return "UNKNOWN_ERROR";
        case ErrorCode::INITIALIZATION_FAILED: return "INITIALIZATION_FAILED";

        default: return "UNKNOWN";
    }
}

String ErrorManager::severityToString(ErrorSeverity severity) {
    switch (severity) {
        case ErrorSeverity::INFO: return "INFO";
        case ErrorSeverity::WARNING: return "WARNING";
        case ErrorSeverity::ERROR: return "ERROR";
        case ErrorSeverity::CRITICAL: return "CRITICAL";
        default: return "UNKNOWN";
    }
}
