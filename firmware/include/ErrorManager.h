/**
 * @file ErrorManager.h
 * @brief Error detection, logging, and recovery management
 *
 * ESP32 Robotics Firmware - Phase 4: Error Management
 *
 * EDUCATIONAL NOTE: Robust embedded systems need comprehensive error handling:
 * 1. Detection: Identify when something goes wrong
 * 2. Logging: Record errors for debugging
 * 3. Recovery: Attempt to fix or work around the problem
 * 4. Reporting: Inform the user/UI about errors
 *
 * This ErrorManager provides a centralized system for managing all error
 * conditions in the robot firmware, with automatic recovery strategies.
 */

#ifndef ERROR_MANAGER_H
#define ERROR_MANAGER_H

#include <Arduino.h>
#include "RobotSubsystem.h"

/**
 * @brief Maximum number of error log entries to store
 */
#define MAX_ERROR_LOG_ENTRIES 20

/**
 * @brief Error severity levels
 */
enum class ErrorSeverity {
    INFO,       // Informational, not a problem
    WARNING,    // Minor issue, system continues
    ERROR,      // Significant issue, degraded operation
    CRITICAL    // Major failure, recovery required
};

/**
 * @brief Error codes for different failure types
 */
enum class ErrorCode {
    NONE = 0,

    // Wi-Fi errors (100-199)
    WIFI_CONNECTION_FAILED = 100,
    WIFI_DISCONNECTED = 101,
    WIFI_WEAK_SIGNAL = 102,
    WIFI_TIMEOUT = 103,

    // Sensor errors (200-299)
    I2C_SENSOR_NOT_FOUND = 200,
    I2C_COMM_ERROR = 201,
    SPI_COMM_ERROR = 202,
    SENSOR_DATA_INVALID = 203,
    SENSOR_TIMEOUT = 204,

    // Actuator errors (300-399)
    SERVO_ATTACH_FAILED = 300,
    MOTOR_DRIVER_ERROR = 301,
    PWM_CONFIG_ERROR = 302,

    // System errors (400-499)
    LOW_MEMORY = 400,
    WATCHDOG_TIMEOUT = 401,
    POWER_LOW_BATTERY = 402,
    TEMPERATURE_HIGH = 403,

    // Command errors (500-599)
    INVALID_COMMAND = 500,
    COMMAND_PARSE_ERROR = 501,
    COMMAND_EXECUTION_FAILED = 502,

    // Generic errors (600+)
    UNKNOWN_ERROR = 600,
    INITIALIZATION_FAILED = 601
};

/**
 * @brief Error log entry structure
 */
struct ErrorLogEntry {
    unsigned long timestamp;     // When the error occurred (millis)
    ErrorCode code;             // Error code
    ErrorSeverity severity;     // Severity level
    String message;             // Descriptive message
    String subsystem;           // Which subsystem reported it
    bool recovered;             // Was recovery attempted/successful
};

/**
 * @brief Recovery action to take for an error
 */
enum class RecoveryAction {
    NONE,                // No action needed
    RETRY,               // Retry the failed operation
    RESET_SUBSYSTEM,     // Reset the affected subsystem
    DISABLE_SUBSYSTEM,   // Disable the subsystem
    SYSTEM_RESTART       // Restart entire system
};

/**
 * @brief ErrorManager class for comprehensive error handling
 *
 * EDUCATIONAL: This demonstrates professional error management in
 * embedded systems. It provides centralized error tracking, automatic
 * recovery, and detailed logging for debugging.
 */
class ErrorManager : public RobotSubsystem {
private:
    ErrorLogEntry _errorLog[MAX_ERROR_LOG_ENTRIES];
    int _logIndex;
    int _logCount;

    ErrorCode _currentError;
    ErrorSeverity _currentSeverity;
    String _currentMessage;

    int _totalErrors;
    int _totalWarnings;
    int _totalRecoveries;
    int _failedRecoveries;

    unsigned long _lastErrorTime;
    const unsigned long ERROR_DEBOUNCE_TIME = 1000;  // 1 second

    // Recovery configuration
    bool _autoRecoveryEnabled;
    int _maxRetries;

    // Helper methods
    void addToLog(ErrorCode code, ErrorSeverity severity, const String& message, const String& subsystem);
    RecoveryAction getRecoveryAction(ErrorCode code);
    bool attemptRecovery(ErrorCode code, const String& subsystem);
    String errorCodeToString(ErrorCode code);
    String severityToString(ErrorSeverity severity);

public:
    /**
     * @brief Constructor
     */
    ErrorManager();

    /**
     * @brief Initialize error management system
     * @return true if successful
     */
    bool initialize() override;

    /**
     * @brief Update error manager (check for system errors)
     */
    void update() override;

    /**
     * @brief Check if error manager is ready
     * @return true if initialized
     */
    bool isReady() override;

    /**
     * @brief Get current status string
     * @return Status description
     */
    String getStatus() override;

    // ========================================================================
    // ERROR REPORTING
    // ========================================================================

    /**
     * @brief Report an error
     * @param code Error code
     * @param severity Severity level
     * @param message Descriptive message
     * @param subsystem Name of subsystem reporting error
     *
     * EDUCATIONAL: This is the main method for reporting errors.
     * Call this whenever something goes wrong in your subsystem.
     */
    void reportError(
        ErrorCode code,
        ErrorSeverity severity,
        const String& message,
        const String& subsystem = "System"
    );

    /**
     * @brief Report informational message
     * @param message Info message
     * @param subsystem Reporting subsystem
     */
    void reportInfo(const String& message, const String& subsystem = "System");

    /**
     * @brief Report warning
     * @param message Warning message
     * @param subsystem Reporting subsystem
     */
    void reportWarning(const String& message, const String& subsystem = "System");

    /**
     * @brief Clear current error state
     */
    void clearError();

    /**
     * @brief Clear all error history
     */
    void clearAllErrors();

    // ========================================================================
    // ERROR STATE QUERIES
    // ========================================================================

    /**
     * @brief Check if system has active errors
     * @return true if there are unresolved errors
     */
    bool hasError() const { return _currentError != ErrorCode::NONE; }

    /**
     * @brief Get current error code
     * @return Active error code
     */
    ErrorCode getCurrentError() const { return _currentError; }

    /**
     * @brief Get current error severity
     * @return Severity of active error
     */
    ErrorSeverity getCurrentSeverity() const { return _currentSeverity; }

    /**
     * @brief Get current error message
     * @return Error description
     */
    String getCurrentMessage() const { return _currentMessage; }

    /**
     * @brief Check if error is critical
     * @return true if current error is CRITICAL
     */
    bool isCriticalError() const { return _currentSeverity == ErrorSeverity::CRITICAL; }

    // ========================================================================
    // ERROR RECOVERY
    // ========================================================================

    /**
     * @brief Enable/disable automatic recovery
     * @param enable true to enable auto recovery
     *
     * EDUCATIONAL: When enabled, the error manager will automatically
     * attempt to recover from errors using predefined strategies.
     */
    void setAutoRecovery(bool enable) { _autoRecoveryEnabled = enable; }

    /**
     * @brief Set maximum retry attempts
     * @param maxRetries Number of retries before giving up
     */
    void setMaxRetries(int maxRetries) { _maxRetries = maxRetries; }

    /**
     * @brief Manually trigger recovery for current error
     * @return true if recovery successful
     */
    bool recoverFromError();

    // ========================================================================
    // ERROR LOG ACCESS
    // ========================================================================

    /**
     * @brief Get number of log entries
     * @return Number of errors logged
     */
    int getLogCount() const { return _logCount; }

    /**
     * @brief Get specific log entry
     * @param index Log entry index (0 = oldest)
     * @return Log entry reference
     */
    const ErrorLogEntry& getLogEntry(int index) const;

    /**
     * @brief Get most recent error
     * @return Most recent log entry
     */
    const ErrorLogEntry& getMostRecentError() const;

    /**
     * @brief Print error log to Serial
     *
     * EDUCATIONAL: Prints formatted error history to help with debugging.
     * Shows timestamp, severity, code, and message for each error.
     */
    void printErrorLog();

    /**
     * @brief Export error log as JSON string
     * @return JSON formatted error log
     *
     * EDUCATIONAL: This allows sending error history to the Streamlit UI
     * for remote debugging and monitoring.
     */
    String exportLogAsJson();

    // ========================================================================
    // STATISTICS
    // ========================================================================

    /**
     * @brief Get total error count
     * @return Total errors reported
     */
    int getTotalErrors() const { return _totalErrors; }

    /**
     * @brief Get total warning count
     * @return Total warnings reported
     */
    int getTotalWarnings() const { return _totalWarnings; }

    /**
     * @brief Get successful recovery count
     * @return Number of successful recoveries
     */
    int getTotalRecoveries() const { return _totalRecoveries; }

    /**
     * @brief Get failed recovery count
     * @return Number of failed recovery attempts
     */
    int getFailedRecoveries() const { return _failedRecoveries; }

    /**
     * @brief Get system health percentage
     * @return Health score (0-100)
     *
     * EDUCATIONAL: Calculates overall system health based on error rates.
     * 100% = no errors, lower values indicate more problems.
     */
    float getSystemHealth() const;

    // ========================================================================
    // SYSTEM CHECKS
    // ========================================================================

    /**
     * @brief Check for low memory condition
     * @return true if memory is low
     */
    bool checkLowMemory();

    /**
     * @brief Check Wi-Fi health
     * @return true if Wi-Fi is healthy
     */
    bool checkWiFiHealth();

    /**
     * @brief Perform comprehensive system health check
     *
     * EDUCATIONAL: Runs through all subsystem health checks and
     * reports any detected issues.
     */
    void performSystemHealthCheck();
};

#endif // ERROR_MANAGER_H
