/**
 * @file TelemetryData.h
 * @brief Telemetry data structure and JSON serialization
 *
 * EDUCATIONAL NOTE: Telemetry is the automated collection and transmission
 * of data from remote sources. In robotics, telemetry provides:
 * - Real-time monitoring of robot state
 * - Historical data logging
 * - Performance analysis
 * - Remote debugging
 * - User interface updates
 *
 * This file defines the data structure that captures the complete state
 * of our robot system at any moment in time. The structure can be:
 * 1. Populated by reading from all subsystems
 * 2. Serialized to JSON for network transmission
 * 3. Logged to SD card or database
 * 4. Displayed in user interface
 *
 * JSON Format Choice:
 * - Human-readable (great for debugging)
 * - Widely supported (every platform can parse JSON)
 * - Self-describing (field names included)
 * - Compact enough for embedded systems
 * - Alternative formats: Protocol Buffers (smaller), MessagePack (binary)
 */

#ifndef TELEMETRY_DATA_H
#define TELEMETRY_DATA_H

#include <Arduino.h>
#include <ArduinoJson.h>

/**
 * @struct TelemetryData
 * @brief Complete snapshot of robot system state
 *
 * EDUCATIONAL: This struct aggregates data from all subsystems into a
 * single cohesive package. Benefits:
 * - Atomic snapshot (all data from same moment)
 * - Easy to serialize/deserialize
 * - Clear documentation of system state
 * - Type safety (vs string parsing)
 */
struct TelemetryData {
    // ========================================================================
    // TIMESTAMP AND SYSTEM INFO
    // ========================================================================

    /**
     * @brief System uptime in milliseconds
     *
     * EDUCATIONAL: millis() returns time since boot. Wraps around after
     * ~49 days. For longer missions, consider using RTC (Real-Time Clock).
     */
    unsigned long timestamp;

    /**
     * @brief Free heap memory in bytes
     *
     * EDUCATIONAL: Monitoring free heap helps detect memory leaks.
     * If this value continuously decreases, you have a leak!
     */
    uint32_t freeHeap;

    /**
     * @brief Current operational mode
     *
     * EDUCATIONAL: Robots often have multiple modes:
     * - "idle": Powered on, waiting for commands
     * - "active": Executing tasks
     * - "low_power": Energy conservation mode
     * - "error": Fault condition
     */
    String operationalMode;

    // ========================================================================
    // SERVO ARM DATA
    // ========================================================================

    /**
     * @brief Current servo angle in degrees (0-180)
     */
    float servoCurrentAngle;

    /**
     * @brief Target servo angle in degrees
     */
    float servoTargetAngle;

    /**
     * @brief Servo enabled state
     */
    bool servoEnabled;

    /**
     * @brief Servo has reached target position
     */
    bool servoAtTarget;

    // ========================================================================
    // ENVIRONMENTAL SENSOR DATA (I2C BME280)
    // ========================================================================

    /**
     * @brief Temperature in degrees Celsius
     *
     * EDUCATIONAL: Temperature affects:
     * - Battery performance (cold = less capacity)
     * - Servo torque (extreme temps = reduced performance)
     * - Electronics reliability
     */
    float temperature;

    /**
     * @brief Relative humidity in percent (0-100%)
     *
     * EDUCATIONAL: High humidity can cause:
     * - Condensation on electronics
     * - Corrosion of contacts
     * - Reduced battery life
     */
    float humidity;

    /**
     * @brief Atmospheric pressure in hectopascals (hPa)
     *
     * EDUCATIONAL: Pressure varies with altitude and weather.
     * Can be used for:
     * - Altitude estimation
     * - Weather prediction
     * - Vertical velocity (rate of pressure change)
     */
    float pressure;

    /**
     * @brief Estimated altitude in meters
     *
     * EDUCATIONAL: Calculated from pressure using barometric formula.
     * Requires calibration with known sea-level pressure.
     */
    float altitude;

    /**
     * @brief Number of successful sensor reads
     */
    unsigned long sensorReadCount;

    /**
     * @brief Number of sensor read errors
     */
    unsigned long sensorErrorCount;

    // ========================================================================
    // MOTOR CONTROL DATA (LOCOMOTION)
    // ========================================================================

    /**
     * @brief Motor A speed (0-255)
     */
    uint8_t motorASpeed;

    /**
     * @brief Motor B speed (0-255)
     */
    uint8_t motorBSpeed;

    /**
     * @brief Motor A direction (true=forward, false=reverse)
     */
    bool motorADirection;

    /**
     * @brief Motor B direction
     */
    bool motorBDirection;

    /**
     * @brief Motors enabled state
     */
    bool motorsEnabled;

    // ========================================================================
    // SPI DEVICE DATA
    // ========================================================================

    /**
     * @brief Number of SPI transfers performed
     */
    unsigned long spiTransferCount;

    /**
     * @brief Last test register value from SPI device
     */
    uint8_t spiTestValue;

    // ========================================================================
    // SYSTEM STATUS
    // ========================================================================

    /**
     * @brief Error code (0 = no error)
     *
     * EDUCATIONAL: Error codes provide quick diagnosis:
     * 0 = No error
     * 1 = Wi-Fi connection failed
     * 2 = Sensor initialization failed
     * 3 = Servo failure
     * 4 = Low battery
     * 5 = I2C communication error
     * ... (defined in config.h or separate header)
     */
    int errorCode;

    /**
     * @brief Human-readable error message
     */
    String errorMessage;

    /**
     * @brief Number of subsystems ready
     */
    int subsystemsReady;

    /**
     * @brief Total number of subsystems
     */
    int subsystemsTotal;

    // ========================================================================
    // JSON SERIALIZATION
    // ========================================================================

    /**
     * @brief Convert telemetry data to JSON string
     *
     * EDUCATIONAL: ArduinoJson library provides efficient JSON creation.
     * StaticJsonDocument allocates on stack (fast, predictable).
     * Size must be large enough for all data (~512-1024 bytes typical).
     *
     * To calculate required size: https://arduinojson.org/v6/assistant/
     *
     * @return JSON string representation of telemetry
     */
    String toJson() const {
        // EDUCATIONAL: StaticJsonDocument size must fit all data.
        // If too small, serialization fails. If too large, wastes stack memory.
        // 1024 bytes is generous for this data set.
        StaticJsonDocument<1024> doc;

        // System info
        doc["timestamp"] = timestamp;
        doc["free_heap"] = freeHeap;
        doc["mode"] = operationalMode;

        // Servo data
        JsonObject servo = doc.createNestedObject("servo");
        servo["current_angle"] = servoCurrentAngle;
        servo["target_angle"] = servoTargetAngle;
        servo["enabled"] = servoEnabled;
        servo["at_target"] = servoAtTarget;

        // Environmental sensor data
        JsonObject environment = doc.createNestedObject("environment");
        environment["temperature"] = temperature;
        environment["humidity"] = humidity;
        environment["pressure"] = pressure;
        environment["altitude"] = altitude;
        environment["read_count"] = sensorReadCount;
        environment["error_count"] = sensorErrorCount;

        // Motor data
        JsonObject motors = doc.createNestedObject("motors");
        motors["motor_a_speed"] = motorASpeed;
        motors["motor_b_speed"] = motorBSpeed;
        motors["motor_a_direction"] = motorADirection ? "forward" : "reverse";
        motors["motor_b_direction"] = motorBDirection ? "forward" : "reverse";
        motors["enabled"] = motorsEnabled;

        // SPI data
        JsonObject spi = doc.createNestedObject("spi");
        spi["transfer_count"] = spiTransferCount;
        spi["test_value"] = spiTestValue;

        // System status
        JsonObject status = doc.createNestedObject("status");
        status["error_code"] = errorCode;
        status["error_message"] = errorMessage;
        status["subsystems_ready"] = subsystemsReady;
        status["subsystems_total"] = subsystemsTotal;

        // Serialize to string
        String output;
        serializeJson(doc, output);
        return output;
    }

    /**
     * @brief Convert telemetry to pretty-printed JSON
     *
     * EDUCATIONAL: Pretty printing adds whitespace for human readability.
     * Use for debugging/logging, not for network transmission (wastes bandwidth).
     *
     * @return Formatted JSON string
     */
    String toPrettyJson() const {
        StaticJsonDocument<1024> doc;

        // Same as toJson() but with pretty printing
        doc["timestamp"] = timestamp;
        doc["free_heap"] = freeHeap;
        doc["mode"] = operationalMode;

        JsonObject servo = doc.createNestedObject("servo");
        servo["current_angle"] = servoCurrentAngle;
        servo["target_angle"] = servoTargetAngle;
        servo["enabled"] = servoEnabled;
        servo["at_target"] = servoAtTarget;

        JsonObject environment = doc.createNestedObject("environment");
        environment["temperature"] = temperature;
        environment["humidity"] = humidity;
        environment["pressure"] = pressure;
        environment["altitude"] = altitude;
        environment["read_count"] = sensorReadCount;
        environment["error_count"] = sensorErrorCount;

        JsonObject motors = doc.createNestedObject("motors");
        motors["motor_a_speed"] = motorASpeed;
        motors["motor_b_speed"] = motorBSpeed;
        motors["motor_a_direction"] = motorADirection ? "forward" : "reverse";
        motors["motor_b_direction"] = motorBDirection ? "forward" : "reverse";
        motors["enabled"] = motorsEnabled;

        JsonObject spi = doc.createNestedObject("spi");
        spi["transfer_count"] = spiTransferCount;
        spi["test_value"] = spiTestValue;

        JsonObject status = doc.createNestedObject("status");
        status["error_code"] = errorCode;
        status["error_message"] = errorMessage;
        status["subsystems_ready"] = subsystemsReady;
        status["subsystems_total"] = subsystemsTotal;

        // Serialize with pretty printing (adds indentation/newlines)
        String output;
        serializeJsonPretty(doc, output);
        return output;
    }

    /**
     * @brief Parse JSON string into telemetry structure
     *
     * EDUCATIONAL: Deserialization converts JSON back to structured data.
     * Useful for:
     * - Receiving commands from UI
     * - Loading saved configurations
     * - Replaying logged telemetry
     *
     * @param jsonString JSON string to parse
     * @return true if parsing successful, false otherwise
     */
    bool fromJson(const String& jsonString) {
        StaticJsonDocument<1024> doc;

        // Attempt to parse JSON
        DeserializationError error = deserializeJson(doc, jsonString);

        if (error) {
            Serial.print("[TelemetryData] JSON parsing failed: ");
            Serial.println(error.c_str());
            return false;
        }

        // Extract values (with defaults if field missing)
        timestamp = doc["timestamp"] | 0;
        freeHeap = doc["free_heap"] | 0;
        operationalMode = doc["mode"] | String("unknown");

        // Servo data
        servoCurrentAngle = doc["servo"]["current_angle"] | 0.0f;
        servoTargetAngle = doc["servo"]["target_angle"] | 0.0f;
        servoEnabled = doc["servo"]["enabled"] | false;
        servoAtTarget = doc["servo"]["at_target"] | false;

        // Environmental data
        temperature = doc["environment"]["temperature"] | 0.0f;
        humidity = doc["environment"]["humidity"] | 0.0f;
        pressure = doc["environment"]["pressure"] | 0.0f;
        altitude = doc["environment"]["altitude"] | 0.0f;
        sensorReadCount = doc["environment"]["read_count"] | 0;
        sensorErrorCount = doc["environment"]["error_count"] | 0;

        // Motor data
        motorASpeed = doc["motors"]["motor_a_speed"] | 0;
        motorBSpeed = doc["motors"]["motor_b_speed"] | 0;
        String dirA = doc["motors"]["motor_a_direction"] | "forward";
        String dirB = doc["motors"]["motor_b_direction"] | "forward";
        motorADirection = (dirA == "forward");
        motorBDirection = (dirB == "forward");
        motorsEnabled = doc["motors"]["enabled"] | false;

        // SPI data
        spiTransferCount = doc["spi"]["transfer_count"] | 0;
        spiTestValue = doc["spi"]["test_value"] | 0;

        // Status
        errorCode = doc["status"]["error_code"] | 0;
        errorMessage = doc["status"]["error_message"] | String("");
        subsystemsReady = doc["status"]["subsystems_ready"] | 0;
        subsystemsTotal = doc["status"]["subsystems_total"] | 0;

        return true;
    }

    /**
     * @brief Print telemetry to Serial in readable format
     *
     * EDUCATIONAL: Useful for debugging - see all values at a glance.
     */
    void printToSerial() const {
        Serial.println("========================================");
        Serial.println("  TELEMETRY DATA");
        Serial.println("========================================");

        Serial.print("Timestamp: ");
        Serial.print(timestamp);
        Serial.println(" ms");

        Serial.print("Free Heap: ");
        Serial.print(freeHeap);
        Serial.println(" bytes");

        Serial.print("Mode: ");
        Serial.println(operationalMode);

        Serial.println();
        Serial.println("SERVO:");
        Serial.print("  Current: ");
        Serial.print(servoCurrentAngle);
        Serial.println("°");
        Serial.print("  Target: ");
        Serial.print(servoTargetAngle);
        Serial.println("°");
        Serial.print("  Enabled: ");
        Serial.println(servoEnabled ? "Yes" : "No");
        Serial.print("  At Target: ");
        Serial.println(servoAtTarget ? "Yes" : "No");

        Serial.println();
        Serial.println("ENVIRONMENT:");
        Serial.print("  Temperature: ");
        Serial.print(temperature);
        Serial.println(" °C");
        Serial.print("  Humidity: ");
        Serial.print(humidity);
        Serial.println(" %");
        Serial.print("  Pressure: ");
        Serial.print(pressure);
        Serial.println(" hPa");
        Serial.print("  Altitude: ");
        Serial.print(altitude);
        Serial.println(" m");
        Serial.print("  Reads: ");
        Serial.print(sensorReadCount);
        Serial.print(" (errors: ");
        Serial.print(sensorErrorCount);
        Serial.println(")");

        Serial.println();
        Serial.println("MOTORS:");
        Serial.print("  Motor A: ");
        Serial.print(motorASpeed);
        Serial.print(" ");
        Serial.println(motorADirection ? "FWD" : "REV");
        Serial.print("  Motor B: ");
        Serial.print(motorBSpeed);
        Serial.print(" ");
        Serial.println(motorBDirection ? "FWD" : "REV");
        Serial.print("  Enabled: ");
        Serial.println(motorsEnabled ? "Yes" : "No");

        Serial.println();
        Serial.println("SPI:");
        Serial.print("  Transfers: ");
        Serial.println(spiTransferCount);
        Serial.print("  Test Value: 0x");
        Serial.println(spiTestValue, HEX);

        Serial.println();
        Serial.println("STATUS:");
        Serial.print("  Error Code: ");
        Serial.println(errorCode);
        Serial.print("  Error Message: ");
        Serial.println(errorMessage);
        Serial.print("  Subsystems: ");
        Serial.print(subsystemsReady);
        Serial.print("/");
        Serial.println(subsystemsTotal);

        Serial.println("========================================");
    }
};

#endif // TELEMETRY_DATA_H
