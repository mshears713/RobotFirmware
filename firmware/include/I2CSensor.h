/**
 * @file I2CSensor.h
 * @brief I2C sensor interface subsystem (BME280 Temperature/Humidity/Pressure)
 *
 * EDUCATIONAL NOTE: I2C (Inter-Integrated Circuit) is a two-wire serial
 * communication protocol widely used for sensors and low-speed peripherals.
 *
 * I2C Fundamentals:
 * - Two wires: SDA (data), SCL (clock)
 * - Multi-device bus: Up to 127 devices on same bus
 * - Master-slave architecture: ESP32 is master, sensors are slaves
 * - 7-bit addressing: Each device has unique address (e.g., 0x76 for BME280)
 * - Speeds: 100kHz (standard), 400kHz (fast mode)
 * - Requires pull-up resistors: 4.7kΩ on both SDA and SCL to 3.3V
 *
 * How I2C works:
 * 1. Master sends START condition
 * 2. Master sends device address + R/W bit
 * 3. Slave acknowledges (ACK)
 * 4. Master sends register address to read/write
 * 5. Data transfer occurs
 * 6. Master sends STOP condition
 *
 * This class interfaces with the Adafruit BME280 sensor library which
 * handles the low-level I2C protocol details.
 */

#ifndef I2C_SENSOR_H
#define I2C_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "RobotSubsystem.h"
#include "config.h"

/**
 * @class I2CSensor
 * @brief Manages BME280 environmental sensor via I2C
 *
 * EDUCATIONAL: This demonstrates I2C communication in embedded systems.
 * The BME280 measures:
 * - Temperature: -40°C to +85°C (±1°C accuracy)
 * - Humidity: 0-100% RH (±3% accuracy)
 * - Pressure: 300-1100 hPa (±1 hPa accuracy)
 *
 * All three readings come from a single sensor chip, making it ideal
 * for environmental monitoring in robotics projects.
 */
class I2CSensor : public RobotSubsystem {
public:
    /**
     * @brief Construct a new I2CSensor object
     *
     * EDUCATIONAL: The constructor sets configuration but doesn't initialize
     * hardware. I2C bus setup happens in initialize().
     *
     * @param address I2C device address (typically 0x76 or 0x77 for BME280)
     * @param readInterval How often to read sensor in milliseconds (default 1000ms)
     */
    I2CSensor(uint8_t address = BME280_I2C_ADDRESS,
              unsigned long readInterval = SENSOR_READ_INTERVAL)
        : _address(address),
          _readInterval(readInterval),
          _temperature(0.0f),
          _humidity(0.0f),
          _pressure(0.0f),
          _lastReadTime(0),
          _readCount(0),
          _errorCount(0)
    {
        // Constructor initializes member variables only
    }

    /**
     * @brief Initialize I2C bus and BME280 sensor
     *
     * EDUCATIONAL: This method:
     * 1. Initializes I2C bus with Wire.begin()
     * 2. Sets I2C clock speed
     * 3. Attempts to detect and initialize BME280
     * 4. Configures sensor sampling rates
     *
     * @return true if sensor initialized successfully, false otherwise
     */
    bool initialize() override {
        Serial.print("[I2CSensor] Initializing I2C bus on pins SDA=");
        Serial.print(I2C_SDA_PIN);
        Serial.print(", SCL=");
        Serial.println(I2C_SCL_PIN);

        // Initialize I2C bus
        // EDUCATIONAL: Wire.begin() configures the I2C pins and enables I2C hardware
        Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

        // Set I2C clock frequency
        // EDUCATIONAL: setClock() adjusts the SCL clock speed
        // 100kHz is standard mode, reliable for most sensors
        Wire.setClock(I2C_FREQUENCY);

        Serial.print("[I2CSensor] I2C clock set to ");
        Serial.print(I2C_FREQUENCY / 1000);
        Serial.println(" kHz");

        // Scan I2C bus to verify device presence
        if (!scanI2CAddress(_address)) {
            Serial.print("[I2CSensor] ✗ No device found at address 0x");
            Serial.println(_address, HEX);
            return false;
        }

        // Initialize BME280 sensor
        // EDUCATIONAL: The Adafruit library handles register configuration
        if (!_bme.begin(_address, &Wire)) {
            Serial.println("[I2CSensor] ✗ BME280 initialization FAILED");
            Serial.println("[I2CSensor] Troubleshooting:");
            Serial.println("  1. Check I2C wiring (SDA, SCL)");
            Serial.println("  2. Verify pull-up resistors (4.7kΩ to 3.3V)");
            Serial.println("  3. Confirm sensor address (0x76 or 0x77)");
            Serial.println("  4. Check power supply (3.3V)");
            return false;
        }

        // Configure sensor sampling
        // EDUCATIONAL: These settings balance accuracy vs power consumption
        _bme.setSampling(
            Adafruit_BME280::MODE_NORMAL,     // Continuous measurement mode
            Adafruit_BME280::SAMPLING_X16,    // Temperature oversampling x16
            Adafruit_BME280::SAMPLING_X16,    // Pressure oversampling x16
            Adafruit_BME280::SAMPLING_X16,    // Humidity oversampling x16
            Adafruit_BME280::FILTER_X16,      // IIR filter coefficient 16
            Adafruit_BME280::STANDBY_MS_500   // Standby time 500ms
        );

        // Take initial reading
        delay(100);  // Give sensor time to stabilize
        readSensorData();

        _isInitialized = true;
        Serial.println("[I2CSensor] ✓ BME280 initialized successfully");
        Serial.print("[I2CSensor] Initial readings: ");
        Serial.print(_temperature);
        Serial.print("°C, ");
        Serial.print(_humidity);
        Serial.print("% RH, ");
        Serial.print(_pressure);
        Serial.println(" hPa");

        return true;
    }

    /**
     * @brief Update sensor readings (called every loop iteration)
     *
     * EDUCATIONAL: This uses non-blocking timing to read the sensor
     * periodically without blocking the main loop. Sensor reads take
     * a few milliseconds, so we don't want to do them every loop iteration.
     */
    void update() override {
        if (!_isInitialized || !_isEnabled) {
            return;
        }

        // Check if it's time to read sensor
        if (hasIntervalElapsed(_readInterval)) {
            readSensorData();
            updateTimestamp();
        }
    }

    /**
     * @brief Check if sensor is ready
     *
     * @return true if initialized and communicating
     */
    bool isReady() override {
        return _isInitialized;
    }

    /**
     * @brief Get current status string
     *
     * @return Status with latest sensor readings
     */
    String getStatus() override {
        if (!_isInitialized) {
            return "I2CSensor: Not initialized";
        }

        char buffer[128];
        snprintf(buffer, sizeof(buffer),
                 "I2CSensor: %.1f°C, %.1f%% RH, %.1f hPa [reads=%lu, errors=%lu]",
                 _temperature, _humidity, _pressure, _readCount, _errorCount);
        return String(buffer);
    }

    /**
     * @brief Get subsystem name
     *
     * @return Subsystem identifier
     */
    const char* getName() const override {
        return "I2CSensor";
    }

    /**
     * @brief Get current temperature reading
     *
     * EDUCATIONAL: Temperature is in Celsius. To convert to Fahrenheit:
     * F = C * 9/5 + 32
     *
     * @return Temperature in degrees Celsius
     */
    float getTemperature() const {
        return _temperature;
    }

    /**
     * @brief Get current humidity reading
     *
     * @return Relative humidity in percent (0-100%)
     */
    float getHumidity() const {
        return _humidity;
    }

    /**
     * @brief Get current atmospheric pressure reading
     *
     * EDUCATIONAL: Pressure is in hectopascals (hPa), also called millibars (mbar).
     * Standard sea-level pressure is ~1013 hPa.
     * Higher altitude = lower pressure (roughly -12 hPa per 100m elevation)
     *
     * @return Atmospheric pressure in hPa
     */
    float getPressure() const {
        return _pressure;
    }

    /**
     * @brief Calculate altitude from pressure reading
     *
     * EDUCATIONAL: This uses the barometric formula to estimate altitude.
     * Requires knowing sea-level pressure (default 1013.25 hPa).
     * Accuracy: ±10-20 meters depending on weather conditions.
     *
     * @param seaLevelPressure Reference sea-level pressure in hPa (default 1013.25)
     * @return Estimated altitude in meters
     */
    float getAltitude(float seaLevelPressure = 1013.25f) const {
        if (!_isInitialized || _pressure == 0) {
            return 0.0f;
        }
        return _bme.readAltitude(seaLevelPressure);
    }

    /**
     * @brief Get number of successful sensor reads
     *
     * @return Read count
     */
    unsigned long getReadCount() const {
        return _readCount;
    }

    /**
     * @brief Get number of sensor read errors
     *
     * EDUCATIONAL: If error count is increasing, check:
     * - Loose I2C connections
     * - Missing pull-up resistors
     * - Power supply issues
     * - Electromagnetic interference
     *
     * @return Error count
     */
    unsigned long getErrorCount() const {
        return _errorCount;
    }

    /**
     * @brief Set sensor read interval
     *
     * @param interval Time between reads in milliseconds
     */
    void setReadInterval(unsigned long interval) {
        _readInterval = interval;
    }

    /**
     * @brief Force an immediate sensor read
     *
     * EDUCATIONAL: Use this sparingly. Prefer letting update() handle
     * periodic reads. This is useful for:
     * - Initial calibration
     * - User-triggered measurements
     * - Debugging
     */
    void forceRead() {
        if (_isInitialized) {
            readSensorData();
        }
    }

private:
    /**
     * @brief Read all sensor values from BME280
     *
     * EDUCATIONAL: The BME280 has internal registers containing measurement
     * data. The Adafruit library reads these registers via I2C and converts
     * the raw values to calibrated readings.
     */
    void readSensorData() {
        // EDUCATIONAL: Reading from I2C sensor can fail due to:
        // - Bus contention
        // - Electrical noise
        // - Timing issues
        // Always check if readings are valid

        float tempReading = _bme.readTemperature();
        float humReading = _bme.readHumidity();
        float pressReading = _bme.readPressure() / 100.0f;  // Convert Pa to hPa

        // Validate readings (BME280 returns valid ranges)
        // EDUCATIONAL: Sanity checking prevents bad data from propagating
        bool validTemp = (tempReading >= -40 && tempReading <= 85);
        bool validHum = (humReading >= 0 && humReading <= 100);
        bool validPress = (pressReading >= 300 && pressReading <= 1100);

        if (validTemp && validHum && validPress) {
            _temperature = tempReading;
            _humidity = humReading;
            _pressure = pressReading;
            _readCount++;

            if (DEBUG_VERBOSE) {
                Serial.print("[I2CSensor] Read #");
                Serial.print(_readCount);
                Serial.print(": ");
                Serial.print(_temperature);
                Serial.print("°C, ");
                Serial.print(_humidity);
                Serial.print("% RH, ");
                Serial.print(_pressure);
                Serial.println(" hPa");
            }
        } else {
            _errorCount++;
            Serial.print("[I2CSensor] ✗ Invalid reading (error #");
            Serial.print(_errorCount);
            Serial.println(")");

            if (DEBUG_VERBOSE) {
                Serial.print("  Temp: ");
                Serial.print(tempReading);
                Serial.print(" (valid: ");
                Serial.print(validTemp ? "yes" : "NO");
                Serial.println(")");
                Serial.print("  Humidity: ");
                Serial.print(humReading);
                Serial.print(" (valid: ");
                Serial.print(validHum ? "yes" : "NO");
                Serial.println(")");
                Serial.print("  Pressure: ");
                Serial.print(pressReading);
                Serial.print(" (valid: ");
                Serial.print(validPress ? "yes" : "NO");
                Serial.println(")");
            }
        }
    }

    /**
     * @brief Scan I2C bus for device at specific address
     *
     * EDUCATIONAL: I2C scanning sends a brief message to an address and
     * checks for acknowledgment (ACK). If device responds, it's present.
     * This is useful for debugging I2C connection issues.
     *
     * @param address I2C address to check
     * @return true if device responds at address
     */
    bool scanI2CAddress(uint8_t address) {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();

        // EDUCATIONAL: endTransmission() return values:
        // 0 = Success (device ACKed)
        // 1 = Data too long
        // 2 = NACK on address (device not found)
        // 3 = NACK on data
        // 4 = Other error

        if (error == 0) {
            Serial.print("[I2CSensor] ✓ Device found at address 0x");
            Serial.println(address, HEX);
            return true;
        } else {
            Serial.print("[I2CSensor] ✗ No response from address 0x");
            Serial.print(address, HEX);
            Serial.print(" (error code: ");
            Serial.print(error);
            Serial.println(")");
            return false;
        }
    }

    // Private member variables
    Adafruit_BME280 _bme;           ///< BME280 sensor object
    uint8_t _address;               ///< I2C device address
    unsigned long _readInterval;    ///< Time between sensor reads (ms)
    float _temperature;             ///< Last temperature reading (°C)
    float _humidity;                ///< Last humidity reading (% RH)
    float _pressure;                ///< Last pressure reading (hPa)
    unsigned long _lastReadTime;    ///< Timestamp of last read
    unsigned long _readCount;       ///< Number of successful reads
    unsigned long _errorCount;      ///< Number of failed reads
};

#endif // I2C_SENSOR_H
