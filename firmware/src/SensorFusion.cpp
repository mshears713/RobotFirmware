/**
 * @file SensorFusion.cpp
 * @brief Implementation of SensorFusion class
 *
 * ESP32 Robotics Firmware - Phase 4: Sensor Fusion
 */

#include "SensorFusion.h"

// ============================================================================
// CONSTRUCTOR
// ============================================================================

SensorFusion::SensorFusion()
    : _weight1(0.5f),
      _weight2(0.5f),
      _fusedValue(0.0f),
      _alpha(0.98f),
      _complementaryOutput(0.0f),
      _sampleIndex(0),
      _sampleCount(0),
      _emaValue(0.0f),
      _emaAlpha(0.2f),
      _sensor1LastUpdate(0),
      _sensor2LastUpdate(0),
      _sensor1ErrorCount(0),
      _sensor2ErrorCount(0),
      _minValue(999999.0f),
      _maxValue(-999999.0f),
      _averageValue(0.0f),
      _totalSamples(0)
{
    // Initialize sample buffer
    for (int i = 0; i < MAX_FUSION_SAMPLES; i++) {
        _samples[i] = 0.0f;
    }
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool SensorFusion::initialize() {
    Serial.println("[SensorFusion] Initializing sensor fusion...");

    resetStatistics();

    _isInitialized = true;
    Serial.println("[SensorFusion] ✓ Sensor fusion ready");
    Serial.println("[SensorFusion]   Methods: Weighted Average, Complementary, Median, EMA");

    return true;
}

// ============================================================================
// UPDATE
// ============================================================================

void SensorFusion::update() {
    if (!_isInitialized) return;

    // EDUCATIONAL: This is called periodically to check sensor health
    // and timeout stale sensors

    // Check for sensor timeouts
    unsigned long now = millis();

    if (now - _sensor1LastUpdate > SENSOR_TIMEOUT && _sensor1LastUpdate > 0) {
        Serial.println("[SensorFusion] ⚠ Sensor 1 timeout");
        _sensor1ErrorCount++;
    }

    if (now - _sensor2LastUpdate > SENSOR_TIMEOUT && _sensor2LastUpdate > 0) {
        Serial.println("[SensorFusion] ⚠ Sensor 2 timeout");
        _sensor2ErrorCount++;
    }

    _lastUpdate = millis();
}

// ============================================================================
// STATUS
// ============================================================================

bool SensorFusion::isReady() {
    return _isInitialized;
}

String SensorFusion::getStatus() {
    String status = "SensorFusion: ";
    status += String(_totalSamples);
    status += " samples, range [";
    status += String(_minValue, 1);
    status += ", ";
    status += String(_maxValue, 1);
    status += "]";
    return status;
}

// ============================================================================
// WEIGHTED AVERAGE FUSION
// ============================================================================

void SensorFusion::setWeights(float weight1, float weight2) {
    _weight1 = weight1;
    _weight2 = weight2;

    Serial.print("[SensorFusion] Weights set: ");
    Serial.print(weight1, 2);
    Serial.print(", ");
    Serial.println(weight2, 2);
}

float SensorFusion::fuseWeightedAverage(float sensor1Value, float sensor2Value) {
    // EDUCATIONAL: Weighted average formula
    // Normalizes weights automatically so they don't need to sum to 1.0

    float totalWeight = _weight1 + _weight2;

    if (totalWeight == 0.0f) {
        // Avoid division by zero
        _fusedValue = (sensor1Value + sensor2Value) / 2.0f;
    } else {
        _fusedValue = (_weight1 * sensor1Value + _weight2 * sensor2Value) / totalWeight;
    }

    updateStatistics(_fusedValue);
    return _fusedValue;
}

// ============================================================================
// COMPLEMENTARY FILTER FUSION
// ============================================================================

void SensorFusion::setComplementaryAlpha(float alpha) {
    // Clamp alpha to valid range [0.0, 1.0]
    _alpha = constrain(alpha, 0.0f, 1.0f);

    Serial.print("[SensorFusion] Complementary alpha: ");
    Serial.println(_alpha, 3);
}

float SensorFusion::fuseComplementary(float fastSensor, float slowSensor, float dt) {
    // EDUCATIONAL: Complementary filter is perfect for IMU fusion
    // Formula: output = alpha * (prev + gyro*dt) + (1-alpha) * accel
    //
    // The fast sensor (e.g., gyroscope) provides high-frequency changes
    // The slow sensor (e.g., accelerometer) provides long-term stability
    //
    // Alpha near 1.0 trusts the fast sensor for quick changes
    // (1-alpha) near 1.0 trusts the slow sensor for stable reference

    _complementaryOutput = _alpha * (_complementaryOutput + fastSensor * dt) +
                          (1.0f - _alpha) * slowSensor;

    updateStatistics(_complementaryOutput);
    return _complementaryOutput;
}

// ============================================================================
// MEDIAN FILTER
// ============================================================================

void SensorFusion::addSample(float value) {
    // EDUCATIONAL: Circular buffer for median filter
    // Stores last N samples, overwrites oldest when full

    _samples[_sampleIndex] = value;
    _sampleIndex = (_sampleIndex + 1) % MAX_FUSION_SAMPLES;

    if (_sampleCount < MAX_FUSION_SAMPLES) {
        _sampleCount++;
    }
}

float SensorFusion::getMedian() {
    if (_sampleCount == 0) {
        return 0.0f;
    }

    // EDUCATIONAL: Create a temporary copy for sorting
    // We don't want to modify the original sample buffer
    float sortedSamples[MAX_FUSION_SAMPLES];
    for (int i = 0; i < _sampleCount; i++) {
        sortedSamples[i] = _samples[i];
    }

    return calculateMedian(sortedSamples, _sampleCount);
}

float SensorFusion::calculateMedian(float* values, int count) {
    // EDUCATIONAL: Simple bubble sort for small arrays
    // For larger arrays, consider quicksort or partial sort
    for (int i = 0; i < count - 1; i++) {
        for (int j = 0; j < count - i - 1; j++) {
            if (values[j] > values[j + 1]) {
                float temp = values[j];
                values[j] = values[j + 1];
                values[j + 1] = temp;
            }
        }
    }

    // Return middle value (or average of two middle values for even count)
    if (count % 2 == 0) {
        return (values[count / 2 - 1] + values[count / 2]) / 2.0f;
    } else {
        return values[count / 2];
    }
}

void SensorFusion::clearSamples() {
    _sampleIndex = 0;
    _sampleCount = 0;

    for (int i = 0; i < MAX_FUSION_SAMPLES; i++) {
        _samples[i] = 0.0f;
    }

    Serial.println("[SensorFusion] Sample buffer cleared");
}

// ============================================================================
// EXPONENTIAL MOVING AVERAGE
// ============================================================================

void SensorFusion::setEmaAlpha(float alpha) {
    // Clamp alpha to valid range [0.0, 1.0]
    _emaAlpha = constrain(alpha, 0.0f, 1.0f);

    Serial.print("[SensorFusion] EMA alpha: ");
    Serial.println(_emaAlpha, 3);
}

float SensorFusion::updateEma(float newValue) {
    // EDUCATIONAL: Exponential Moving Average formula
    // EMA = alpha * newValue + (1 - alpha) * previousEMA
    //
    // This is a first-order low-pass filter:
    // - Higher alpha = faster response, less filtering
    // - Lower alpha = slower response, more filtering
    //
    // It's computationally efficient and uses minimal memory

    _emaValue = _emaAlpha * newValue + (1.0f - _emaAlpha) * _emaValue;

    updateStatistics(_emaValue);
    return _emaValue;
}

void SensorFusion::resetEma(float initialValue) {
    _emaValue = initialValue;
    Serial.print("[SensorFusion] EMA reset to: ");
    Serial.println(initialValue);
}

// ============================================================================
// SENSOR HEALTH MONITORING
// ============================================================================

void SensorFusion::reportSensorUpdate(int sensor, bool error) {
    unsigned long now = millis();

    if (sensor == 1) {
        _sensor1LastUpdate = now;
        if (error) {
            _sensor1ErrorCount++;
        }
    } else if (sensor == 2) {
        _sensor2LastUpdate = now;
        if (error) {
            _sensor2ErrorCount++;
        }
    }
}

bool SensorFusion::isSensorHealthy(int sensor) {
    if (sensor == 1) {
        return isSensorHealthy(_sensor1LastUpdate, _sensor1ErrorCount);
    } else if (sensor == 2) {
        return isSensorHealthy(_sensor2LastUpdate, _sensor2ErrorCount);
    }
    return false;
}

bool SensorFusion::isSensorHealthy(unsigned long lastUpdate, int errorCount) {
    // EDUCATIONAL: Sensor health criteria:
    // 1. Has updated recently (within timeout)
    // 2. Has low error rate (less than 10% failure)

    unsigned long now = millis();

    if (lastUpdate == 0) {
        return false;  // Never updated
    }

    if (now - lastUpdate > SENSOR_TIMEOUT) {
        return false;  // Timed out
    }

    // Consider unhealthy if more than 10 errors
    if (errorCount > 10) {
        return false;
    }

    return true;
}

int SensorFusion::getSensorErrorCount(int sensor) {
    if (sensor == 1) {
        return _sensor1ErrorCount;
    } else if (sensor == 2) {
        return _sensor2ErrorCount;
    }
    return 0;
}

// ============================================================================
// STATISTICS
// ============================================================================

void SensorFusion::updateStatistics(float value) {
    // Update min/max
    if (value < _minValue) {
        _minValue = value;
    }
    if (value > _maxValue) {
        _maxValue = value;
    }

    // Update running average
    // Formula: newAvg = (oldAvg * count + newValue) / (count + 1)
    _averageValue = (_averageValue * _totalSamples + value) / (_totalSamples + 1);

    _totalSamples++;
}

void SensorFusion::resetStatistics() {
    _minValue = 999999.0f;
    _maxValue = -999999.0f;
    _averageValue = 0.0f;
    _totalSamples = 0;

    _sensor1ErrorCount = 0;
    _sensor2ErrorCount = 0;

    Serial.println("[SensorFusion] Statistics reset");
}
