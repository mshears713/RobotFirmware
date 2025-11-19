/**
 * @file SensorFusion.h
 * @brief Sensor fusion algorithms for combining multiple sensor readings
 *
 * ESP32 Robotics Firmware - Phase 4: Sensor Fusion
 *
 * EDUCATIONAL NOTE: Sensor fusion combines data from multiple sensors to produce
 * more accurate, reliable measurements than any single sensor could provide.
 *
 * Common fusion techniques:
 * 1. Weighted Average: Simple but effective for redundant sensors
 * 2. Kalman Filter: Optimal for noisy sensors with known characteristics
 * 3. Complementary Filter: Combines slow and fast sensors (like IMU)
 * 4. Median Filter: Removes outliers and spikes
 *
 * This implementation focuses on beginner-friendly weighted averaging and
 * complementary filtering suitable for embedded systems.
 */

#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <Arduino.h>
#include "RobotSubsystem.h"

/**
 * @brief Maximum number of sensor samples to store
 */
#define MAX_FUSION_SAMPLES 10

/**
 * @brief Fusion methods available
 */
enum class FusionMethod {
    WEIGHTED_AVERAGE,     // Simple weighted average of inputs
    COMPLEMENTARY,        // Complementary filter (high-pass + low-pass)
    MEDIAN,              // Median filter for outlier rejection
    EXPONENTIAL_MOVING   // Exponential moving average
};

/**
 * @brief SensorFusion class for combining sensor data
 *
 * EDUCATIONAL: This class demonstrates how to improve sensor accuracy
 * by intelligently combining multiple readings. Perfect for learning
 * about signal processing in embedded systems.
 */
class SensorFusion : public RobotSubsystem {
private:
    // Weighted average state
    float _weight1;
    float _weight2;
    float _fusedValue;

    // Complementary filter state
    float _alpha;  // Coefficient for complementary filter (0.0-1.0)
    float _complementaryOutput;

    // Median filter state
    float _samples[MAX_FUSION_SAMPLES];
    int _sampleIndex;
    int _sampleCount;

    // Exponential moving average state
    float _emaValue;
    float _emaAlpha;  // Smoothing factor (0.0-1.0)

    // Sensor health tracking
    unsigned long _sensor1LastUpdate;
    unsigned long _sensor2LastUpdate;
    int _sensor1ErrorCount;
    int _sensor2ErrorCount;
    const unsigned long SENSOR_TIMEOUT = 5000;  // 5 seconds

    // Statistics
    float _minValue;
    float _maxValue;
    float _averageValue;
    int _totalSamples;

    // Helper methods
    float calculateMedian(float* values, int count);
    void updateStatistics(float value);
    bool isSensorHealthy(unsigned long lastUpdate, int errorCount);

public:
    /**
     * @brief Constructor
     */
    SensorFusion();

    /**
     * @brief Initialize sensor fusion system
     * @return true if successful
     */
    bool initialize() override;

    /**
     * @brief Update fusion calculations
     *
     * EDUCATIONAL: Call this periodically to update internal state.
     * The actual fusion happens when you call the specific fusion methods.
     */
    void update() override;

    /**
     * @brief Check if sensor fusion is ready
     * @return true if initialized
     */
    bool isReady() override;

    /**
     * @brief Get current status string
     * @return Status description
     */
    String getStatus() override;

    // ========================================================================
    // WEIGHTED AVERAGE FUSION
    // ========================================================================

    /**
     * @brief Set weights for weighted average fusion
     * @param weight1 Weight for first sensor (0.0-1.0)
     * @param weight2 Weight for second sensor (0.0-1.0)
     *
     * EDUCATIONAL: Weights don't need to sum to 1.0, they'll be normalized.
     * Higher weight = more trust in that sensor's reading.
     */
    void setWeights(float weight1, float weight2);

    /**
     * @brief Fuse two sensor values using weighted average
     * @param sensor1Value Reading from first sensor
     * @param sensor2Value Reading from second sensor
     * @return Fused value
     *
     * EDUCATIONAL: Formula: output = (w1*s1 + w2*s2) / (w1 + w2)
     * Example: If sensor1 is more accurate, give it higher weight.
     */
    float fuseWeightedAverage(float sensor1Value, float sensor2Value);

    /**
     * @brief Get last weighted average fusion result
     * @return Last fused value
     */
    float getWeightedAverage() const { return _fusedValue; }

    // ========================================================================
    // COMPLEMENTARY FILTER FUSION
    // ========================================================================

    /**
     * @brief Set alpha coefficient for complementary filter
     * @param alpha Filter coefficient (0.0-1.0)
     *
     * EDUCATIONAL: Alpha determines the balance between sensors:
     * - alpha near 1.0 = trust sensor1 (high-pass filter)
     * - alpha near 0.0 = trust sensor2 (low-pass filter)
     * - alpha = 0.98 is common for gyro/accelerometer fusion
     */
    void setComplementaryAlpha(float alpha);

    /**
     * @brief Fuse two sensors using complementary filter
     * @param fastSensor High-frequency but drifty sensor (e.g., gyro)
     * @param slowSensor Low-frequency stable sensor (e.g., accelerometer)
     * @param dt Time delta since last update in seconds
     * @return Fused value
     *
     * EDUCATIONAL: Formula: output = alpha*(prev + fastSensor*dt) + (1-alpha)*slowSensor
     * Perfect for IMU sensor fusion (gyro + accel).
     */
    float fuseComplementary(float fastSensor, float slowSensor, float dt);

    /**
     * @brief Get last complementary filter result
     * @return Last complementary output
     */
    float getComplementaryOutput() const { return _complementaryOutput; }

    // ========================================================================
    // MEDIAN FILTER
    // ========================================================================

    /**
     * @brief Add sample to median filter buffer
     * @param value New sensor reading
     *
     * EDUCATIONAL: Median filter stores recent samples and returns
     * the middle value when sorted. Great for removing spikes and outliers.
     */
    void addSample(float value);

    /**
     * @brief Get median of stored samples
     * @return Median value
     *
     * EDUCATIONAL: The median is less sensitive to outliers than mean.
     * Example: [10, 11, 12, 999] → median=11.5, mean=258
     */
    float getMedian();

    /**
     * @brief Clear all samples from median filter
     */
    void clearSamples();

    // ========================================================================
    // EXPONENTIAL MOVING AVERAGE
    // ========================================================================

    /**
     * @brief Set alpha for exponential moving average
     * @param alpha Smoothing factor (0.0-1.0)
     *
     * EDUCATIONAL: EMA is a low-pass filter:
     * - alpha near 1.0 = fast response, less smoothing
     * - alpha near 0.0 = slow response, more smoothing
     * - alpha = 0.1 to 0.3 is typical for sensor smoothing
     */
    void setEmaAlpha(float alpha);

    /**
     * @brief Update exponential moving average with new value
     * @param newValue New sensor reading
     * @return Updated EMA value
     *
     * EDUCATIONAL: Formula: EMA = alpha*newValue + (1-alpha)*previousEMA
     * Each new reading contributes proportionally to the average.
     */
    float updateEma(float newValue);

    /**
     * @brief Get current EMA value
     * @return EMA output
     */
    float getEma() const { return _emaValue; }

    /**
     * @brief Reset EMA to initial value
     * @param initialValue Starting value for EMA
     */
    void resetEma(float initialValue = 0.0f);

    // ========================================================================
    // SENSOR HEALTH MONITORING
    // ========================================================================

    /**
     * @brief Report sensor update for health tracking
     * @param sensor Sensor number (1 or 2)
     * @param error Set to true if this reading had an error
     */
    void reportSensorUpdate(int sensor, bool error = false);

    /**
     * @brief Check if sensor is healthy
     * @param sensor Sensor number (1 or 2)
     * @return true if sensor is responding and reliable
     */
    bool isSensorHealthy(int sensor);

    /**
     * @brief Get sensor error count
     * @param sensor Sensor number (1 or 2)
     * @return Number of errors recorded
     */
    int getSensorErrorCount(int sensor);

    // ========================================================================
    // STATISTICS
    // ========================================================================

    /**
     * @brief Get minimum value seen
     * @return Minimum fused value
     */
    float getMinValue() const { return _minValue; }

    /**
     * @brief Get maximum value seen
     * @return Maximum fused value
     */
    float getMaxValue() const { return _maxValue; }

    /**
     * @brief Get average of all fused values
     * @return Running average
     */
    float getAverageValue() const { return _averageValue; }

    /**
     * @brief Get total number of samples processed
     * @return Sample count
     */
    int getTotalSamples() const { return _totalSamples; }

    /**
     * @brief Reset statistics
     */
    void resetStatistics();
};

#endif // SENSOR_FUSION_H
