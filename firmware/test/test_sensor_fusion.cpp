/**
 * @file test_sensor_fusion.cpp
 * @brief Unit tests for SensorFusion class - Phase 4
 */

#include <unity.h>
#include "SensorFusion.h"

SensorFusion* fusion = nullptr;

void setUp() {
    fusion = new SensorFusion();
    fusion->initialize();
}

void tearDown() {
    if (fusion != nullptr) {
        delete fusion;
        fusion = nullptr;
    }
}

/**
 * @brief Test weighted average fusion
 */
void test_weighted_average_fusion() {
    fusion->setWeights(0.7f, 0.3f);

    float sensor1 = 10.0f;
    float sensor2 = 20.0f;

    float result = fusion->fuseWeightedAverage(sensor1, sensor2);

    // Expected: (0.7*10 + 0.3*20) / (0.7+0.3) = (7 + 6) / 1.0 = 13.0
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 13.0f, result);
}

/**
 * @brief Test exponential moving average
 */
void test_exponential_moving_average() {
    fusion->setEmaAlpha(0.5f);
    fusion->resetEma(10.0f);

    // First update: EMA = 0.5*15 + 0.5*10 = 12.5
    float result = fusion->updateEma(15.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 12.5f, result);

    // Second update: EMA = 0.5*20 + 0.5*12.5 = 16.25
    result = fusion->updateEma(20.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 16.25f, result);
}

/**
 * @brief Test median filter
 */
void test_median_filter() {
    fusion->clearSamples();

    // Add samples: 10, 15, 12, 100 (outlier), 11
    fusion->addSample(10.0f);
    fusion->addSample(15.0f);
    fusion->addSample(12.0f);
    fusion->addSample(100.0f);  // Outlier
    fusion->addSample(11.0f);

    // Median should be 12.0 (middle value, ignoring outlier)
    float median = fusion->getMedian();
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 12.0f, median);
}

/**
 * @brief Test statistics tracking
 */
void test_statistics_tracking() {
    fusion->resetStatistics();

    fusion->fuseWeightedAverage(10.0f, 20.0f);
    fusion->fuseWeightedAverage(15.0f, 25.0f);
    fusion->fuseWeightedAverage(20.0f, 30.0f);

    TEST_ASSERT_EQUAL(3, fusion->getTotalSamples());
    TEST_ASSERT_GREATER_THAN(0.0f, fusion->getMinValue());
    TEST_ASSERT_GREATER_THAN(0.0f, fusion->getMaxValue());
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_weighted_average_fusion);
    RUN_TEST(test_exponential_moving_average);
    RUN_TEST(test_median_filter);
    RUN_TEST(test_statistics_tracking);

    return UNITY_END();
}
