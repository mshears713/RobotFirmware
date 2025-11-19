#!/bin/bash
#
# ESP32 Robotics Firmware - Test Script
# Phase 5: Deployment Preparation
#

set -e  # Exit on error

echo "========================================="
echo "  ESP32 Robotics Firmware Tester"
echo "========================================="
echo ""

# Change to firmware directory
cd "$(dirname "$0")/../firmware"

echo "🧪 Running unit tests..."
echo ""

pio test

echo ""
echo "✅ All tests complete!"
echo ""
