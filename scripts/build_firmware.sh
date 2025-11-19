#!/bin/bash
#
# ESP32 Robotics Firmware - Build Script
# Phase 5: Deployment Preparation
#

set -e  # Exit on error

echo "========================================="
echo "  ESP32 Robotics Firmware Builder"
echo "========================================="
echo ""

# Change to firmware directory
cd "$(dirname "$0")/../firmware"

echo "📦 Installing/updating PlatformIO dependencies..."
pio lib install

echo ""
echo "🔨 Building firmware..."
pio run

echo ""
echo "✅ Build complete!"
echo ""
echo "Build artifacts:"
ls -lh .pio/build/esp32dev/firmware.* 2>/dev/null || echo "  (Build artifacts in .pio/build/esp32dev/)"

echo ""
echo "Next steps:"
echo "  1. Upload firmware: ./scripts/upload_firmware.sh"
echo "  2. Monitor serial: pio device monitor"
echo "  3. Run tests: pio test"
echo ""
