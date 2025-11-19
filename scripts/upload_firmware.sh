#!/bin/bash
#
# ESP32 Robotics Firmware - Upload Script
# Phase 5: Deployment Preparation
#

set -e  # Exit on error

echo "========================================="
echo "  ESP32 Robotics Firmware Uploader"
echo "========================================="
echo ""

# Change to firmware directory
cd "$(dirname "$0")/../firmware"

# Check if port is specified
if [ -n "$1" ]; then
    PORT="$1"
    echo "📡 Using specified port: $PORT"
else
    echo "🔍 Auto-detecting ESP32..."
    PORT=""
fi

echo ""
echo "📤 Uploading firmware to ESP32..."
if [ -n "$PORT" ]; then
    pio run --target upload --upload-port "$PORT"
else
    pio run --target upload
fi

echo ""
echo "✅ Upload complete!"
echo ""
echo "Next steps:"
echo "  1. Monitor serial output: pio device monitor"
echo "  2. Note the ESP32 IP address from serial output"
echo "  3. Update ESP32_IP in ui/robot_console.py"
echo "  4. Run Streamlit UI: streamlit run ui/robot_console.py"
echo ""
