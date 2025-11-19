#!/bin/bash
#
# ESP32 Robotics Firmware - Docker Upload Script for WSL/Linux
# Uploads firmware to ESP32 using containerized environment
#

set -e

echo "========================================"
echo "  ESP32 Firmware - Docker Upload (WSL)"
echo "========================================"
echo ""

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "ERROR: Docker is not running!"
    exit 1
fi

cd "$(dirname "$0")/.."

echo ""
echo "NOTE: USB Device Access"
echo "======================="
echo ""
echo "For WSL2:"
echo "  1. Install usbipd-win on Windows"
echo "  2. Attach device: usbipd wsl attach --busid X-Y"
echo "  3. Verify: lsusb"
echo ""
echo "For native Linux:"
echo "  USB devices should work automatically with --privileged"
echo ""

# List available serial devices
echo "Available serial devices:"
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  No USB serial devices found"
echo ""

read -p "Continue with upload? (y/n) " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    exit 0
fi

echo ""
echo "Uploading firmware..."
docker-compose -f docker/docker-compose.yml run --rm firmware-builder pio run --target upload

echo ""
echo "========================================"
echo "  Upload Complete!"
echo "========================================"
echo ""
