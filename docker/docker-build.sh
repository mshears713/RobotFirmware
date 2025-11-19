#!/bin/bash
#
# ESP32 Robotics Firmware - Docker Build Script for WSL/Linux
# Builds firmware using containerized PlatformIO environment
#

set -e

echo "========================================"
echo "  ESP32 Firmware - Docker Build (WSL)"
echo "========================================"
echo ""

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "ERROR: Docker is not running!"
    echo "Please start Docker and try again."
    exit 1
fi

cd "$(dirname "$0")/.."

echo "Building Docker image..."
docker-compose -f docker/docker-compose.yml build firmware-builder

echo ""
echo "Building firmware..."
docker-compose -f docker/docker-compose.yml run --rm firmware-builder pio run

echo ""
echo "========================================"
echo "  Build Complete!"
echo "========================================"
echo ""
echo "Firmware binary location:"
echo "  firmware/.pio/build/esp32dev/firmware.bin"
echo ""
echo "Next steps:"
echo "  1. To upload: ./docker/docker-build-upload.sh"
echo "  2. To test: ./docker/docker-build-test.sh"
echo ""
