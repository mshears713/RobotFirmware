#!/bin/bash
#
# ESP32 Robotics UI - Docker Launch Script for WSL/Linux
# Launches Streamlit UI in Docker container
#

set -e

echo "========================================"
echo "  ESP32 Robot Console UI - Docker (WSL)"
echo "========================================"
echo ""

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "ERROR: Docker is not running!"
    exit 1
fi

cd "$(dirname "$0")/.."

echo "Building Docker image..."
docker-compose -f docker/docker-compose.yml build robot-ui

echo ""
echo "Starting Streamlit UI..."
echo ""
echo "Access the UI at: http://localhost:8501"
echo ""
echo "Press Ctrl+C to stop"
echo ""

docker-compose -f docker/docker-compose.yml up robot-ui
