#!/bin/bash
#
# ESP32 Robotics UI - Launch Script
# Phase 5: Deployment Preparation
#

set -e  # Exit on error

echo "========================================="
echo "  ESP32 Robotics Streamlit UI Launcher"
echo "========================================="
echo ""

# Change to project root
cd "$(dirname "$0")/.."

# Check if virtual environment exists
if [ ! -d "venv" ]; then
    echo "📦 Creating Python virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
echo "🔧 Activating virtual environment..."
source venv/bin/activate

# Install/update dependencies
echo "📥 Installing/updating Python dependencies..."
pip install -q --upgrade pip
pip install -q -r ui/requirements.txt

echo ""
echo "🚀 Launching Streamlit UI..."
echo ""
echo "IMPORTANT: Update ESP32_IP in the UI sidebar!"
echo ""

streamlit run ui/robot_console.py
