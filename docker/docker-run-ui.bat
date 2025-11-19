@echo off
REM ESP32 Robotics UI - Docker Launch Script for Windows
REM Launches Streamlit UI in Docker container

echo ========================================
echo  ESP32 Robot Console UI - Docker (Windows)
echo ========================================
echo.

REM Check if Docker is running
docker info >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Docker is not running!
    echo Please start Docker Desktop and try again.
    pause
    exit /b 1
)

echo Building Docker image...
docker-compose -f docker\docker-compose.yml build robot-ui

echo.
echo Starting Streamlit UI...
docker-compose -f docker\docker-compose.yml up robot-ui

REM This keeps the window open if there's an error
if %errorlevel% neq 0 (
    echo.
    echo ERROR: Failed to start UI
    pause
)
