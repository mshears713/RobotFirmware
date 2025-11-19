@echo off
REM ESP32 Robotics Firmware - Docker Build Script for Windows
REM Builds firmware using containerized PlatformIO environment

echo ========================================
echo  ESP32 Firmware - Docker Build (Windows)
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
docker-compose -f docker\docker-compose.yml build firmware-builder

echo.
echo Building firmware...
docker-compose -f docker\docker-compose.yml run --rm firmware-builder pio run

echo.
echo ========================================
echo  Build Complete!
echo ========================================
echo.
echo Firmware binary location:
echo   firmware\.pio\build\esp32dev\firmware.bin
echo.
echo Next steps:
echo   1. To upload: docker-build-upload.bat
echo   2. To test: docker-build-test.bat
echo.
pause
