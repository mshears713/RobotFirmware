@echo off
REM ESP32 Robotics Firmware - Docker Upload Script for Windows
REM Uploads firmware to ESP32 using containerized environment

echo ========================================
echo  ESP32 Firmware - Docker Upload (Windows)
echo ========================================
echo.

REM Check if Docker is running
docker info >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Docker is not running!
    pause
    exit /b 1
)

echo.
echo IMPORTANT: USB Device Access in Docker Desktop
echo ============================================
echo.
echo For Windows with Docker Desktop:
echo   1. Go to Docker Desktop Settings
echo   2. Navigate to Resources ^> USB devices
echo   3. Enable and share your ESP32 USB device
echo.
echo For WSL2 Backend:
echo   1. Use usbipd-win to share USB devices
echo   2. Install: winget install usbipd
echo   3. Share device: usbipd wsl attach --busid X-Y
echo.
echo Alternatively, use native upload:
echo   cd firmware
echo   pio run --target upload
echo.
pause

echo.
echo Uploading firmware...
docker-compose -f docker\docker-compose.yml run --rm firmware-builder pio run --target upload

echo.
echo ========================================
echo  Upload Complete!
echo ========================================
echo.
pause
