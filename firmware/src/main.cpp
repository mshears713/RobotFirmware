/**
 * @file main.cpp
 * @brief Main firmware entry point for ESP32 Robotics Project
 *
 * ESP32 Robotics Firmware - WALL-E & EVE Apprenticeship
 * Phase 4: Additional Features, Testing & Optimization
 *
 * EDUCATIONAL NOTE: This is the heart of the embedded firmware. The structure
 * follows the standard Arduino pattern:
 * 1. Global declarations and includes
 * 2. setup() - runs once at startup
 * 3. loop() - runs continuously forever
 *
 * Phase 3 additions:
 * - Wi-Fi connectivity (station mode)
 * - HTTP web server with REST API
 * - Remote telemetry via JSON
 * - Remote command execution
 * - Integration with Streamlit UI
 *
 * Phase 4 additions:
 * - Power management with multiple power modes
 * - Sensor fusion for improved accuracy
 * - Comprehensive error detection and recovery
 * - Enhanced mode management
 * - System health monitoring
 */

#include <Arduino.h>
#include "config.h"
#include "RobotSubsystem.h"
#include "ServoArm.h"
#include "I2CSensor.h"
#include "SPIDevice.h"
#include "Locomotion.h"
#include "TelemetryData.h"
#include "WiFiManager.h"
#include "RobotWebServer.h"
#include "CommandProcessor.h"
#include "PowerManager.h"
#include "SensorFusion.h"
#include "ErrorManager.h"

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// LED state tracking
bool ledState = false;
unsigned long lastLedToggle = 0;

// Button input tracking
bool lastButtonState = HIGH;
bool currentButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Debug output timing
unsigned long lastDebugPrint = 0;
unsigned long lastTelemetryPrint = 0;

// Operational mode
String currentMode = "idle";  // idle, active, low_power

// Subsystem instances
ServoArm* servoArm = nullptr;
I2CSensor* i2cSensor = nullptr;
SPIDevice* spiDevice = nullptr;
Locomotion* locomotion = nullptr;

// Phase 3: Networking subsystems
WiFiManager* wifiManager = nullptr;
RobotWebServer* webServer = nullptr;
CommandProcessor* commandProcessor = nullptr;

// Phase 4: Advanced subsystems
PowerManager* powerManager = nullptr;
SensorFusion* sensorFusion = nullptr;
ErrorManager* errorManager = nullptr;

// Telemetry data
TelemetryData telemetry;

// Demo mode state machine
int demoState = 0;
unsigned long lastDemoTransition = 0;
const unsigned long demoInterval = 3000;  // Change demo state every 3 seconds

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

void setupSerial();
void setupGPIO();
void setupSubsystems();
void setupNetworking();
void updateLED();
void updateButton();
void updateTelemetry();
void printDebugInfo();
void printTelemetry();
void runDemoSequence();

// Phase 3: Callback functions for web server
TelemetryData provideTelemetry();
void handleCommand(const JsonDocument& command);

// ============================================================================
// SETUP FUNCTION
// ============================================================================

void setup() {
    setupSerial();

    Serial.println("\n\n");
    Serial.println("========================================");
    Serial.println("  ESP32 Robotics Firmware");
    Serial.println("  WALL-E & EVE Apprenticeship");
    Serial.println("  Phase 4: Advanced Features");
    Serial.println("========================================");
    Serial.println();

    setupGPIO();
    setupSubsystems();
    setupNetworking();

    Serial.println();
    Serial.println("========================================");
    Serial.println("  Initialization Complete!");
    Serial.println("  Entering main loop...");
    Serial.println("========================================");
    Serial.println();

    if (wifiManager && wifiManager->isReady()) {
        Serial.println("🌐 Wi-Fi Connected!");
        Serial.print("   IP Address: ");
        Serial.println(wifiManager->getIPAddress());
        Serial.print("   Open in browser: http://");
        Serial.println(wifiManager->getIPAddress());
        Serial.println();
    } else {
        Serial.println("⚠ Wi-Fi not connected - check config.h");
        Serial.println();
    }

    Serial.println("Press BOOT button to cycle through demo modes:");
    Serial.println("  Mode 0: Servo sweep");
    Serial.println("  Mode 1: Motor forward");
    Serial.println("  Mode 2: Motor rotation");
    Serial.println("  Mode 3: All systems active");
    Serial.println();
    Serial.println("Or control remotely via HTTP API or Streamlit UI!");
    Serial.println();
}

// ============================================================================
// MAIN LOOP FUNCTION
// ============================================================================

void loop() {
    // Update LED (status indicator)
    updateLED();

    // Check button input
    updateButton();

    // Update all subsystems
    if (servoArm != nullptr && servoArm->isReady()) {
        servoArm->update();
    }

    if (i2cSensor != nullptr && i2cSensor->isReady()) {
        i2cSensor->update();
    }

    if (spiDevice != nullptr && spiDevice->isReady()) {
        spiDevice->update();
    }

    if (locomotion != nullptr && locomotion->isReady()) {
        locomotion->update();
    }

    // Phase 3: Update networking subsystems
    if (wifiManager != nullptr) {
        wifiManager->update();
    }

    if (webServer != nullptr && webServer->isReady()) {
        webServer->update();
    }

    // Phase 4: Update advanced subsystems
    if (errorManager != nullptr && errorManager->isReady()) {
        errorManager->update();
    }

    if (powerManager != nullptr && powerManager->isReady()) {
        powerManager->update();
    }

    if (sensorFusion != nullptr && sensorFusion->isReady()) {
        sensorFusion->update();
    }

    // Update telemetry data structure
    updateTelemetry();

    // Run automated demo sequence
    if (currentMode == "active") {
        runDemoSequence();
    }

    // Periodic debug output
    if (DEBUG_VERBOSE && DEBUG_SUBSYSTEM_STATUS) {
        unsigned long now = millis();
        if (now - lastDebugPrint >= DEBUG_PRINT_INTERVAL) {
            printDebugInfo();
            lastDebugPrint = now;
        }
    }

    // Periodic telemetry output (JSON format)
    unsigned long now = millis();
    if (now - lastTelemetryPrint >= TELEMETRY_INTERVAL * 10) {  // Every second
        printTelemetry();
        lastTelemetryPrint = now;
    }

    // Small delay for background tasks
    delay(10);
}

// ============================================================================
// INITIALIZATION FUNCTIONS
// ============================================================================

void setupSerial() {
    Serial.begin(SERIAL_BAUD_RATE);

    unsigned long startTime = millis();
    while (!Serial && (millis() - startTime < 3000)) {
        ; // Wait for serial connection
    }

    Serial.println();
    Serial.println("[INIT] Serial communication initialized");
    Serial.print("[INIT] Baud rate: ");
    Serial.println(SERIAL_BAUD_RATE);
}

void setupGPIO() {
    Serial.println("[INIT] Configuring GPIO pins...");

    pinMode(LED_BUILTIN_PIN, OUTPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(LED_BUILTIN_PIN, LOW);
    digitalWrite(STATUS_LED_PIN, LOW);

    pinMode(BUTTON_INPUT_PIN, INPUT_PULLUP);

    Serial.println("[INIT] ✓ GPIO configuration complete");
}

void setupSubsystems() {
    Serial.println("[INIT] Initializing subsystems...");
    Serial.println();

    // Initialize ServoArm
    servoArm = new ServoArm(SERVO_ARM_PIN);
    if (servoArm->initialize()) {
        Serial.println("[INIT] ✓ ServoArm ready");
    } else {
        Serial.println("[INIT] ✗ ServoArm initialization failed");
    }

    Serial.println();

    // Initialize I2C Sensor
    i2cSensor = new I2CSensor(BME280_I2C_ADDRESS);
    if (i2cSensor->initialize()) {
        Serial.println("[INIT] ✓ I2C Sensor ready");
    } else {
        Serial.println("[INIT] ⚠ I2C Sensor initialization failed");
        Serial.println("[INIT] System will continue without sensor data");
    }

    Serial.println();

    // Initialize SPI Device
    spiDevice = new SPIDevice(SPI_CS_PIN);
    if (spiDevice->initialize()) {
        Serial.println("[INIT] ✓ SPI Device ready");
    } else {
        Serial.println("[INIT] ⚠ SPI Device initialization failed");
    }

    Serial.println();

    // Initialize Locomotion (motors)
    locomotion = new Locomotion();
    if (locomotion->initialize()) {
        Serial.println("[INIT] ✓ Locomotion ready");
        // Motors start disabled for safety
    } else {
        Serial.println("[INIT] ✗ Locomotion initialization failed");
    }

    Serial.println();

    // Initial servo demo
    Serial.println("[INIT] Testing servo movement...");
    if (servoArm != nullptr && servoArm->isReady()) {
        servoArm->setTargetAngle(45);
        delay(500);
        servoArm->setTargetAngle(135);
        delay(500);
        servoArm->setTargetAngle(90);
        delay(500);
        Serial.println("[INIT] ✓ Servo test complete");
    }
}

void setupNetworking() {
    Serial.println("[INIT] Initializing networking...");
    Serial.println();

    // Initialize WiFi
    wifiManager = new WiFiManager(WIFI_SSID, WIFI_PASSWORD);
    if (wifiManager->initialize()) {
        Serial.println("[INIT] ✓ WiFi Manager ready");
    } else {
        Serial.println("[INIT] ⚠ WiFi connection failed");
        Serial.println("[INIT] Will retry in background");
    }

    Serial.println();

    // Phase 4: Initialize Error Manager (first, so other subsystems can use it)
    errorManager = new ErrorManager();
    if (errorManager->initialize()) {
        Serial.println("[INIT] ✓ Error Manager ready");
    } else {
        Serial.println("[INIT] ✗ Error Manager initialization failed");
    }

    Serial.println();

    // Phase 4: Initialize Power Manager
    powerManager = new PowerManager();
    if (powerManager->initialize()) {
        Serial.println("[INIT] ✓ Power Manager ready");
    } else {
        Serial.println("[INIT] ✗ Power Manager initialization failed");
    }

    Serial.println();

    // Phase 4: Initialize Sensor Fusion
    sensorFusion = new SensorFusion();
    if (sensorFusion->initialize()) {
        Serial.println("[INIT] ✓ Sensor Fusion ready");
    } else {
        Serial.println("[INIT] ✗ Sensor Fusion initialization failed");
    }

    Serial.println();

    // Initialize Command Processor (now with Phase 4 subsystems)
    commandProcessor = new CommandProcessor(servoArm, locomotion, powerManager, errorManager);
    Serial.println("[INIT] ✓ Command Processor ready");

    Serial.println();

    // Initialize Web Server
    webServer = new RobotWebServer(HTTP_SERVER_PORT);

    // Register telemetry provider callback
    webServer->setTelemetryProvider(provideTelemetry);

    // Register command callback
    webServer->setCommandCallback(handleCommand);

    if (webServer->initialize()) {
        Serial.println("[INIT] ✓ Web Server ready");
    } else {
        Serial.println("[INIT] ✗ Web Server initialization failed");
    }
}

// ============================================================================
// UPDATE FUNCTIONS
// ============================================================================

void updateLED() {
    unsigned long currentTime = millis();

    if (currentTime - lastLedToggle >= LED_BLINK_INTERVAL) {
        ledState = !ledState;
        digitalWrite(LED_BUILTIN_PIN, ledState ? HIGH : LOW);
        digitalWrite(STATUS_LED_PIN, ledState ? HIGH : LOW);
        lastLedToggle = currentTime;
    }
}

void updateButton() {
    bool reading = digitalRead(BUTTON_INPUT_PIN);

    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (reading != currentButtonState) {
            currentButtonState = reading;

            // Button pressed (LOW with pullup)
            if (currentButtonState == LOW) {
                Serial.println();
                Serial.println("========================================");
                Serial.println("  🔘 BUTTON PRESSED!");
                Serial.println("========================================");

                // Cycle through demonstration modes
                demoState = (demoState + 1) % 4;
                lastDemoTransition = millis();

                switch (demoState) {
                    case 0:
                        Serial.println("  Mode 0: Servo Sweep Demo");
                        currentMode = "active";
                        if (locomotion) locomotion->disable();
                        break;

                    case 1:
                        Serial.println("  Mode 1: Motor Forward Demo");
                        currentMode = "active";
                        if (locomotion) locomotion->enable();
                        break;

                    case 2:
                        Serial.println("  Mode 2: Motor Rotation Demo");
                        currentMode = "active";
                        if (locomotion) locomotion->enable();
                        break;

                    case 3:
                        Serial.println("  Mode 3: All Systems Active");
                        currentMode = "active";
                        if (locomotion) locomotion->enable();
                        break;
                }

                Serial.println("========================================");
                Serial.println();
            }
        }
    }

    lastButtonState = reading;
}

void updateTelemetry() {
    // Collect data from all subsystems into telemetry structure

    // System info
    telemetry.timestamp = millis();
    telemetry.freeHeap = ESP.getFreeHeap();
    telemetry.operationalMode = currentMode;

    // Servo data
    if (servoArm != nullptr) {
        telemetry.servoCurrentAngle = servoArm->getCurrentAngle();
        telemetry.servoTargetAngle = servoArm->getTargetAngle();
        telemetry.servoEnabled = servoArm->isEnabled();
        telemetry.servoAtTarget = servoArm->hasReachedTarget();
    } else {
        telemetry.servoCurrentAngle = 0;
        telemetry.servoTargetAngle = 0;
        telemetry.servoEnabled = false;
        telemetry.servoAtTarget = false;
    }

    // Environmental sensor data
    if (i2cSensor != nullptr) {
        telemetry.temperature = i2cSensor->getTemperature();
        telemetry.humidity = i2cSensor->getHumidity();
        telemetry.pressure = i2cSensor->getPressure();
        telemetry.altitude = i2cSensor->getAltitude();
        telemetry.sensorReadCount = i2cSensor->getReadCount();
        telemetry.sensorErrorCount = i2cSensor->getErrorCount();
    } else {
        telemetry.temperature = 0;
        telemetry.humidity = 0;
        telemetry.pressure = 0;
        telemetry.altitude = 0;
        telemetry.sensorReadCount = 0;
        telemetry.sensorErrorCount = 0;
    }

    // Motor data
    if (locomotion != nullptr) {
        telemetry.motorASpeed = locomotion->getMotorASpeed();
        telemetry.motorBSpeed = locomotion->getMotorBSpeed();
        telemetry.motorADirection = locomotion->getMotorADirection();
        telemetry.motorBDirection = locomotion->getMotorBDirection();
        telemetry.motorsEnabled = locomotion->areMotorsEnabled();
    } else {
        telemetry.motorASpeed = 0;
        telemetry.motorBSpeed = 0;
        telemetry.motorADirection = true;
        telemetry.motorBDirection = true;
        telemetry.motorsEnabled = false;
    }

    // SPI data
    if (spiDevice != nullptr) {
        telemetry.spiTransferCount = spiDevice->getTransferCount();
        telemetry.spiTestValue = spiDevice->getTestRegisterValue();
    } else {
        telemetry.spiTransferCount = 0;
        telemetry.spiTestValue = 0;
    }

    // System status
    telemetry.errorCode = 0;  // No errors for now
    telemetry.errorMessage = "";

    // Count ready subsystems
    int ready = 0;
    int total = 7;  // servoArm, i2cSensor, spiDevice, locomotion, powerManager, sensorFusion, errorManager

    if (servoArm && servoArm->isReady()) ready++;
    if (i2cSensor && i2cSensor->isReady()) ready++;
    if (spiDevice && spiDevice->isReady()) ready++;
    if (locomotion && locomotion->isReady()) ready++;
    if (powerManager && powerManager->isReady()) ready++;
    if (sensorFusion && sensorFusion->isReady()) ready++;
    if (errorManager && errorManager->isReady()) ready++;

    telemetry.subsystemsReady = ready;
    telemetry.subsystemsTotal = total;

    // Phase 4: Power management data
    if (powerManager != nullptr) {
        telemetry.powerMode = powerManager->getPowerModeString();
        telemetry.timeSinceActivity = powerManager->getTimeSinceActivity();
        telemetry.estimatedCurrent = powerManager->getEstimatedCurrentDraw();
        telemetry.lastWakeReason = powerManager->getWakeReasonString();
    } else {
        telemetry.powerMode = "unknown";
        telemetry.timeSinceActivity = 0;
        telemetry.estimatedCurrent = 0.0f;
        telemetry.lastWakeReason = "none";
    }

    // Phase 4: Sensor fusion data
    if (sensorFusion != nullptr && i2cSensor != nullptr) {
        // Use sensor fusion to combine temperature readings
        // For demo purposes, fuse the single sensor with itself using EMA
        float rawTemp = i2cSensor->getTemperature();
        telemetry.fusedTemperature = sensorFusion->updateEma(rawTemp);
        telemetry.fusionConfidence = (i2cSensor->getErrorCount() == 0) ? 1.0f : 0.5f;
        telemetry.fusionSampleCount = sensorFusion->getTotalSamples();
    } else {
        telemetry.fusedTemperature = telemetry.temperature;
        telemetry.fusionConfidence = 0.0f;
        telemetry.fusionSampleCount = 0;
    }

    // Phase 4: Error management data
    if (errorManager != nullptr) {
        telemetry.errorLogCount = errorManager->getTotalErrors();
        telemetry.warningCount = errorManager->getTotalWarnings();
        telemetry.recoveryCount = errorManager->getTotalRecoveries();
        telemetry.systemHealth = errorManager->getSystemHealth();

        if (errorManager->hasError()) {
            telemetry.currentErrorSeverity = String("ERROR");
            telemetry.errorCode = (int)errorManager->getCurrentError();
            telemetry.errorMessage = errorManager->getCurrentMessage();
        } else {
            telemetry.currentErrorSeverity = String("INFO");
            telemetry.errorCode = 0;
            telemetry.errorMessage = "";
        }
    } else {
        telemetry.errorLogCount = 0;
        telemetry.warningCount = 0;
        telemetry.recoveryCount = 0;
        telemetry.systemHealth = 100.0f;
        telemetry.currentErrorSeverity = String("INFO");
    }

    // Update operational mode from command processor
    if (commandProcessor != nullptr) {
        telemetry.operationalMode = commandProcessor->getOperationalMode();
    }
}

// ============================================================================
// DEMONSTRATION FUNCTIONS
// ============================================================================

void runDemoSequence() {
    unsigned long now = millis();

    // Don't transition too fast
    if (now - lastDemoTransition < demoInterval) {
        return;
    }

    lastDemoTransition = now;

    switch (demoState) {
        case 0:  // Servo sweep
            if (servoArm != nullptr) {
                static int servoTarget = 0;
                servoTarget = (servoTarget + 45) % 180;
                servoArm->setTargetAngle(servoTarget);

                if (DEBUG_VERBOSE) {
                    Serial.print("[Demo] Servo moving to ");
                    Serial.print(servoTarget);
                    Serial.println("°");
                }
            }
            break;

        case 1:  // Motor forward
            if (locomotion != nullptr && locomotion->areMotorsEnabled()) {
                static bool motorState = false;
                motorState = !motorState;

                if (motorState) {
                    locomotion->moveForward(150);  // 150/255 speed
                    Serial.println("[Demo] Motors FORWARD");
                } else {
                    locomotion->stopMotors();
                    Serial.println("[Demo] Motors STOP");
                }
            }
            break;

        case 2:  // Motor rotation
            if (locomotion != nullptr && locomotion->areMotorsEnabled()) {
                static bool rotateDir = true;
                locomotion->rotateInPlace(128, rotateDir);
                rotateDir = !rotateDir;

                Serial.print("[Demo] Rotating ");
                Serial.println(rotateDir ? "CLOCKWISE" : "COUNTER-CLOCKWISE");
            }
            break;

        case 3:  // All systems
            // Servo movement
            if (servoArm != nullptr) {
                int servoTarget = random(0, 181);
                servoArm->setTargetAngle(servoTarget);
            }

            // Motor movement
            if (locomotion != nullptr && locomotion->areMotorsEnabled()) {
                int choice = random(0, 4);
                switch (choice) {
                    case 0:
                        locomotion->moveForward(120);
                        Serial.println("[Demo] Forward");
                        break;
                    case 1:
                        locomotion->moveBackward(120);
                        Serial.println("[Demo] Backward");
                        break;
                    case 2:
                        locomotion->turnLeft(120, 64);
                        Serial.println("[Demo] Turn Left");
                        break;
                    case 3:
                        locomotion->turnRight(120, 64);
                        Serial.println("[Demo] Turn Right");
                        break;
                }
            }
            break;
    }
}

// ============================================================================
// OUTPUT FUNCTIONS
// ============================================================================

void printDebugInfo() {
    Serial.println();
    Serial.println("========================================");
    Serial.println("  System Status");
    Serial.println("========================================");

    unsigned long uptime = millis() / 1000;
    Serial.print("  Uptime: ");
    Serial.print(uptime);
    Serial.println(" seconds");

    Serial.print("  Free Heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");

    Serial.print("  Mode: ");
    Serial.println(currentMode);

    Serial.print("  Demo State: ");
    Serial.println(demoState);

    Serial.println();
    Serial.println("  Subsystems:");

    if (servoArm != nullptr) {
        Serial.print("    - ");
        Serial.println(servoArm->getStatus());
    }

    if (i2cSensor != nullptr) {
        Serial.print("    - ");
        Serial.println(i2cSensor->getStatus());
    }

    if (spiDevice != nullptr) {
        Serial.print("    - ");
        Serial.println(spiDevice->getStatus());
    }

    if (locomotion != nullptr) {
        Serial.print("    - ");
        Serial.println(locomotion->getStatus());
    }

    // Phase 3: Network status
    if (wifiManager != nullptr) {
        Serial.print("    - ");
        Serial.println(wifiManager->getStatus());
    }

    if (webServer != nullptr) {
        Serial.print("    - ");
        Serial.println(webServer->getStatus());
    }

    // Phase 4: Advanced subsystems
    if (powerManager != nullptr) {
        Serial.print("    - ");
        Serial.println(powerManager->getStatus());
    }

    if (sensorFusion != nullptr) {
        Serial.print("    - ");
        Serial.println(sensorFusion->getStatus());
    }

    if (errorManager != nullptr) {
        Serial.print("    - ");
        Serial.println(errorManager->getStatus());
    }

    Serial.println("========================================");
    Serial.println();
}

void printTelemetry() {
    // Print compact JSON telemetry
    Serial.println();
    Serial.println("--- TELEMETRY ---");
    Serial.println(telemetry.toJson());
    Serial.println("--- END ---");
    Serial.println();
}

// ============================================================================
// PHASE 3: WEB SERVER CALLBACK FUNCTIONS
// ============================================================================

/**
 * @brief Telemetry provider callback for web server
 *
 * EDUCATIONAL: This function is called by the web server when a client
 * requests telemetry data (GET /telemetry). It returns the current
 * telemetry structure populated with data from all subsystems.
 *
 * @return Current telemetry data
 */
TelemetryData provideTelemetry() {
    // Return the global telemetry structure
    // It's kept up-to-date by updateTelemetry() in loop()
    return telemetry;
}

/**
 * @brief Command callback for web server
 *
 * EDUCATIONAL: This function is called when the web server receives
 * a command (POST /command). It forwards the JSON command to the
 * CommandProcessor for execution.
 *
 * @param command JSON document containing command
 */
void handleCommand(const JsonDocument& command) {
    if (commandProcessor != nullptr) {
        bool success = commandProcessor->processCommand(command);

        if (success) {
            Serial.println("[WebAPI] ✓ Command executed successfully");
        } else {
            Serial.println("[WebAPI] ✗ Command execution failed");
        }
    } else {
        Serial.println("[WebAPI] ✗ Command processor not available");
    }
}

// ============================================================================
// EDUCATIONAL NOTES - PHASE 3 COMPLETE
// ============================================================================

/**
 * CONGRATULATIONS! You've completed Phase 3 of the ESP32 Robotics Firmware!
 *
 * What you've learned in Phase 3:
 * ✓ Wi-Fi connectivity (station mode, DHCP)
 * ✓ HTTP web server implementation
 * ✓ RESTful API design and implementation
 * ✓ Remote telemetry streaming via JSON
 * ✓ Remote command execution
 * ✓ Command pattern implementation
 * ✓ Callback function patterns
 * ✓ Integration with Streamlit UI
 *
 * Phase 3 subsystems added:
 * - WiFiManager: Wi-Fi connectivity with auto-reconnect
 * - RobotWebServer: HTTP server with REST API (/telemetry, /command, /status)
 * - CommandProcessor: Command parsing and execution
 *
 * Complete system capabilities:
 * 1. Hardware Control:
 *    - Servo arm positioning
 *    - DC motor control (differential drive)
 *    - Environmental sensing (temp, humidity, pressure)
 *    - SPI device communication
 *
 * 2. Networking:
 *    - Wi-Fi connectivity with status monitoring
 *    - HTTP web server on port 80
 *    - JSON-based API
 *    - CORS support for web browsers
 *
 * 3. Remote Capabilities:
 *    - Real-time telemetry streaming
 *    - Remote command execution
 *    - Web browser interface
 *    - Streamlit UI integration
 *
 * API Endpoints:
 * - GET  /           - HTML info page
 * - GET  /telemetry  - JSON telemetry data
 * - POST /command    - Execute JSON commands
 * - GET  /status     - Server health check
 *
 * Example Commands (via Streamlit or curl):
 * {"command": "set_servo_angle", "value": 90}
 * {"command": "move_forward", "speed": 150}
 * {"command": "turn_left", "speed": 120, "turn_rate": 64}
 * {"command": "stop_motors"}
 * {"command": "enable_motors"}
 *
 * Next steps (Phase 4):
 * - Low-power modes and power management
 * - Error recovery and fault tolerance
 * - Enhanced UI features (charts, history)
 * - Sensor fusion algorithms
 * - Advanced telemetry (bandwidth optimization)
 *
 * Try these experiments:
 * 1. Create custom commands in CommandProcessor
 * 2. Add authentication to web API
 * 3. Implement command queuing
 * 4. Add WebSocket support for real-time bidirectional communication
 * 5. Create mobile-friendly web interface
 * 6. Log commands and telemetry to SD card
 *
 * Hardware + Network testing checklist:
 * ☐ ESP32 connected to Wi-Fi network (check Serial output for IP)
 * ☐ All sensors and actuators connected (from Phase 1-2)
 * ☐ Access http://[ESP32_IP] from web browser
 * ☐ Test /telemetry endpoint
 * ☐ Test /command endpoint with curl or Streamlit
 * ☐ Monitor Serial output for command execution
 *
 * Troubleshooting:
 * - Can't connect to Wi-Fi: Check SSID/password in config.h
 * - Can't access web server: Verify IP address, check firewall
 * - Commands not working: Check Serial output for errors
 * - Telemetry not updating: Verify subsystems initialized correctly
 *
 * Remember: Network debugging requires both hardware AND software skills!
 */
