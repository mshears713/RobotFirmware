/**
 * @file main.cpp
 * @brief Main firmware entry point for ESP32 Robotics Project
 *
 * ESP32 Robotics Firmware - WALL-E & EVE Apprenticeship
 * Phase 2: Core Firmware Features & Subsystem Control
 *
 * EDUCATIONAL NOTE: This is the heart of the embedded firmware. The structure
 * follows the standard Arduino pattern:
 * 1. Global declarations and includes
 * 2. setup() - runs once at startup
 * 3. loop() - runs continuously forever
 *
 * Phase 2 additions:
 * - I2C sensor integration (BME280)
 * - SPI device communication
 * - PWM motor control
 * - Telemetry data collection and reporting
 */

#include <Arduino.h>
#include "config.h"
#include "RobotSubsystem.h"
#include "ServoArm.h"
#include "I2CSensor.h"
#include "SPIDevice.h"
#include "Locomotion.h"
#include "TelemetryData.h"

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
void updateLED();
void updateButton();
void updateTelemetry();
void printDebugInfo();
void printTelemetry();
void runDemoSequence();

// ============================================================================
// SETUP FUNCTION
// ============================================================================

void setup() {
    setupSerial();

    Serial.println("\n\n");
    Serial.println("========================================");
    Serial.println("  ESP32 Robotics Firmware");
    Serial.println("  WALL-E & EVE Apprenticeship");
    Serial.println("  Phase 2: Core Features");
    Serial.println("========================================");
    Serial.println();

    setupGPIO();
    setupSubsystems();

    Serial.println();
    Serial.println("========================================");
    Serial.println("  Initialization Complete!");
    Serial.println("  Entering main loop...");
    Serial.println("========================================");
    Serial.println();
    Serial.println("Press BOOT button to cycle through demo modes:");
    Serial.println("  Mode 0: Servo sweep");
    Serial.println("  Mode 1: Motor forward");
    Serial.println("  Mode 2: Motor rotation");
    Serial.println("  Mode 3: All systems active");
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
    int total = 4;  // servoArm, i2cSensor, spiDevice, locomotion

    if (servoArm && servoArm->isReady()) ready++;
    if (i2cSensor && i2cSensor->isReady()) ready++;
    if (spiDevice && spiDevice->isReady()) ready++;
    if (locomotion && locomotion->isReady()) ready++;

    telemetry.subsystemsReady = ready;
    telemetry.subsystemsTotal = total;
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
// EDUCATIONAL NOTES - PHASE 2 COMPLETE
// ============================================================================

/**
 * CONGRATULATIONS! You've completed Phase 2 of the ESP32 Robotics Firmware.
 *
 * What you've learned:
 * ✓ I2C communication protocol (BME280 sensor)
 * ✓ SPI communication protocol (generic device)
 * ✓ PWM motor control (differential drive)
 * ✓ Telemetry data collection and JSON serialization
 * ✓ Multi-subsystem integration
 * ✓ State machine implementation (demo modes)
 *
 * New subsystems added:
 * - I2CSensor: Environmental sensing (temperature, humidity, pressure)
 * - SPIDevice: High-speed serial communication
 * - Locomotion: DC motor control for robot movement
 * - TelemetryData: Complete system state snapshot
 *
 * Key concepts mastered:
 * - I2C addressing and bus scanning
 * - SPI transaction patterns (CS, MOSI, MISO, SCK)
 * - PWM duty cycle for motor speed control
 * - H-bridge motor direction control
 * - JSON serialization with ArduinoJson
 * - Structured telemetry design
 *
 * Next steps (Phase 3):
 * - Wi-Fi connectivity (station mode)
 * - HTTP web server on ESP32
 * - RESTful API endpoints (/telemetry, /command)
 * - Streamlit UI for robot monitoring and control
 * - Real-time telemetry streaming
 *
 * Try these experiments:
 * 1. Add more demo modes with different movement patterns
 * 2. Implement PID control for precise servo positioning
 * 3. Add sensor fusion (combine multiple sensor readings)
 * 4. Create custom telemetry fields
 * 5. Log telemetry to SD card
 * 6. Implement emergency stop on sensor threshold
 *
 * Hardware testing checklist:
 * ☐ BME280 sensor connected via I2C (SDA=21, SCL=22, pull-ups required)
 * ☐ Motor driver connected (L298N, DRV8833, or similar)
 * ☐ Motors powered separately (NOT from ESP32 3.3V!)
 * ☐ Common ground between ESP32 and motor power supply
 * ☐ Servo powered appropriately (5V, sufficient current)
 *
 * Remember: Hardware debugging is part of the learning process!
 */
