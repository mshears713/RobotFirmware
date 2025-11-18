# ESP32 Robotics Firmware - Claude AI Agent Guide

This document provides comprehensive guidance for AI agents working on the ESP32 Robotics Firmware Apprenticeship project. It contains essential information about architecture, development practices, coding standards, and implementation guidelines.

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Architecture & Design Philosophy](#architecture--design-philosophy)
3. [Technology Stack](#technology-stack)
4. [Project Structure](#project-structure)
5. [Development Environment](#development-environment)
6. [Coding Standards](#coding-standards)
7. [Core Components](#core-components)
8. [Communication Protocols](#communication-protocols)
9. [Implementation Guidelines](#implementation-guidelines)
10. [Testing Strategy](#testing-strategy)
11. [Common Tasks](#common-tasks)
12. [Troubleshooting](#troubleshooting)

---

## Project Overview

### Purpose
This is a **beginner-friendly, narrative-driven apprenticeship** for teaching firmware engineering and embedded systems development using ESP32 microcontrollers. The project uses a WALL-E & EVE inspired storyline where learners act as field systems engineers maintaining and upgrading robotics subsystems.

### Key Objectives
- Teach ESP32 firmware development from basics to intermediate level
- Introduce embedded communication protocols (I²C, SPI, PWM)
- Implement Wi-Fi telemetry and remote control
- Build interactive Streamlit UI for robot monitoring and control
- Complete the journey in 3-4 weeks for beginners

### Target Audience
- Beginners in embedded systems development
- Developers familiar with programming but new to firmware
- Learners seeking hands-on robotics experience

---

## Architecture & Design Philosophy

### System Architecture

The system consists of two main components:

```
┌─────────────────────────┐              ┌───────────────────────────┐
│   Streamlit UI (PC)     │ <──Wi-Fi───> │   ESP32 Firmware          │
│                         │              │                           │
│  • Telemetry Display    │              │  • Servo PWM Control      │
│  • Command Console      │              │  • Sensors (I2C/SPI)      │
│  • SQLite Logging       │              │  • Locomotion Control     │
│  • Real-time Charts     │              │  • HTTP Server            │
│  • Mode Management      │              │  • Low-Power Modes        │
│                         │              │  • Sensor Fusion          │
└─────────────────────────┘              └───────────────────────────┘
```

### Design Principles

1. **Progressive Complexity**: Start simple (LED blink), gradually add features
2. **Modularity**: Use subsystem abstraction for clean architecture
3. **Educational First**: Extensive inline comments and documentation
4. **Robustness**: Error handling and recovery mechanisms throughout
5. **Beginner-Friendly**: Interactive demos, tooltips, and guided walkthroughs

### Key Design Patterns

- **Abstract Subsystem Pattern**: Base `RobotSubsystem` class for all hardware modules
- **Main Loop Architecture**: Cooperative multitasking via update() calls
- **Telemetry Struct**: Centralized data structure for system state
- **Command Pattern**: HTTP endpoint handlers for remote control
- **Repository Pattern**: SQLite for telemetry history

---

## Technology Stack

### Firmware (ESP32)
- **Platform**: ESP32 microcontroller (dual-core, Wi-Fi/BLE)
- **Development Environment**: PlatformIO (VSCode extension)
- **Framework**: Arduino-ESP32 or ESP-IDF
- **Language**: C++ (with embedded C for low-level operations)

### UI (Desktop)
- **Framework**: Streamlit (Python)
- **Database**: SQLite (local telemetry logging)
- **Data Processing**: pandas
- **HTTP Client**: requests library

### Communication
- **Wi-Fi**: ESP32 connects to local network or acts as AP
- **Protocol**: HTTP REST API (JSON payloads)
- **Telemetry**: GET /telemetry (periodic fetch)
- **Commands**: POST /command (control actions)

---

## Project Structure

### Expected Directory Layout

```
RobotFirmware/
├── firmware/                      # ESP32 firmware source
│   ├── src/
│   │   ├── main.cpp              # Main entry point and loop
│   │   ├── config.h              # Configuration constants
│   │   ├── subsystems/           # Hardware subsystem classes
│   │   │   ├── RobotSubsystem.h
│   │   │   ├── ServoArm.h/cpp
│   │   │   ├── I2CSensor.h/cpp
│   │   │   ├── SPIDevice.h/cpp
│   │   │   └── Locomotion.h/cpp
│   │   ├── telemetry/
│   │   │   ├── TelemetryData.h
│   │   │   └── TelemetryManager.h/cpp
│   │   ├── network/
│   │   │   ├── WiFiManager.h/cpp
│   │   │   └── WebServer.h/cpp
│   │   └── utils/
│   │       ├── debug.h
│   │       └── lowpower.h/cpp
│   ├── lib/                      # External libraries
│   ├── test/                     # Unit tests
│   └── platformio.ini            # PlatformIO configuration
│
├── ui/                           # Streamlit UI application
│   ├── robot_console.py          # Main Streamlit app
│   ├── components/               # UI component modules
│   │   ├── telemetry_display.py
│   │   ├── command_panel.py
│   │   ├── mode_manager.py
│   │   └── error_display.py
│   ├── database/
│   │   └── telemetry_db.py      # SQLite interface
│   ├── config.py                 # UI configuration
│   └── requirements.txt          # Python dependencies
│
├── docs/                         # Additional documentation
│   ├── architecture.md
│   ├── api.md                    # HTTP API documentation
│   ├── wiring.md                 # Hardware wiring diagrams
│   └── troubleshooting.md
│
├── examples/                     # Example telemetry/commands
│   ├── sample_telemetry.json
│   └── sample_commands.json
│
├── scripts/                      # Build and deployment scripts
│   ├── build_firmware.sh
│   ├── upload_firmware.sh
│   └── run_ui.sh
│
├── README.md                     # Project overview
└── claude.md                     # This file
```

---

## Development Environment

### Firmware Setup (PlatformIO)

#### Installation
```bash
# Install VSCode first, then:
# 1. Open VSCode
# 2. Go to Extensions (Ctrl+Shift+X)
# 3. Search for "PlatformIO IDE"
# 4. Click Install
```

#### Create New Firmware Project
```bash
# Using PlatformIO CLI (or use VSCode GUI)
pio project init --board esp32dev
```

#### platformio.ini Configuration
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps =
    arduino-libraries/Servo@^1.1.8
    adafruit/Adafruit BME280 Library@^2.2.2
    bblanchon/ArduinoJson@^6.21.3
```

#### Build and Upload Commands
```bash
pio run              # Build firmware
pio run -t upload    # Upload to ESP32
pio device monitor   # Open serial monitor
```

### UI Setup (Streamlit)

#### Create Python Environment
```bash
python -m venv venv
source venv/bin/activate  # Linux/macOS
# venv\Scripts\activate   # Windows
```

#### Install Dependencies
```bash
pip install streamlit pandas requests sqlite3
# Or use requirements.txt:
pip install -r ui/requirements.txt
```

#### Run Streamlit App
```bash
streamlit run ui/robot_console.py
```

---

## Coding Standards

### Firmware (C++)

#### Naming Conventions
```cpp
// Classes: PascalCase
class ServoArm : public RobotSubsystem { };

// Functions/Methods: camelCase
void updatePosition();
void setAngle(float degrees);

// Constants: UPPER_SNAKE_CASE
#define SERVO_PIN 18
const int MAX_ANGLE = 180;

// Variables: camelCase
int currentAngle;
bool isEnabled;

// Private members: prefix with underscore
private:
    int _pinNumber;
    bool _isInitialized;
```

#### Code Documentation
```cpp
/**
 * @brief Controls a servo motor for robotic arm movements
 *
 * This class provides PWM-based servo control with angle limits,
 * smooth transitions, and position feedback. Inherits from
 * RobotSubsystem for integration with main loop.
 *
 * @educational This demonstrates PWM control fundamentals and
 * how duty cycle translates to servo position (typically
 * 1ms=0°, 1.5ms=90°, 2ms=180° on a 20ms period).
 */
class ServoArm : public RobotSubsystem {
public:
    /**
     * @brief Initialize servo on specified pin
     * @param pin GPIO pin number for PWM output
     * @param minAngle Minimum allowed angle (default 0)
     * @param maxAngle Maximum allowed angle (default 180)
     */
    ServoArm(int pin, float minAngle = 0, float maxAngle = 180);

    // ... methods
};
```

#### Educational Comments
```cpp
// EDUCATIONAL NOTE: I2C uses two wires - SDA (data) and SCL (clock).
// The master (ESP32) controls clock, slaves respond to their addresses.
// Pull-up resistors (typically 4.7kΩ) are required on both lines.

Wire.begin(SDA_PIN, SCL_PIN);  // Initialize I2C bus
Wire.setClock(100000);         // Set to 100kHz (standard mode)
```

#### Error Handling
```cpp
bool ServoArm::initialize() {
    if (!servo.attach(_pin)) {
        Serial.println("ERROR: Failed to attach servo to pin " + String(_pin));
        _isInitialized = false;
        return false;
    }

    _isInitialized = true;
    Serial.println("SUCCESS: Servo initialized on pin " + String(_pin));
    return true;
}
```

### UI Code (Python)

#### Naming Conventions
```python
# Classes: PascalCase
class TelemetryDisplay:
    pass

# Functions: snake_case
def fetch_telemetry():
    pass

# Constants: UPPER_SNAKE_CASE
ESP32_IP = "192.168.1.100"
TELEMETRY_ENDPOINT = "/telemetry"

# Variables: snake_case
current_mode = "idle"
telemetry_data = {}
```

#### Streamlit Code Structure
```python
import streamlit as st
import requests
import pandas as pd

# Page configuration first
st.set_page_config(
    page_title="Robot Systems Console",
    page_icon="🤖",
    layout="wide"
)

# Session state initialization
if 'esp32_ip' not in st.session_state:
    st.session_state.esp32_ip = "192.168.1.100"

# Main UI layout
def main():
    st.title("🤖 WALL-E Robot Systems Console")

    # Sidebar for configuration
    with st.sidebar:
        render_configuration_panel()

    # Main content area
    col1, col2 = st.columns(2)

    with col1:
        render_telemetry_display()

    with col2:
        render_command_panel()
```

#### Error Handling in UI
```python
try:
    response = requests.get(
        f"http://{esp32_ip}/telemetry",
        timeout=5
    )
    response.raise_for_status()
    telemetry = response.json()
    return telemetry

except requests.exceptions.Timeout:
    st.error("⏱️ Timeout: ESP32 not responding. Check connection.")
    return None

except requests.exceptions.ConnectionError:
    st.error("🔌 Connection Error: Cannot reach ESP32. Verify IP and Wi-Fi.")
    return None

except requests.exceptions.JSONDecodeError:
    st.error("📄 Invalid Data: ESP32 returned malformed JSON.")
    return None
```

---

## Core Components

### 1. RobotSubsystem (Abstract Base Class)

**Purpose**: Provide common interface for all hardware subsystems

**Location**: `firmware/src/subsystems/RobotSubsystem.h`

```cpp
class RobotSubsystem {
public:
    virtual bool initialize() = 0;    // Setup hardware
    virtual void update() = 0;        // Called each loop iteration
    virtual bool isReady() = 0;       // Health check
    virtual String getStatus() = 0;   // For telemetry

protected:
    bool _isInitialized = false;
    unsigned long _lastUpdate = 0;
};
```

**Usage Pattern**:
```cpp
class ServoArm : public RobotSubsystem {
public:
    bool initialize() override {
        // Hardware setup
    }

    void update() override {
        // Per-loop logic
    }

    bool isReady() override {
        return _isInitialized && servo.attached();
    }

    String getStatus() override {
        return "Servo: " + String(currentAngle) + "°";
    }
};
```

### 2. ServoArm Subsystem

**Purpose**: Control servo motors via PWM for robotic arm movements

**Key Features**:
- Angle range limiting (safety)
- Smooth transitions (avoid jerky motion)
- Position feedback
- PWM frequency configuration

**Example Implementation**:
```cpp
class ServoArm : public RobotSubsystem {
private:
    Servo servo;
    int _pin;
    float _currentAngle;
    float _targetAngle;
    float _minAngle, _maxAngle;
    float _transitionSpeed;  // degrees per update

public:
    ServoArm(int pin, float minAngle = 0, float maxAngle = 180)
        : _pin(pin), _minAngle(minAngle), _maxAngle(maxAngle),
          _currentAngle(90), _targetAngle(90), _transitionSpeed(1.0) {}

    bool initialize() override {
        if (servo.attach(_pin)) {
            servo.write(_currentAngle);
            _isInitialized = true;
            return true;
        }
        return false;
    }

    void update() override {
        if (!_isInitialized) return;

        // Smooth transition to target
        if (abs(_targetAngle - _currentAngle) > 0.5) {
            float delta = _targetAngle - _currentAngle;
            float step = constrain(delta, -_transitionSpeed, _transitionSpeed);
            _currentAngle += step;
            servo.write(_currentAngle);
        }
    }

    void setTargetAngle(float angle) {
        _targetAngle = constrain(angle, _minAngle, _maxAngle);
    }

    float getCurrentAngle() const { return _currentAngle; }
};
```

### 3. I2CSensor Subsystem

**Purpose**: Interface with I²C sensors (e.g., BME280 temperature/humidity/pressure)

**I²C Fundamentals**:
- Two-wire protocol: SDA (data), SCL (clock)
- Master-slave architecture (ESP32 is master)
- 7-bit addressing (device addresses like 0x76)
- Requires pull-up resistors (typically 4.7kΩ)

**Example Implementation**:
```cpp
#include <Wire.h>

class I2CSensor : public RobotSubsystem {
private:
    uint8_t _address;
    float _temperature;
    float _humidity;
    unsigned long _lastReadTime;
    const unsigned long READ_INTERVAL = 1000;  // 1 second

public:
    I2CSensor(uint8_t address) : _address(address) {}

    bool initialize() override {
        Wire.begin(SDA_PIN, SCL_PIN);
        Wire.setClock(100000);  // 100kHz standard mode

        // Check if device responds
        Wire.beginTransmission(_address);
        if (Wire.endTransmission() == 0) {
            _isInitialized = true;
            return true;
        }
        return false;
    }

    void update() override {
        if (!_isInitialized) return;

        unsigned long now = millis();
        if (now - _lastReadTime >= READ_INTERVAL) {
            readSensorData();
            _lastReadTime = now;
        }
    }

    void readSensorData() {
        // I2C read sequence (example)
        Wire.beginTransmission(_address);
        Wire.write(0xFA);  // Temperature register
        Wire.endTransmission();

        Wire.requestFrom(_address, (uint8_t)2);
        if (Wire.available() >= 2) {
            uint16_t raw = (Wire.read() << 8) | Wire.read();
            _temperature = raw / 100.0;  // Convert to °C
        }
    }

    float getTemperature() const { return _temperature; }
};
```

### 4. Telemetry System

**Purpose**: Collect and serialize system state for transmission

**TelemetryData Structure**:
```cpp
struct TelemetryData {
    unsigned long timestamp;

    // Servo data
    float servoAngle;
    bool servoEnabled;

    // Sensor data
    float temperature;
    float humidity;
    float pressure;

    // Locomotion
    int motorSpeed;
    bool motorDirection;

    // System status
    int batteryVoltage;
    int wifiSignalStrength;
    String operationalMode;
    int errorCode;

    // Convert to JSON string
    String toJson() {
        StaticJsonDocument<512> doc;
        doc["timestamp"] = timestamp;
        doc["servo_angle"] = servoAngle;
        doc["servo_enabled"] = servoEnabled;
        doc["temperature"] = temperature;
        doc["humidity"] = humidity;
        doc["motor_speed"] = motorSpeed;
        doc["mode"] = operationalMode;
        doc["error_code"] = errorCode;

        String output;
        serializeJson(doc, output);
        return output;
    }
};
```

### 5. Wi-Fi and HTTP Server

**Purpose**: Enable wireless communication and REST API

**Wi-Fi Setup**:
```cpp
#include <WiFi.h>
#include <WebServer.h>

const char* WIFI_SSID = "YourSSID";
const char* WIFI_PASSWORD = "YourPassword";

WebServer server(80);

void setupWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}
```

**HTTP Endpoints**:
```cpp
void setupWebServer() {
    // GET /telemetry - Return current telemetry
    server.on("/telemetry", HTTP_GET, []() {
        TelemetryData data = collectTelemetry();
        server.send(200, "application/json", data.toJson());
    });

    // POST /command - Execute command
    server.on("/command", HTTP_POST, []() {
        if (server.hasArg("plain")) {
            String body = server.arg("plain");
            StaticJsonDocument<256> doc;

            if (deserializeJson(doc, body) == DeserializationError::Ok) {
                String cmd = doc["command"];
                executeCommand(cmd, doc);
                server.send(200, "application/json", "{\"status\":\"ok\"}");
            } else {
                server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
            }
        }
    });

    server.begin();
    Serial.println("HTTP server started");
}
```

### 6. Main Loop Structure

**Purpose**: Cooperative multitasking for subsystem updates

```cpp
// Global subsystem instances
ServoArm* armServo;
I2CSensor* bme280;
Locomotion* motors;
TelemetryData telemetry;

void setup() {
    Serial.begin(115200);

    // Initialize subsystems
    armServo = new ServoArm(SERVO_PIN);
    bme280 = new I2CSensor(0x76);
    motors = new Locomotion(MOTOR_PIN_A, MOTOR_PIN_B);

    if (!armServo->initialize()) {
        Serial.println("ERROR: Servo init failed");
    }
    if (!bme280->initialize()) {
        Serial.println("ERROR: Sensor init failed");
    }
    if (!motors->initialize()) {
        Serial.println("ERROR: Motors init failed");
    }

    // Setup networking
    setupWiFi();
    setupWebServer();

    Serial.println("System initialized!");
}

void loop() {
    // Update all subsystems
    armServo->update();
    bme280->update();
    motors->update();

    // Handle web server requests
    server.handleClient();

    // Check for low-power mode trigger
    if (shouldEnterLowPower()) {
        enterLowPowerMode();
    }

    // Small delay to prevent watchdog timeout
    delay(10);
}
```

---

## Communication Protocols

### I²C (Inter-Integrated Circuit)

**When to Use**: Sensors (BME280, IMU, OLED displays)

**Characteristics**:
- 2 wires: SDA (data), SCL (clock)
- Multi-device bus (up to 127 devices)
- Master-slave architecture
- Speeds: 100kHz (standard), 400kHz (fast)

**Wiring**:
```
ESP32          Device
-----          ------
GPIO 21 (SDA) ── SDA (requires 4.7kΩ pull-up to 3.3V)
GPIO 22 (SCL) ── SCL (requires 4.7kΩ pull-up to 3.3V)
GND           ── GND
3.3V          ── VCC (check device voltage!)
```

**Common Issues**:
- Missing pull-up resistors → Unreliable communication
- Wrong address → Device not found (use I2C scanner)
- Voltage mismatch → Use level shifter if needed
- Bus contention → Only one master at a time

### SPI (Serial Peripheral Interface)

**When to Use**: High-speed devices (SD cards, displays, some sensors)

**Characteristics**:
- 4 wires minimum: MOSI, MISO, SCK, CS
- Full-duplex communication
- Much faster than I²C (up to 80MHz on ESP32)
- Separate CS line per device

**Wiring**:
```
ESP32          Device
-----          ------
GPIO 23 (MOSI) ── MOSI (Master Out Slave In)
GPIO 19 (MISO) ── MISO (Master In Slave Out)
GPIO 18 (SCK)  ── SCK  (Clock)
GPIO 5  (CS)   ── CS   (Chip Select, one per device)
GND            ── GND
3.3V           ── VCC
```

**Code Example**:
```cpp
#include <SPI.h>

SPIClass spi(VSPI);  // Use VSPI bus

void setup() {
    spi.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
}

uint8_t readSPIRegister(uint8_t reg) {
    digitalWrite(CS_PIN, LOW);   // Select device
    spi.transfer(reg | 0x80);    // Send read command
    uint8_t value = spi.transfer(0x00);  // Read response
    digitalWrite(CS_PIN, HIGH);  // Deselect device
    return value;
}
```

### PWM (Pulse Width Modulation)

**When to Use**: Servos, motor speed control, LED brightness

**Characteristics**:
- Single wire output per channel
- Duty cycle controls average voltage
- Frequency important (servos: 50Hz, motors: 1-20kHz)

**Servo Control**:
```cpp
// Servo expects 50Hz (20ms period)
// Pulse width: 1ms = 0°, 1.5ms = 90°, 2ms = 180°

Servo servo;
servo.attach(SERVO_PIN);
servo.write(90);  // Library handles PWM timing
```

**Motor Speed Control**:
```cpp
// Higher frequency PWM for smooth motor control
const int MOTOR_PIN = 25;
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 5000;      // 5kHz
const int PWM_RESOLUTION = 8;   // 8-bit (0-255)

void setup() {
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_PIN, PWM_CHANNEL);
}

void setMotorSpeed(int speed) {
    // speed: 0-255
    ledcWrite(PWM_CHANNEL, speed);
}
```

---

## Implementation Guidelines

### Phase-Based Development

Follow the README's 5-phase implementation plan:

1. **Phase 1**: Environment setup, basic GPIO, serial debug
2. **Phase 2**: Subsystems (servo, I²C, SPI, locomotion)
3. **Phase 3**: Wi-Fi, HTTP server, Streamlit UI
4. **Phase 4**: Advanced features (low-power, error handling)
5. **Phase 5**: Documentation, testing, packaging

### Educational Features Integration

When implementing any feature, include:

1. **Extensive Inline Comments**
```cpp
// CONCEPT: PWM duty cycle controls average voltage
// 50% duty = 50% of 3.3V = 1.65V average
// This controls motor speed proportionally
```

2. **Tooltips in UI**
```python
st.slider(
    "Servo Angle",
    0, 180, 90,
    help="Controls servo arm position. Most servos operate 0-180°. "
         "Internally uses PWM: 1ms pulse = 0°, 2ms pulse = 180°"
)
```

3. **Interactive Demos**
```python
# Live visualization of PWM signal
if st.checkbox("Show PWM Signal Visualization"):
    duty_cycle = st.slider("Duty Cycle %", 0, 100, 50)
    plot_pwm_waveform(duty_cycle)
```

4. **Error Guidance**
```cpp
if (!sensor.begin()) {
    Serial.println("ERROR: Sensor not found!");
    Serial.println("TROUBLESHOOTING:");
    Serial.println("  1. Check I2C wiring (SDA/SCL)");
    Serial.println("  2. Verify pull-up resistors (4.7kΩ)");
    Serial.println("  3. Run I2C scanner to find address");
    Serial.println("  4. Check power supply (3.3V)");
}
```

### Memory Management

ESP32 has limited RAM (~520KB). Follow these practices:

```cpp
// ❌ Avoid String class in embedded (heap fragmentation)
String message = "Hello " + name;  // BAD

// ✅ Use char arrays instead
char message[64];
snprintf(message, sizeof(message), "Hello %s", name);  // GOOD

// ❌ Avoid dynamic allocation in loop
void loop() {
    int* data = new int[100];  // BAD - heap fragmentation
    // ...
    delete[] data;
}

// ✅ Use static allocation
int data[100];  // GOOD - stack or global

void loop() {
    // Use data array
}
```

### Timing and Delays

```cpp
// ❌ Never use delay() in production code
void loop() {
    readSensor();
    delay(1000);  // BAD - blocks everything
    updateMotor();
}

// ✅ Use non-blocking timing
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_INTERVAL = 1000;

void loop() {
    unsigned long now = millis();

    if (now - lastSensorRead >= SENSOR_INTERVAL) {
        readSensor();
        lastSensorRead = now;
    }

    updateMotor();  // Runs every loop iteration
}
```

### Error Handling Best Practices

```cpp
// Firmware error codes
enum ErrorCode {
    ERR_NONE = 0,
    ERR_WIFI_CONNECT = 1,
    ERR_SENSOR_INIT = 2,
    ERR_SERVO_ATTACH = 3,
    ERR_LOW_BATTERY = 4,
    ERR_I2C_TIMEOUT = 5
};

class ErrorManager {
private:
    ErrorCode _lastError;
    String _errorMessage;

public:
    void setError(ErrorCode code, String message) {
        _lastError = code;
        _errorMessage = message;
        Serial.println("ERROR [" + String(code) + "]: " + message);
    }

    void clearError() {
        _lastError = ERR_NONE;
        _errorMessage = "";
    }

    ErrorCode getLastError() const { return _lastError; }
    String getErrorMessage() const { return _errorMessage; }
    bool hasError() const { return _lastError != ERR_NONE; }
};
```

---

## Testing Strategy

### Unit Testing (Firmware)

Use PlatformIO's built-in testing framework:

```cpp
// test/test_servo_arm.cpp
#include <unity.h>
#include "ServoArm.h"

void test_servo_angle_limits() {
    ServoArm servo(18, 0, 180);
    servo.initialize();

    servo.setTargetAngle(200);  // Beyond max
    TEST_ASSERT_EQUAL_FLOAT(180, servo.getTargetAngle());

    servo.setTargetAngle(-50);  // Below min
    TEST_ASSERT_EQUAL_FLOAT(0, servo.getTargetAngle());
}

void test_servo_initialization() {
    ServoArm servo(18);
    TEST_ASSERT_TRUE(servo.initialize());
    TEST_ASSERT_TRUE(servo.isReady());
}

void setUp() {
    // Runs before each test
}

void tearDown() {
    // Runs after each test
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_servo_angle_limits);
    RUN_TEST(test_servo_initialization);
    return UNITY_END();
}
```

Run tests:
```bash
pio test
```

### Integration Testing

Test subsystem interactions:

```cpp
// test/test_integration.cpp
void test_telemetry_collection() {
    ServoArm servo(18);
    I2CSensor sensor(0x76);

    servo.initialize();
    sensor.initialize();

    // Update subsystems
    servo.setTargetAngle(90);
    servo.update();
    sensor.update();

    // Collect telemetry
    TelemetryData data;
    data.servoAngle = servo.getCurrentAngle();
    data.temperature = sensor.getTemperature();

    TEST_ASSERT_FLOAT_WITHIN(1.0, 90.0, data.servoAngle);
    TEST_ASSERT_TRUE(data.temperature > -40 && data.temperature < 85);
}
```

### UI Testing

```python
# ui/tests/test_telemetry.py
import unittest
import requests_mock
from robot_console import fetch_telemetry

class TestTelemetryFetch(unittest.TestCase):
    @requests_mock.Mocker()
    def test_successful_fetch(self, mock):
        mock.get(
            'http://192.168.1.100/telemetry',
            json={'servo_angle': 90, 'temperature': 25.5}
        )

        data = fetch_telemetry('192.168.1.100')
        self.assertEqual(data['servo_angle'], 90)
        self.assertEqual(data['temperature'], 25.5)

    @requests_mock.Mocker()
    def test_timeout_handling(self, mock):
        mock.get(
            'http://192.168.1.100/telemetry',
            exc=requests.exceptions.Timeout
        )

        data = fetch_telemetry('192.168.1.100')
        self.assertIsNone(data)
```

---

## Common Tasks

### Task 1: Add a New Subsystem

1. Create header file inheriting from `RobotSubsystem`
2. Implement required methods (initialize, update, isReady, getStatus)
3. Add instance to main.cpp globals
4. Initialize in setup()
5. Call update() in loop()
6. Add telemetry fields to TelemetryData struct

Example:
```cpp
// NewSubsystem.h
class NewSubsystem : public RobotSubsystem {
public:
    bool initialize() override;
    void update() override;
    bool isReady() override;
    String getStatus() override;
};

// main.cpp
NewSubsystem* newSub;

void setup() {
    newSub = new NewSubsystem();
    newSub->initialize();
}

void loop() {
    newSub->update();
}
```

### Task 2: Add a New HTTP Endpoint

```cpp
void setupWebServer() {
    // Existing endpoints...

    // New endpoint: GET /status
    server.on("/status", HTTP_GET, []() {
        StaticJsonDocument<256> doc;
        doc["wifi_rssi"] = WiFi.RSSI();
        doc["uptime"] = millis() / 1000;
        doc["free_heap"] = ESP.getFreeHeap();

        String response;
        serializeJson(doc, response);
        server.send(200, "application/json", response);
    });

    server.begin();
}
```

### Task 3: Add UI Component to Streamlit

```python
# ui/components/status_display.py
import streamlit as st

def render_status_display(esp32_ip):
    st.subheader("📊 System Status")

    try:
        response = requests.get(f"http://{esp32_ip}/status", timeout=3)
        data = response.json()

        col1, col2, col3 = st.columns(3)

        with col1:
            st.metric("Wi-Fi Signal", f"{data['wifi_rssi']} dBm")

        with col2:
            st.metric("Uptime", f"{data['uptime']} seconds")

        with col3:
            st.metric("Free Memory", f"{data['free_heap']} bytes")

    except Exception as e:
        st.error(f"Failed to fetch status: {e}")

# robot_console.py
from components.status_display import render_status_display

def main():
    render_status_display(st.session_state.esp32_ip)
```

### Task 4: Implement Sensor Fusion

```cpp
class SensorFusion {
private:
    float _fusedValue;
    float _weight1, _weight2;

public:
    SensorFusion(float w1 = 0.7, float w2 = 0.3)
        : _weight1(w1), _weight2(w2), _fusedValue(0) {}

    float fuse(float sensor1Value, float sensor2Value) {
        // Simple weighted average fusion
        _fusedValue = (_weight1 * sensor1Value) + (_weight2 * sensor2Value);
        return _fusedValue;
    }

    float getFusedValue() const { return _fusedValue; }
};

// Usage in main loop
SensorFusion tempFusion(0.6, 0.4);

void loop() {
    sensor1->update();
    sensor2->update();

    float temp1 = sensor1->getTemperature();
    float temp2 = sensor2->getTemperature();
    float fusedTemp = tempFusion.fuse(temp1, temp2);

    telemetry.temperature = fusedTemp;
}
```

### Task 5: Add Low-Power Mode

```cpp
#include <esp_sleep.h>

void enterLowPowerMode() {
    Serial.println("Entering low-power mode...");

    // Save state if needed
    saveTelemetryToFlash();

    // Configure wake-up sources
    esp_sleep_enable_timer_wakeup(30 * 1000000);  // Wake after 30 seconds
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 1);  // Wake on GPIO 33 HIGH

    // Enter light sleep (Wi-Fi off, faster wake)
    esp_light_sleep_start();

    // Or deep sleep (complete power down, slow wake)
    // esp_deep_sleep_start();

    Serial.println("Woken from sleep!");
}

bool shouldEnterLowPower() {
    // Example: Enter low-power if idle for 5 minutes
    static unsigned long lastActivity = millis();

    if (hasActivity()) {
        lastActivity = millis();
        return false;
    }

    return (millis() - lastActivity > 5 * 60 * 1000);
}
```

---

## Troubleshooting

### ESP32 Not Connecting to Wi-Fi

**Symptoms**: Wi-Fi.status() != WL_CONNECTED, no IP address

**Causes & Solutions**:

1. **Wrong credentials**
   ```cpp
   // Check SSID and password carefully (case-sensitive)
   const char* WIFI_SSID = "YourNetwork";  // Must match exactly
   const char* WIFI_PASSWORD = "YourPassword";
   ```

2. **5GHz network**
   ```
   ESP32 only supports 2.4GHz Wi-Fi. Ensure router broadcasts 2.4GHz.
   ```

3. **MAC filtering**
   ```
   Check router: Add ESP32 MAC to allowed devices
   Get MAC: Serial.println(WiFi.macAddress());
   ```

4. **Distance/Signal**
   ```cpp
   // Check signal strength
   Serial.print("RSSI: ");
   Serial.println(WiFi.RSSI());  // Should be > -70 dBm
   ```

### I²C Device Not Found

**Symptoms**: Wire.endTransmission() returns non-zero

**Diagnosis**:
```cpp
// I2C Scanner Code
void scanI2C() {
    Serial.println("Scanning I2C bus...");

    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("Device found at 0x");
            Serial.println(addr, HEX);
        }
    }
}
```

**Solutions**:
1. Check wiring (SDA to GPIO 21, SCL to GPIO 22)
2. Add pull-up resistors (4.7kΩ to 3.3V on both SDA and SCL)
3. Verify device address in datasheet
4. Check power supply (correct voltage, sufficient current)

### Servo Not Moving

**Checklist**:
- [x] Servo powered separately (not from ESP32 pin - insufficient current)
- [x] Common ground between ESP32 and servo power
- [x] Signal wire connected to correct GPIO
- [x] servo.attach() returned true
- [x] Angle within 0-180° range

**Test Code**:
```cpp
void testServo() {
    servo.write(0);
    delay(1000);
    servo.write(90);
    delay(1000);
    servo.write(180);
    delay(1000);

    Serial.println("Servo test complete. Did it move?");
}
```

### Streamlit Can't Connect to ESP32

**Network Check**:
```python
import subprocess

def ping_esp32(ip):
    try:
        result = subprocess.run(
            ['ping', '-c', '1', ip],
            capture_output=True,
            timeout=5
        )
        return result.returncode == 0
    except:
        return False

if not ping_esp32(esp32_ip):
    st.error(f"❌ Cannot reach {esp32_ip}. Check network connection.")
```

**Solutions**:
1. Verify ESP32 has IP (check Serial output)
2. Ensure PC and ESP32 on same network
3. Disable firewall temporarily
4. Try ESP32's IP directly in browser: `http://192.168.1.xxx/telemetry`

### Memory Issues (Heap Fragmentation)

**Symptoms**: Random crashes, ESP32 reboots, "Out of memory" errors

**Prevention**:
```cpp
// ❌ Avoid
String buildMessage() {
    String msg = "Temperature: ";
    msg += String(temp);
    msg += ", Humidity: ";
    msg += String(humidity);
    return msg;  // Heap fragmentation
}

// ✅ Use instead
void buildMessage(char* buffer, size_t bufferSize) {
    snprintf(buffer, bufferSize,
             "Temperature: %.1f, Humidity: %.1f",
             temp, humidity);
}

// Monitor free heap
Serial.print("Free heap: ");
Serial.println(ESP.getFreeHeap());
```

### Serial Monitor Shows Garbage

**Baud Rate Mismatch**:
```cpp
// Firmware
Serial.begin(115200);  // Must match PlatformIO setting

// platformio.ini
monitor_speed = 115200  // Must match Serial.begin()
```

**Other Causes**:
- Loose USB connection
- Wrong COM port selected
- ESP32 in bootloader mode (hold BOOT button during reset to exit)

---

## Best Practices Summary

### Firmware Development
1. ✅ Use subsystem abstraction for modularity
2. ✅ Non-blocking timing (millis(), avoid delay())
3. ✅ Static memory allocation when possible
4. ✅ Extensive error checking and logging
5. ✅ Educational comments explaining concepts
6. ✅ Test each component before integration

### UI Development
1. ✅ Graceful error handling with user-friendly messages
2. ✅ Tooltips and help text for all controls
3. ✅ Visual feedback for all actions
4. ✅ Responsive layout (use columns, expanders)
5. ✅ Session state for persistent data
6. ✅ Timeout handling for network requests

### Documentation
1. ✅ Inline code comments explaining "why" not "what"
2. ✅ Function/class docstrings with examples
3. ✅ README with setup instructions
4. ✅ Wiring diagrams for hardware connections
5. ✅ API documentation for HTTP endpoints
6. ✅ Troubleshooting guide for common issues

### Educational Focus
1. ✅ Progressive complexity (simple → advanced)
2. ✅ Interactive demos and visualizations
3. ✅ Concept explanations with real-world context
4. ✅ Encouraging experimentation and iteration
5. ✅ Beginner-friendly error messages
6. ✅ Links to external learning resources

---

## API Reference

### HTTP Endpoints

#### GET /telemetry
Returns current system telemetry as JSON.

**Response**:
```json
{
  "timestamp": 1234567890,
  "servo_angle": 90.0,
  "servo_enabled": true,
  "temperature": 25.5,
  "humidity": 60.2,
  "pressure": 1013.25,
  "motor_speed": 128,
  "motor_direction": true,
  "battery_voltage": 7.4,
  "wifi_rssi": -45,
  "mode": "active",
  "error_code": 0
}
```

#### POST /command
Execute a command on the robot.

**Request Body**:
```json
{
  "command": "set_servo_angle",
  "value": 120
}
```

**Supported Commands**:
- `set_servo_angle`: Set servo target angle (value: 0-180)
- `set_motor_speed`: Set motor speed (value: 0-255)
- `set_mode`: Change operational mode (value: "idle", "active", "low_power")
- `reset_errors`: Clear error state

**Response**:
```json
{
  "status": "ok",
  "message": "Command executed successfully"
}
```

**Error Response**:
```json
{
  "status": "error",
  "message": "Invalid command",
  "error_code": 10
}
```

---

## Glossary

- **PWM**: Pulse Width Modulation - technique for controlling power by varying duty cycle
- **I²C**: Inter-Integrated Circuit - two-wire serial protocol for low-speed devices
- **SPI**: Serial Peripheral Interface - high-speed four-wire serial protocol
- **Duty Cycle**: Percentage of time signal is HIGH in PWM (0-100%)
- **Pull-up Resistor**: Resistor connecting signal to VCC, prevents floating signals
- **Telemetry**: Automated communication of measurements from remote sources
- **Subsystem**: Modular hardware component with dedicated control logic
- **Deep Sleep**: Ultra-low power mode with complete CPU shutdown
- **Light Sleep**: Low power mode with faster wake-up, Wi-Fi off
- **Heap Fragmentation**: Memory inefficiency from repeated allocation/deallocation
- **Watchdog Timer**: Hardware timer that resets system if software hangs

---

## Quick Reference Commands

### PlatformIO CLI
```bash
pio project init --board esp32dev    # Create new project
pio lib install "library-name"       # Install library
pio run                              # Build firmware
pio run -t upload                    # Upload to ESP32
pio device monitor                   # Open serial monitor
pio test                             # Run unit tests
pio run -t clean                     # Clean build files
```

### Git Workflow
```bash
git status                           # Check current status
git add .                            # Stage all changes
git commit -m "message"              # Commit with message
git push -u origin branch-name       # Push to remote
git pull origin branch-name          # Pull latest changes
```

### Streamlit Commands
```bash
streamlit run robot_console.py       # Run UI
streamlit run app.py --server.port 8080  # Custom port
streamlit cache clear                # Clear cached data
```

---

## Additional Resources

### Official Documentation
- [ESP32 Arduino Core](https://docs.espressif.com/projects/arduino-esp32/en/latest/)
- [PlatformIO Docs](https://docs.platformio.org/)
- [Streamlit Docs](https://docs.streamlit.io/)
- [ArduinoJson](https://arduinojson.org/)

### Learning Resources
- [I²C Tutorial](https://learn.sparkfun.com/tutorials/i2c)
- [SPI Tutorial](https://learn.sparkfun.com/tutorials/serial-peripheral-interface-spi)
- [PWM Explained](https://learn.sparkfun.com/tutorials/pulse-width-modulation)
- [ESP32 Deep Dive](https://randomnerdtutorials.com/getting-started-with-esp32/)

### Hardware Datasheets
- [ESP32-WROOM-32 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf)
- [BME280 Sensor](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)
- [Servo Motor Basics](https://www.servocity.com/how-do-servo-motors-work)

---

## Contribution Guidelines

When extending this project:

1. **Maintain Educational Focus**: All code should include teaching comments
2. **Follow Coding Standards**: Use naming conventions outlined in this document
3. **Test Thoroughly**: Unit tests for firmware, integration tests for system
4. **Document Changes**: Update this claude.md and relevant docs
5. **Keep It Simple**: Prioritize clarity over cleverness for beginners
6. **Add Examples**: Provide working code snippets for new features

---

## Version History

- **v1.0** (Initial): Base project structure and documentation
- Future versions will be documented here

---

**Document maintained by**: AI Agents & Human Contributors
**Last updated**: 2025-11-18
**Project**: ESP32 Robotics Firmware Apprenticeship
