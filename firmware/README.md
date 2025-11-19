# ESP32 Robotics Firmware - Phase 4 Complete

Phase 3: Wi-Fi Telemetry & Streamlit UI Integration

## Overview

This is the embedded firmware for the ESP32 Robotics Apprenticeship project. It implements a modular, object-oriented architecture for controlling robotic subsystems including servos, environmental sensors (I2C), SPI devices, and DC motors for locomotion. The firmware now includes Wi-Fi connectivity, an HTTP REST API server for telemetry streaming, and command execution via JSON over the network.

## Project Structure

```
firmware/
├── platformio.ini          # PlatformIO configuration
├── src/
│   └── main.cpp           # Main firmware entry point (Phase 3 integrated)
├── include/
│   ├── config.h           # Configuration constants
│   ├── RobotSubsystem.h   # Abstract base class for subsystems
│   ├── ServoArm.h         # Servo control subsystem (Phase 1)
│   ├── I2CSensor.h        # I2C environmental sensor (Phase 2)
│   ├── SPIDevice.h        # SPI device interface (Phase 2)
│   ├── Locomotion.h       # Motor control for movement (Phase 2)
│   ├── TelemetryData.h    # Telemetry data structure & JSON (Phase 2)
│   ├── WiFiManager.h      # Wi-Fi connectivity management (Phase 3)
│   ├── RobotWebServer.h   # HTTP REST API server (Phase 3)
│   └── CommandProcessor.h # Command parsing and execution (Phase 3)
├── lib/                   # External libraries (auto-managed)
└── test/                  # Unit tests
    ├── test_servo_arm.cpp
    └── test_locomotion.cpp
```

## Hardware Requirements

### Essential
- ESP32 development board (ESP32-WROOM-32 or similar)
- USB cable for programming and power

### Phase 1 Components
- Servo motor (standard hobby servo, 5V)
- LED with current-limiting resistor (220Ω-1kΩ)
- Pushbutton (built-in BOOT button works)

### Phase 2 Components
- BME280 sensor (I2C environmental sensor)
  - Temperature, humidity, pressure measurement
  - Requires 4.7kΩ pull-up resistors on SDA and SCL
- DC motors (2x) with motor driver
  - Motor driver: L298N, DRV8833, TB6612FNG, or similar
  - Separate power supply for motors (6-12V, NOT from ESP32!)
- SPI device (optional - for testing SPI communication)

## Pin Assignments

### Phase 1 Pins

| Component | GPIO Pin | Notes |
|-----------|----------|-------|
| Built-in LED | GPIO 2 | Most ESP32 dev boards |
| Status LED | GPIO 13 | External LED (optional) |
| Button Input | GPIO 0 | BOOT button (built-in) |
| Servo Arm | GPIO 18 | PWM output |

### Phase 2 Pins

| Component | GPIO Pin | Notes |
|-----------|----------|-------|
| I2C SDA | GPIO 21 | Data line (requires 4.7kΩ pull-up) |
| I2C SCL | GPIO 22 | Clock line (requires 4.7kΩ pull-up) |
| SPI MOSI | GPIO 23 | Master Out Slave In |
| SPI MISO | GPIO 19 | Master In Slave Out |
| SPI SCK | GPIO 18 | Serial Clock |
| SPI CS | GPIO 5 | Chip Select |
| Motor A PWM | GPIO 25 | Speed control |
| Motor A DIR | GPIO 26 | Direction control |
| Motor B PWM | GPIO 27 | Speed control |
| Motor B DIR | GPIO 14 | Direction control |

See `include/config.h` for complete pin configuration and customization.

## Getting Started

### 1. Install PlatformIO

**VSCode Extension (Recommended):**
1. Open Visual Studio Code
2. Go to Extensions (Ctrl+Shift+X)
3. Search for "PlatformIO IDE"
4. Click Install

**Or use PlatformIO CLI:**
```bash
pip install platformio
```

### 2. Open Project

```bash
cd firmware
```

Then open this folder in VSCode with PlatformIO extension installed.

### 3. Build Firmware

**Using VSCode:**
- Click the checkmark (✓) icon in the bottom toolbar
- Or press: Ctrl+Alt+B

**Using CLI:**
```bash
pio run
```

### 4. Upload to ESP32

**Using VSCode:**
- Connect ESP32 via USB
- Click the arrow (→) icon in the bottom toolbar
- Or press: Ctrl+Alt+U

**Using CLI:**
```bash
pio run -t upload
```

### 5. Monitor Serial Output

**Using VSCode:**
- Click the plug icon in the bottom toolbar
- Or press: Ctrl+Alt+S

**Using CLI:**
```bash
pio device monitor
```

Press Ctrl+C to exit serial monitor.

## Configuration

Edit `include/config.h` to customize:

- **GPIO Pin Assignments:** Change pins for LEDs, servos, sensors
- **Timing Constants:** Adjust blink rates, update intervals
- **Servo Parameters:** Set angle limits, transition speed
- **Wi-Fi Credentials:** (Phase 3) SSID and password
- **Debug Settings:** Enable/disable verbose logging

Example:
```cpp
#define LED_BLINK_INTERVAL 500     // Change to 250 for faster blink
#define SERVO_TRANSITION_SPEED 5.0 // Increase for faster servo motion
```

## Features Implemented (Phase 1)

✅ **Development Environment**
- PlatformIO project configuration
- ESP32 board support
- Library dependency management

✅ **Basic GPIO Control**
- LED blinking (non-blocking)
- Digital output control
- Status indicators

✅ **Input Handling**
- Button reading with debouncing
- Pull-up resistor configuration
- Event detection

✅ **Serial Debugging**
- 115200 baud communication
- Periodic status reporting
- System diagnostics

✅ **Subsystem Architecture**
- Abstract `RobotSubsystem` base class
- `ServoArm` implementation
- Polymorphic subsystem management

✅ **Servo Control**
- PWM-based position control
- Smooth angle transitions
- Safety limits (0-180°)

✅ **Main Loop Structure**
- Non-blocking timing with `millis()`
- Cooperative multitasking
- Periodic update cycle

## Features Implemented (Phase 2)

✅ **I2C Communication**
- BME280 environmental sensor integration
- I2C bus scanning and device detection
- Temperature, humidity, pressure readings
- Altitude calculation from barometric pressure
- Error counting and reliability tracking

✅ **SPI Communication**
- Generic SPI device interface
- Configurable clock speed and mode
- Register read/write operations
- Burst read/write support
- Transaction management

✅ **Motor Control (Locomotion)**
- Dual DC motor PWM control
- H-bridge direction control
- Differential drive implementation
- High-level movement commands (forward, backward, turn, rotate)
- Motor enable/disable for safety
- Speed limiting and validation

✅ **Telemetry System**
- Comprehensive system state structure
- JSON serialization with ArduinoJson
- Real-time data collection from all subsystems
- Structured data format for UI consumption
- Pretty-print support for debugging

✅ **Demo Modes**
- Interactive button-controlled demonstrations
- 4 demo modes showcasing different subsystems
- State machine implementation
- Automated testing sequences

## Features Implemented (Phase 3)

✅ **Wi-Fi Connectivity**
- Station mode Wi-Fi client
- Auto-reconnection with configurable retry logic
- DHCP IP address assignment
- RSSI (signal strength) monitoring
- Connection status tracking

✅ **HTTP REST API Server**
- Web server on port 80 for telemetry and control
- CORS support for cross-origin requests
- JSON-based request/response format
- Callback-based architecture for flexible integration

✅ **API Endpoints**
- `GET /` - HTML info page with robot status
- `GET /telemetry` - Real-time telemetry data (JSON)
- `POST /command` - Execute robot commands (JSON)
- `GET /status` - Server status and uptime

✅ **Command Processing**
- JSON command parser with validation
- Supported commands:
  - `set_servo_angle` - Control servo position
  - `set_motor_speed` - Individual motor control
  - `move_forward` / `move_backward` - Movement commands
  - `turn_left` / `turn_right` - Turning maneuvers
  - `rotate` - In-place rotation
  - `stop_motors` - Emergency stop
  - `enable_motors` / `disable_motors` - Safety control
- Command error tracking and reporting

✅ **Network Integration**
- Seamless integration with existing subsystems
- Non-blocking network operations
- Telemetry streaming at configurable intervals
- Network status in debug output

## Wi-Fi Setup

### 1. Configure Credentials

Edit `include/config.h` to set your Wi-Fi network:

```cpp
#define WIFI_SSID "YourWiFiSSID"
#define WIFI_PASSWORD "YourWiFiPassword"
```

### 2. Upload and Connect

After uploading the firmware, the ESP32 will:
1. Attempt to connect to the configured network
2. Display IP address in serial output
3. Start HTTP server on port 80

### 3. Find Your Robot's IP Address

Watch the serial monitor for:
```
[WiFiManager] ✓ Connected to YourWiFiSSID
[WiFiManager] IP Address: 192.168.1.XXX
[RobotWebServer] ✓ Server started on port 80
```

## API Documentation

### GET /telemetry

Returns complete robot state in JSON format.

**Response:**
```json
{
  "timestamp": 123456,
  "free_heap": 234567,
  "mode": "active",
  "servo": {
    "current_angle": 90.0,
    "target_angle": 90.0,
    "enabled": true,
    "at_target": true
  },
  "environment": {
    "temperature": 25.3,
    "humidity": 45.2,
    "pressure": 1013.2,
    "altitude": 123.4,
    "read_count": 100,
    "error_count": 0
  },
  "motors": {
    "motor_a_speed": 0,
    "motor_b_speed": 0,
    "motor_a_direction": "forward",
    "motor_b_direction": "forward",
    "enabled": false
  },
  "status": {
    "error_code": 0,
    "error_message": "",
    "subsystems_ready": 5,
    "subsystems_total": 5
  }
}
```

### POST /command

Execute a robot command.

**Request Body (JSON):**

Move servo:
```json
{
  "command": "set_servo_angle",
  "value": 90
}
```

Control motor:
```json
{
  "command": "set_motor_speed",
  "motor": "A",
  "speed": 150,
  "forward": true
}
```

Move forward:
```json
{
  "command": "move_forward",
  "speed": 128
}
```

Turn left:
```json
{
  "command": "turn_left",
  "speed": 120,
  "turn_rate": 64
}
```

Rotate in place:
```json
{
  "command": "rotate",
  "speed": 128,
  "clockwise": true
}
```

Stop motors:
```json
{
  "command": "stop_motors"
}
```

**Response:**
```json
{
  "success": true,
  "message": "Command executed successfully"
}
```

### GET /status

Returns web server status.

**Response:**
```json
{
  "server": "ESP32 Robot API",
  "uptime": 123456,
  "version": "1.0"
}
```

## Usage Examples

### Basic LED Control

The firmware automatically blinks the built-in LED every 500ms. To change the blink rate:

```cpp
// In config.h
#define LED_BLINK_INTERVAL 250  // Blink twice as fast
```

### Servo Control

Move servo to specific angle:

```cpp
servoArm->setTargetAngle(90);  // Move to center (smooth transition)
```

Immediate movement (no transition):

```cpp
servoArm->setAngleImmediate(45);  // Jump to 45 degrees instantly
```

Check if servo reached target:

```cpp
if (servoArm->hasReachedTarget()) {
    Serial.println("Servo arrived at target position!");
}
```

### Button Input

The BOOT button (GPIO 0) is configured to trigger servo movements. Press the button to move the servo to a random angle.

Modify button behavior in `main.cpp`:

```cpp
if (currentButtonState == LOW) {
    // Button pressed - your code here
    servoArm->setTargetAngle(180);
}
```

## Testing

### Manual Testing

1. Upload firmware to ESP32
2. Open serial monitor (115200 baud)
3. Observe startup messages
4. Watch LED blinking
5. Press BOOT button to test input and servo

### Expected Serial Output

```
========================================
  ESP32 Robotics Firmware
  WALL-E & EVE Apprenticeship
  Phase 1: Foundations
========================================

[INIT] Serial communication initialized
[INIT] Baud rate: 115200
[INIT] Configuring GPIO pins...
[INIT]   - LED pins configured (2, 13)
[INIT]   - Button pin configured (0) with pull-up
[INIT] ✓ GPIO configuration complete
[INIT] Initializing subsystems...
[ServoArm] Initializing on pin 18
[ServoArm] ✓ Initialization successful
...
```

### Unit Tests (Future)

Unit tests will be added in the `test/` directory:

```bash
pio test
```

## Troubleshooting

### Upload Failed

**Problem:** Cannot upload firmware to ESP32

**Solutions:**
1. Check USB cable (must support data, not just power)
2. Install USB-to-Serial drivers (CP2102, CH340)
3. Hold BOOT button during upload
4. Try different USB port
5. Check that correct board is selected in `platformio.ini`

### Serial Monitor Shows Garbage

**Problem:** Unreadable characters in serial output

**Solutions:**
1. Verify baud rate is 115200 (must match `Serial.begin()`)
2. Check `monitor_speed = 115200` in `platformio.ini`
3. Try different USB cable
4. Disconnect and reconnect

### Servo Not Moving

**Problem:** Servo doesn't respond to commands

**Solutions:**
1. Check servo power supply (5V, separate from ESP32)
2. Verify common ground between ESP32 and servo power
3. Confirm signal wire connected to GPIO 18
4. Check servo is functional (test with another controller)
5. Look for initialization errors in serial output

### ESP32 Keeps Resetting

**Problem:** Board resets continuously

**Solutions:**
1. Check power supply (sufficient current, stable voltage)
2. Look for short circuits in wiring
3. Reduce load (disconnect peripherals temporarily)
4. Check for infinite loops or watchdog timeouts in code

### Compilation Errors

**Problem:** Build fails with errors

**Solutions:**
1. Clean build files: `pio run -t clean`
2. Delete `.pio` directory and rebuild
3. Update PlatformIO: `pio upgrade`
4. Check for typos in code
5. Verify all `#include` statements are correct

## Code Style

The firmware follows these conventions:

- **Classes:** PascalCase (`ServoArm`, `RobotSubsystem`)
- **Functions:** camelCase (`updateLED`, `setupSerial`)
- **Constants:** UPPER_SNAKE_CASE (`LED_BLINK_INTERVAL`)
- **Variables:** camelCase (`ledState`, `currentAngle`)
- **Private members:** prefix with `_` (`_isInitialized`, `_pin`)

## Educational Features

This firmware is designed for learning:

📚 **Extensive Comments:** Every function and concept explained
🎓 **EDUCATIONAL NOTE blocks:** Deep dives into key concepts
💡 **Best Practices:** Non-blocking timing, proper error handling
🔬 **Experiments:** Suggestions for hands-on learning
📊 **Debug Output:** Comprehensive system status reporting

## Next Steps

**Completed Phases:**
- ✅ Phase 1: Foundations & Environment Setup
- ✅ Phase 2: Core Firmware Features & Subsystem Control
- ✅ Phase 3: Wi-Fi Telemetry & Streamlit UI Integration

**Using the Streamlit UI:**

The Streamlit web interface (located in `../ui/`) provides a complete graphical interface for:
- Real-time telemetry monitoring
- Robot control (servo, motors, movement)
- Telemetry history and charts
- Command history tracking
- SQLite database logging

See `../ui/README.md` for setup and usage instructions.

**Future Enhancements (Beyond Phase 3):**
- [ ] Phase 4: Advanced Sensors & Perception
- [ ] Phase 5: Autonomous Navigation
- [ ] Phase 6: Computer Vision Integration
- [ ] Phase 7: Advanced AI Features

## Resources

- [PlatformIO Documentation](https://docs.platformio.org/)
- [ESP32 Arduino Core](https://docs.espressif.com/projects/arduino-esp32/)
- [Servo Library Reference](https://www.arduino.cc/reference/en/libraries/servo/)
- [Project Main README](../README.md)
- [Claude AI Agent Guide](../claude.md)

## License

Educational project for learning embedded systems development.

## Contributors

This is an apprenticeship project designed for hands-on learning.

---

**Happy Coding! 🤖**

*Remember: The best way to learn is to experiment. Don't be afraid to break things!*
