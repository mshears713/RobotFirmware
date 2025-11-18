# ESP32 Robotics Firmware

Phase 1: Foundations & Environment Setup

## Overview

This is the embedded firmware for the ESP32 Robotics Apprenticeship project. It implements a modular, object-oriented architecture for controlling robotic subsystems including servos, sensors, and motors.

## Project Structure

```
firmware/
├── platformio.ini          # PlatformIO configuration
├── src/
│   └── main.cpp           # Main firmware entry point
├── include/
│   ├── config.h           # Configuration constants
│   ├── RobotSubsystem.h   # Abstract base class for subsystems
│   └── ServoArm.h         # Servo control subsystem
├── lib/                   # External libraries (auto-managed)
└── test/                  # Unit tests
```

## Hardware Requirements

- ESP32 development board (ESP32-WROOM-32 or similar)
- USB cable for programming and power
- Optional:
  - Servo motor (for arm control testing)
  - LED with current-limiting resistor
  - Pushbutton

## Pin Assignments

### Phase 1 Pins

| Component | GPIO Pin | Notes |
|-----------|----------|-------|
| Built-in LED | GPIO 2 | Most ESP32 dev boards |
| Status LED | GPIO 13 | External LED (optional) |
| Button Input | GPIO 0 | BOOT button (built-in) |
| Servo Arm | GPIO 18 | PWM output |

See `include/config.h` for complete pin configuration.

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

After mastering Phase 1, proceed to:

**Phase 2: Core Firmware Features**
- [ ] I2C sensor integration (BME280)
- [ ] SPI device communication
- [ ] PWM motor control
- [ ] Telemetry data structure

**Phase 3: Wi-Fi & Telemetry**
- [ ] Wi-Fi connectivity
- [ ] HTTP server for telemetry
- [ ] JSON serialization
- [ ] Streamlit UI integration

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
