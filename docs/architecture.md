# ESP32 Robotics Firmware Architecture

**Phase 5: Complete System Architecture Documentation**

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Firmware Architecture](#firmware-architecture)
3. [Subsystem Design](#subsystem-design)
4. [Communication Flow](#communication-flow)
5. [Power Management](#power-management)
6. [Error Handling](#error-handling)
7. [Data Structures](#data-structures)

---

## System Overview

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     STREAMLIT UI (Desktop)                      │
│  ┌──────────────┐  ┌──────────────┐  ┌───────────────────────┐ │
│  │  Telemetry   │  │   Command    │  │   Error/Health        │ │
│  │  Display     │  │   Console    │  │   Monitoring          │ │
│  └──────────────┘  └──────────────┘  └───────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                              │
                         Wi-Fi (HTTP/JSON)
                              │
┌─────────────────────────────────────────────────────────────────┐
│                      ESP32 FIRMWARE                             │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                    Main Loop                              │  │
│  │  ┌─────────┐  ┌──────────┐  ┌───────────┐  ┌─────────┐  │  │
│  │  │ Servo   │  │  Sensors │  │  Motors   │  │  WiFi   │  │  │
│  │  │ Update  │  │  Update  │  │  Update   │  │  Update │  │  │
│  │  └─────────┘  └──────────┘  └───────────┘  └─────────┘  │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │              HTTP REST API Server                        │  │
│  │  GET /telemetry  │  POST /command  │  GET /status       │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │              Advanced Subsystems (Phase 4)               │  │
│  │  PowerManager  │  SensorFusion  │  ErrorManager         │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                              │
                    Hardware Interfaces
                              │
┌─────────────────────────────────────────────────────────────────┐
│                        HARDWARE LAYER                           │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────┐   │
│  │  Servo   │  │  BME280  │  │ DC Motors│  │  SPI Device  │   │
│  │  (PWM)   │  │  (I2C)   │  │  (PWM)   │  │    (SPI)     │   │
│  └──────────┘  └──────────┘  └──────────┘  └──────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

### System Components

1. **ESP32 Firmware** - Core embedded software
2. **Streamlit UI** - Desktop monitoring and control interface
3. **Hardware Subsystems** - Physical robot components
4. **Communication Layer** - Wi-Fi and HTTP/JSON protocol

---

## Firmware Architecture

### Design Principles

1. **Modularity** - Each subsystem is independent and replaceable
2. **Abstraction** - Common interface via RobotSubsystem base class
3. **Real-time** - Non-blocking cooperative multitasking
4. **Robustness** - Comprehensive error handling and recovery
5. **Extensibility** - Easy to add new subsystems

### Component Hierarchy

```
RobotSubsystem (Abstract Base Class)
    ├── ServoArm
    ├── I2CSensor
    ├── SPIDevice
    ├── Locomotion
    ├── WiFiManager
    ├── RobotWebServer
    ├── PowerManager
    ├── SensorFusion
    └── ErrorManager

TelemetryData (Data Structure)
    └── JSON Serialization

CommandProcessor (Command Pattern)
    └── Command Handlers
```

### File Organization

```
firmware/
├── platformio.ini              # Build configuration
├── include/                    # Header files
│   ├── config.h               # System constants
│   ├── RobotSubsystem.h       # Base class
│   ├── ServoArm.h            # Servo control
│   ├── I2CSensor.h           # I2C sensor interface
│   ├── SPIDevice.h           # SPI device interface
│   ├── Locomotion.h          # Motor control
│   ├── TelemetryData.h       # Telemetry structure
│   ├── WiFiManager.h         # Wi-Fi management
│   ├── RobotWebServer.h      # HTTP server
│   ├── CommandProcessor.h    # Command handling
│   ├── PowerManager.h        # Power management
│   ├── SensorFusion.h        # Sensor fusion
│   └── ErrorManager.h        # Error handling
├── src/
│   ├── main.cpp              # Main entry point
│   ├── PowerManager.cpp      # Implementation
│   ├── SensorFusion.cpp      # Implementation
│   └── ErrorManager.cpp      # Implementation
└── test/                      # Unit tests
    ├── test_servo_arm.cpp
    ├── test_locomotion.cpp
    ├── test_power_manager.cpp
    └── test_sensor_fusion.cpp
```

---

## Subsystem Design

### RobotSubsystem Abstract Base Class

Every hardware subsystem inherits from this interface:

```cpp
class RobotSubsystem {
public:
    virtual bool initialize() = 0;    // Setup hardware
    virtual void update() = 0;        // Called each loop
    virtual bool isReady() = 0;       // Health check
    virtual String getStatus() = 0;   // Status string

protected:
    bool _isInitialized = false;
    unsigned long _lastUpdate = 0;
};
```

**Benefits:**
- Uniform interface for all subsystems
- Polymorphic update in main loop
- Easy to add new hardware
- Clear initialization and health checking

### Core Subsystems

#### 1. ServoArm (Phase 1)
**Purpose:** Control servo motors via PWM

**Key Features:**
- Angle range limiting (0-180°)
- Smooth transitions (no jerky motion)
- Position feedback
- Target angle tracking

**Hardware Interface:** GPIO PWM output

#### 2. I2CSensor (Phase 2)
**Purpose:** Read environmental data from BME280

**Key Features:**
- Temperature, humidity, pressure readings
- Altitude estimation
- Error counting and timeout detection
- Non-blocking periodic reads

**Hardware Interface:** I2C bus (SDA, SCL)

#### 3. SPIDevice (Phase 2)
**Purpose:** Generic SPI device communication

**Key Features:**
- Full-duplex SPI transfers
- Chip select management
- Transfer counting for diagnostics

**Hardware Interface:** SPI bus (MOSI, MISO, SCK, CS)

#### 4. Locomotion (Phase 2)
**Purpose:** DC motor control for robot movement

**Key Features:**
- Differential drive (two motors)
- Speed and direction control
- High-level movement commands (forward, backward, turn, rotate)
- Motor enable/disable for safety

**Hardware Interface:** PWM (speed) + GPIO (direction)

#### 5. WiFiManager (Phase 3)
**Purpose:** Wi-Fi connectivity management

**Key Features:**
- Station mode connection
- Auto-reconnect on disconnect
- Signal strength monitoring
- IP address management

**Hardware Interface:** ESP32 Wi-Fi radio

#### 6. RobotWebServer (Phase 3)
**Purpose:** HTTP REST API server

**Key Features:**
- GET /telemetry - Return current telemetry
- POST /command - Execute commands
- GET /status - Health check
- CORS support for web browsers
- JSON request/response

**Hardware Interface:** Software (uses WiFiManager)

### Advanced Subsystems (Phase 4)

#### 7. PowerManager
**Purpose:** Battery life optimization

**Power Modes:**
- ACTIVE - Full operation (~160mA)
- IDLE - Reduced activity (~100mA)
- LOW_POWER - Modem sleep (~30mA)
- LIGHT_SLEEP - CPU paused (~0.8mA)
- DEEP_SLEEP - Full shutdown (~10µA)

**Features:**
- Automatic mode transitions based on inactivity
- Wake-up configuration (timer, GPIO)
- Current draw estimation
- Wi-Fi power save integration

#### 8. SensorFusion
**Purpose:** Improve sensor accuracy

**Algorithms:**
- Weighted Average - Combine multiple sensors
- Complementary Filter - IMU fusion
- Median Filter - Outlier rejection
- Exponential Moving Average - Signal smoothing

**Features:**
- Sensor health monitoring
- Statistics tracking (min, max, average)
- Configurable fusion parameters

#### 9. ErrorManager
**Purpose:** System reliability and diagnostics

**Features:**
- Error severity levels (Info, Warning, Error, Critical)
- Error logging with timestamps
- Automatic recovery strategies
- System health scoring (0-100%)
- JSON error log export

**Recovery Actions:**
- Retry failed operations
- Reset subsystems
- Disable faulty components
- System restart (critical errors)

---

## Communication Flow

### Telemetry Flow (ESP32 → UI)

```
1. Main Loop
   └─> updateTelemetry()
       ├─> Read all subsystem states
       ├─> Populate TelemetryData struct
       └─> Ready for HTTP request

2. Streamlit UI
   └─> HTTP GET /telemetry
       └─> ESP32 WebServer
           └─> provideTelemetry() callback
               └─> telemetry.toJson()
                   └─> Return JSON string

3. UI processes JSON
   ├─> Update displays
   ├─> Log to SQLite
   └─> Update charts
```

### Command Flow (UI → ESP32)

```
1. Streamlit UI
   └─> User clicks command button
       └─> HTTP POST /command
           └─> JSON payload: {"command": "...", ...}

2. ESP32 WebServer
   └─> handleCommand() callback
       └─> CommandProcessor.processCommand()
           ├─> Parse JSON
           ├─> Validate command
           ├─> Route to handler
           └─> Execute on subsystem

3. Subsystem Action
   └─> Update hardware
       └─> Next telemetry includes change
```

### Data Formats

#### Telemetry JSON Example
```json
{
  "timestamp": 123456,
  "free_heap": 280000,
  "mode": "active",
  "servo": {
    "current_angle": 90.5,
    "target_angle": 90.0,
    "enabled": true,
    "at_target": true
  },
  "environment": {
    "temperature": 25.3,
    "humidity": 45.2,
    "pressure": 1013.25,
    "altitude": 100.5
  },
  "motors": {
    "motor_a_speed": 150,
    "motor_b_speed": 150,
    "motor_a_direction": "forward",
    "motor_b_direction": "forward",
    "enabled": true
  },
  "power": {
    "mode": "ACTIVE",
    "time_since_activity": 1234,
    "estimated_current": 160.0
  },
  "errors": {
    "log_count": 0,
    "warning_count": 0,
    "system_health": 100.0
  }
}
```

#### Command JSON Example
```json
{
  "command": "move_forward",
  "speed": 150
}
```

---

## Power Management

### Power Mode State Machine

```
    ┌─────────────┐
    │   ACTIVE    │ <──── User activity
    └──────┬──────┘       Command received
           │ 30s idle
           v
    ┌─────────────┐
    │    IDLE     │
    └──────┬──────┘
           │ 60s idle
           v
    ┌─────────────┐
    │ LOW_POWER   │ ──── Wi-Fi modem sleep
    └──────┬──────┘
           │ 5min idle
           v
    ┌─────────────┐
    │ LIGHT_SLEEP │ ──── Wake via timer/GPIO
    └─────────────┘

    Manual command only:
    ┌─────────────┐
    │ DEEP_SLEEP  │ ──── System restart on wake
    └─────────────┘
```

### Battery Life Estimation

| Mode | Current | Battery Life (2000mAh) |
|------|---------|------------------------|
| ACTIVE | 160mA | ~12.5 hours |
| IDLE | 100mA | ~20 hours |
| LOW_POWER | 30mA | ~66 hours (2.7 days) |
| LIGHT_SLEEP | 0.8mA | ~2500 hours (104 days) |
| DEEP_SLEEP | 0.01mA | ~200,000 hours (22 years) |

---

## Error Handling

### Error Propagation

```
Subsystem Detects Error
    └─> errorManager->reportError(code, severity, message)
        ├─> Log to error history
        ├─> Update telemetry.errorCode
        ├─> Print to Serial
        └─> Auto-recovery attempt (if enabled)
            ├─> Success → Clear error
            └─> Failure → Increment failedRecoveries
```

### Error Codes

| Range | Category | Examples |
|-------|----------|----------|
| 100-199 | Wi-Fi | 100: Connection failed, 101: Disconnected |
| 200-299 | Sensors | 200: I2C not found, 201: I2C comm error |
| 300-399 | Actuators | 300: Servo attach failed, 301: Motor driver error |
| 400-499 | System | 400: Low memory, 402: Low battery |
| 500-599 | Commands | 500: Invalid command, 502: Execution failed |

---

## Data Structures

### TelemetryData

**Size:** ~200 bytes (pre-serialization)
**JSON Size:** ~1000 bytes (serialized)
**Update Rate:** 10 Hz (100ms interval)
**Network Bandwidth:** ~10 KB/s @ 10Hz

**Fields:** 30+ data points including:
- System info (timestamp, heap, mode)
- Servo state (angle, target, status)
- Environment (temp, humidity, pressure)
- Motors (speed, direction, status)
- Power (mode, activity, current estimate)
- Fusion (fused values, confidence)
- Errors (counts, health score)

### Memory Usage

**Total RAM:** ~520 KB (ESP32)

**Firmware Usage:**
- Global variables: ~10 KB
- Subsystem objects: ~5 KB
- Network buffers: ~20 KB
- JSON buffers: ~3 KB
- Stack: ~8 KB

**Available:** ~474 KB (91% free)

**Flash Usage:**
- Firmware code: ~300 KB
- Libraries: ~200 KB
- Total: ~500 KB of 4 MB (12.5% used)

---

## Performance Characteristics

### Timing

| Operation | Duration | Frequency |
|-----------|----------|-----------|
| Main loop iteration | <1ms | Continuous |
| Servo update | <0.1ms | Every loop |
| I2C sensor read | ~5ms | 1 Hz |
| SPI transfer | <1ms | As needed |
| HTTP request handling | 5-20ms | On demand |
| Telemetry serialization | ~2ms | 10 Hz |

### Network Performance

- **Wi-Fi Connection Time:** 2-5 seconds
- **HTTP Response Time:** 10-50ms (local network)
- **Telemetry Fetch Rate:** Up to 20 Hz (limited by network)
- **Command Execution Latency:** <100ms

---

## Extensibility

### Adding a New Subsystem

1. Create header inheriting from `RobotSubsystem`
2. Implement required virtual methods
3. Add instance to `main.cpp` globals
4. Initialize in `setupSubsystems()`
5. Update in `loop()`
6. Add telemetry fields to `TelemetryData`
7. Add commands to `CommandProcessor`

Example:
```cpp
// NewSensor.h
class NewSensor : public RobotSubsystem {
    // Implementation
};

// main.cpp
NewSensor* newSensor = nullptr;

void setupSubsystems() {
    newSensor = new NewSensor();
    newSensor->initialize();
}

void loop() {
    if (newSensor && newSensor->isReady()) {
        newSensor->update();
    }
}
```

### Adding a New Command

```cpp
// CommandProcessor.h
else if (strcmp(command, "new_command") == 0) {
    success = handleNewCommand(commandJson);
}

bool handleNewCommand(const JsonDocument& cmd) {
    // Implementation
    return true;
}
```

---

## Conclusion

This architecture provides:

✅ **Modularity** - Clean separation of concerns
✅ **Scalability** - Easy to add new features
✅ **Reliability** - Comprehensive error handling
✅ **Performance** - Efficient real-time operation
✅ **Maintainability** - Well-documented and organized
✅ **Educational Value** - Clear examples of embedded patterns

The design balances professional embedded practices with beginner-friendly code structure, making it an excellent learning platform for firmware engineering.
