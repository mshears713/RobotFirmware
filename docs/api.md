# ESP32 Robotics Firmware - HTTP REST API Documentation

**Complete API Reference - All Phases**

---

## Base URL

```
http://<ESP32_IP_ADDRESS>/
```

Default IP addresses:
- Station mode: Assigned by DHCP (check Serial output)
- AP mode: `192.168.4.1`

---

## Table of Contents

1. [Telemetry Endpoints](#telemetry-endpoints)
2. [Command Endpoints](#command-endpoints)
3. [Status Endpoints](#status-endpoints)
4. [Command Reference](#command-reference)
5. [Error Responses](#error-responses)
6. [Examples](#examples)

---

## Telemetry Endpoints

### GET /telemetry

Returns current robot telemetry data as JSON.

**Request:**
```http
GET /telemetry HTTP/1.1
Host: <ESP32_IP>
```

**Response:**
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
    "altitude": 100.5,
    "read_count": 1234,
    "error_count": 0
  },

  "motors": {
    "motor_a_speed": 150,
    "motor_b_speed": 150,
    "motor_a_direction": "forward",
    "motor_b_direction": "forward",
    "enabled": true
  },

  "spi": {
    "transfer_count": 567,
    "test_value": 42
  },

  "status": {
    "error_code": 0,
    "error_message": "",
    "subsystems_ready": 7,
    "subsystems_total": 7
  },

  "power": {
    "mode": "ACTIVE",
    "time_since_activity": 1234,
    "estimated_current": 160.0,
    "last_wake_reason": "Power-on/Reset"
  },

  "fusion": {
    "fused_temperature": 25.3,
    "confidence": 1.0,
    "sample_count": 100
  },

  "errors": {
    "log_count": 0,
    "warning_count": 0,
    "recovery_count": 0,
    "system_health": 100.0,
    "current_severity": "INFO"
  }
}
```

**Status Codes:**
- `200 OK` - Success
- `500 Internal Server Error` - Failed to generate telemetry

**Update Rate:** Updated every 100ms in firmware
**Recommended Poll Rate:** 1-10 Hz (avoid overloading ESP32)

---

## Command Endpoints

### POST /command

Execute a command on the robot.

**Request:**
```http
POST /command HTTP/1.1
Host: <ESP32_IP>
Content-Type: application/json

{
  "command": "move_forward",
  "speed": 150
}
```

**Response (Success):**
```json
{
  "status": "ok",
  "message": "Command executed successfully"
}
```

**Response (Error):**
```json
{
  "status": "error",
  "message": "Invalid command parameter",
  "error_code": 500
}
```

**Status Codes:**
- `200 OK` - Command executed successfully
- `400 Bad Request` - Invalid JSON or missing parameters
- `500 Internal Server Error` - Command execution failed

---

## Status Endpoints

### GET /status

Health check endpoint.

**Request:**
```http
GET /status HTTP/1.1
Host: <ESP32_IP>
```

**Response:**
```json
{
  "status": "online",
  "uptime": 123456,
  "version": "Phase 4",
  "wifi_rssi": -45
}
```

### GET /

Information page (returns HTML).

**Request:**
```http
GET / HTTP/1.1
Host: <ESP32_IP>
```

**Response:**
```html
<html>
<body>
<h1>ESP32 Robot Systems</h1>
<p>Firmware: Phase 4 - Advanced Features</p>
<p>Status: Online</p>
<p>Available endpoints:</p>
<ul>
  <li>GET /telemetry</li>
  <li>POST /command</li>
  <li>GET /status</li>
</ul>
</body>
</html>
```

---

## Command Reference

### Servo Control Commands

#### set_servo_angle
Set servo target angle.

**Parameters:**
- `value` (float, required): Target angle in degrees (0-180)

**Example:**
```json
{
  "command": "set_servo_angle",
  "value": 90
}
```

**Validation:**
- Angle is clamped to configured min/max range
- Smooth transition applied automatically

---

### Motor Control Commands

#### set_motor_speed
Set individual motor speed and direction.

**Parameters:**
- `motor` (string, required): "A", "B", or "BOTH"
- `speed` (integer, required): 0-255
- `forward` (boolean, optional): true (forward) or false (reverse), default true

**Example:**
```json
{
  "command": "set_motor_speed",
  "motor": "A",
  "speed": 200,
  "forward": true
}
```

#### move_forward
Move robot forward at specified speed.

**Parameters:**
- `speed` (integer, optional): 0-255, default 128

**Example:**
```json
{
  "command": "move_forward",
  "speed": 150
}
```

#### move_backward
Move robot backward at specified speed.

**Parameters:**
- `speed` (integer, optional): 0-255, default 128

**Example:**
```json
{
  "command": "move_backward",
  "speed": 150
}
```

#### turn_left
Turn robot left (differential drive).

**Parameters:**
- `speed` (integer, optional): Forward speed 0-255, default 120
- `turn_rate` (integer, optional): Inside wheel speed reduction 0-255, default 64

**Example:**
```json
{
  "command": "turn_left",
  "speed": 120,
  "turn_rate": 64
}
```

#### turn_right
Turn robot right (differential drive).

**Parameters:**
- `speed` (integer, optional): Forward speed 0-255, default 120
- `turn_rate` (integer, optional): Inside wheel speed reduction 0-255, default 64

**Example:**
```json
{
  "command": "turn_right",
  "speed": 120,
  "turn_rate": 64
}
```

#### rotate
Rotate robot in place.

**Parameters:**
- `speed` (integer, optional): Rotation speed 0-255, default 128
- `clockwise` (boolean, optional): true (CW) or false (CCW), default true

**Example:**
```json
{
  "command": "rotate",
  "speed": 100,
  "clockwise": false
}
```

#### stop_motors
Stop all motors immediately (emergency stop).

**Parameters:** None

**Example:**
```json
{
  "command": "stop_motors"
}
```

#### enable_motors
Enable motor control.

**Parameters:** None

**Example:**
```json
{
  "command": "enable_motors"
}
```

#### disable_motors
Disable motor control (safety).

**Parameters:** None

**Example:**
```json
{
  "command": "disable_motors"
}
```

---

### Power Management Commands (Phase 4)

#### set_power_mode
Change power management mode.

**Parameters:**
- `mode` (string, required): "active", "idle", "low_power", "light_sleep", or "deep_sleep"

**Example:**
```json
{
  "command": "set_power_mode",
  "mode": "low_power"
}
```

**Power Modes:**
- `active` - Full operation (~160mA)
- `idle` - Reduced activity (~100mA)
- `low_power` - Wi-Fi modem sleep (~30mA)
- `light_sleep` - CPU paused, wake via timer/GPIO (~0.8mA)
- `deep_sleep` - Complete shutdown, system restart on wake (~10µA)

**Warning:** `light_sleep` and `deep_sleep` will disconnect Wi-Fi. Configure wake sources before using.

#### set_operational_mode
Change robot operational mode.

**Parameters:**
- `mode` (string, required): "idle", "active", "maintenance", or "error"

**Example:**
```json
{
  "command": "set_operational_mode",
  "mode": "active"
}
```

**Notes:**
- Resets inactivity timer
- Updates telemetry.mode field
- Does not affect power mode directly

---

### Error Management Commands (Phase 4)

#### clear_errors
Clear all logged errors.

**Parameters:** None

**Example:**
```json
{
  "command": "clear_errors"
}
```

**Effect:**
- Clears error log history
- Resets error counters
- Sets system health to 100%

#### perform_health_check
Run comprehensive system health check.

**Parameters:** None

**Example:**
```json
{
  "command": "perform_health_check"
}
```

**Checks:**
- Memory usage
- Wi-Fi connection status
- Subsystem health
- Error rates

**Output:** Results logged to Serial and error manager

---

## Error Responses

### Command Execution Errors

**Missing Command Field:**
```json
{
  "status": "error",
  "message": "Missing 'command' field",
  "error_code": 501
}
```

**Unknown Command:**
```json
{
  "status": "error",
  "message": "Unknown command: invalid_cmd",
  "error_code": 500
}
```

**Missing Parameter:**
```json
{
  "status": "error",
  "message": "Missing 'value' parameter",
  "error_code": 501
}
```

**Subsystem Not Available:**
```json
{
  "status": "error",
  "message": "Locomotion not available",
  "error_code": 502
}
```

### HTTP Errors

**Malformed JSON:**
```
HTTP/1.1 400 Bad Request
{
  "error": "Invalid JSON"
}
```

**Method Not Allowed:**
```
HTTP/1.1 405 Method Not Allowed
```

---

## Examples

### cURL Examples

**Fetch Telemetry:**
```bash
curl http://192.168.1.100/telemetry
```

**Move Forward:**
```bash
curl -X POST http://192.168.1.100/command \
  -H "Content-Type: application/json" \
  -d '{"command":"move_forward","speed":150}'
```

**Set Servo Angle:**
```bash
curl -X POST http://192.168.1.100/command \
  -H "Content-Type: application/json" \
  -d '{"command":"set_servo_angle","value":45}'
```

**Enable Low-Power Mode:**
```bash
curl -X POST http://192.168.1.100/command \
  -H "Content-Type: application/json" \
  -d '{"command":"set_power_mode","mode":"low_power"}'
```

**Clear Errors:**
```bash
curl -X POST http://192.168.1.100/command \
  -H "Content-Type: application/json" \
  -d '{"command":"clear_errors"}'
```

### Python Examples

**Fetch Telemetry:**
```python
import requests

resp = requests.get('http://192.168.1.100/telemetry', timeout=5)
telemetry = resp.json()

print(f"Temperature: {telemetry['environment']['temperature']}°C")
print(f"Servo Angle: {telemetry['servo']['current_angle']}°")
```

**Send Command:**
```python
import requests

command = {
    "command": "move_forward",
    "speed": 150
}

resp = requests.post(
    'http://192.168.1.100/command',
    json=command,
    timeout=5
)

result = resp.json()
print(result['status'])  # "ok" or "error"
```

### JavaScript Example

```javascript
// Fetch telemetry
fetch('http://192.168.1.100/telemetry')
  .then(response => response.json())
  .then(data => {
    console.log('Temperature:', data.environment.temperature);
    console.log('Servo Angle:', data.servo.current_angle);
  });

// Send command
fetch('http://192.168.1.100/command', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json'
  },
  body: JSON.stringify({
    command: 'set_servo_angle',
    value: 90
  })
})
  .then(response => response.json())
  .then(data => console.log(data.status));
```

---

## Rate Limiting

**Recommendations:**
- Telemetry polling: 1-10 Hz maximum
- Command sending: No more than 10 commands/second
- Burst protection: ESP32 may drop requests if overwhelmed

**Best Practices:**
- Poll telemetry at consistent intervals
- Wait for command response before sending next
- Implement exponential backoff on errors
- Monitor ESP32 free heap via telemetry

---

## CORS Support

The API includes CORS headers for web browser access:

```
Access-Control-Allow-Origin: *
Access-Control-Allow-Methods: GET, POST, OPTIONS
Access-Control-Allow-Headers: Content-Type
```

This allows direct API access from web pages without proxy.

---

## Security Considerations

**Current Implementation:**
- No authentication (educational project)
- No encryption (HTTP, not HTTPS)
- Open CORS (any origin allowed)

**Production Recommendations:**
- Add API key authentication
- Implement HTTPS (ESP32 supports TLS)
- Restrict CORS to specific origins
- Add rate limiting
- Implement command whitelisting
- Add request signing

---

## Troubleshooting

### Can't Connect to API

1. **Verify ESP32 IP:** Check Serial monitor output during boot
2. **Ping ESP32:** `ping <IP_ADDRESS>`
3. **Check Wi-Fi:** Ensure ESP32 and client on same network
4. **Firewall:** Temporarily disable to test
5. **Port 80:** Ensure not blocked by antivirus

### Commands Not Executing

1. **Check telemetry.error_code:** May indicate subsystem failure
2. **Serial monitor:** View command execution logs
3. **Subsystem status:** Verify subsystem.isReady() via /telemetry
4. **JSON format:** Validate JSON syntax
5. **Content-Type:** Ensure "application/json" header

### Slow Response Times

1. **Network congestion:** Reduce polling rate
2. **ESP32 overload:** Check free_heap in telemetry
3. **Wi-Fi signal:** Check wifi_rssi (should be > -70dBm)
4. **Too many clients:** Limit to 1-2 simultaneous connections

---

## Version History

- **Phase 1:** Basic subsystems (servo, GPIO)
- **Phase 2:** Core features (I2C, SPI, locomotion, telemetry)
- **Phase 3:** Wi-Fi connectivity, HTTP API, Streamlit UI
- **Phase 4:** Power management, sensor fusion, error handling

---

## API Testing Tools

**Recommended Tools:**
- **Postman** - GUI API testing
- **cURL** - Command-line testing
- **Hoppscotch** - Web-based API client
- **Python requests** - Scripted testing
- **Streamlit UI** - Included in project

---

For implementation details, see:
- `firmware/include/RobotWebServer.h`
- `firmware/include/CommandProcessor.h`
- `firmware/include/TelemetryData.h`
