# ESP32 Robotics Firmware - Troubleshooting Guide

**Complete Problem Diagnosis and Solutions**

---

## Table of Contents

1. [Firmware Build & Upload Issues](#firmware-build--upload-issues)
2. [Wi-Fi Connectivity Problems](#wi-fi-connectivity-problems)
3. [HTTP API Issues](#http-api-issues)
4. [Subsystem Failures](#subsystem-failures)
5. [Power & Performance Issues](#power--performance-issues)
6. [Streamlit UI Problems](#streamlit-ui-problems)
7. [Hardware Debugging](#hardware-debugging)
8. [Common Error Codes](#common-error-codes)

---

## Firmware Build & Upload Issues

### Problem: PlatformIO Not Found

**Symptoms:**
- `pio: command not found`
- Build scripts fail immediately

**Solutions:**

```bash
# Install PlatformIO
pip install platformio

# Or install via package manager (Linux)
sudo apt-get install platformio

# Verify installation
pio --version
```

**Alternative:** Use VSCode with PlatformIO IDE extension

---

### Problem: Compilation Errors

**Symptoms:**
- Build fails with compiler errors
- Missing header files

**Solutions:**

1. **Update PlatformIO platform:**
```bash
cd firmware
pio platform update
```

2. **Clean build:**
```bash
pio run --target clean
pio run
```

3. **Install dependencies:**
```bash
pio lib install
```

4. **Check platformio.ini:**
Ensure all libraries are listed in `lib_deps`

---

### Problem: Upload Fails - Port Not Found

**Symptoms:**
- `Error: No upload port found`
- Serial port permission denied

**Solutions:**

1. **List available ports:**
```bash
pio device list
```

2. **Specify port manually:**
```bash
./scripts/upload_firmware.sh /dev/ttyUSB0
```

3. **Add user to dialout group (Linux):**
```bash
sudo usermod -a -G dialout $USER
# Logout and login again
```

4. **Windows:** Install CH340/CP2102 USB drivers

5. **Check cable:** Use a data cable, not charge-only

---

### Problem: Upload Stalls or Times Out

**Symptoms:**
- Upload starts but hangs at "Connecting..."
- Timeout after 10 seconds

**Solutions:**

1. **Hold BOOT button during upload:**
   - Press and hold BOOT
   - Click UPLOAD
   - Release BOOT when "Connecting..." appears

2. **Reset ESP32:**
   - Press EN/RST button
   - Try upload immediately

3. **Check baud rate:**
```ini
; platformio.ini
upload_speed = 115200  ; Try slower speed
```

4. **Erase flash completely:**
```bash
pio run --target erase
pio run --target upload
```

---

## Wi-Fi Connectivity Problems

### Problem: ESP32 Won't Connect to Wi-Fi

**Symptoms:**
- Serial output shows "Wi-Fi connection failed"
- Stuck in retry loop
- LED not blinking

**Diagnosis:**

1. **Check Serial output:**
```
pio device monitor
```

2. **Look for error messages:**
```
[WiFi] Connecting to SSID...
[WiFi] ✗ Connection failed
```

**Solutions:**

1. **Verify credentials in config.h:**
```cpp
#define WIFI_SSID "YourNetworkName"
#define WIFI_PASSWORD "YourPassword"
```

2. **Check 2.4GHz requirement:**
   - ESP32 only supports 2.4GHz Wi-Fi
   - Cannot connect to 5GHz-only networks

3. **Signal strength:**
   - Move ESP32 closer to router
   - Check for interference

4. **Router settings:**
   - Ensure DHCP is enabled
   - Check MAC address filtering
   - Verify WPA2 encryption (not WPA3)

5. **Hidden SSID:**
```cpp
WiFi.begin(WIFI_SSID, WIFI_PASSWORD, 0, NULL, true);
// Last parameter 'true' for hidden networks
```

---

### Problem: ESP32 Connects But No IP Address

**Symptoms:**
- Wi-Fi status shows "Connected"
- IP address is 0.0.0.0

**Solutions:**

1. **Wait longer:**
DHCP can take 5-10 seconds

2. **Static IP configuration:**
```cpp
// In WiFiManager.cpp
IPAddress local_IP(192, 168, 1, 100);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

WiFi.config(local_IP, gateway, subnet);
WiFi.begin(ssid, password);
```

3. **Check router DHCP pool:**
Ensure enough addresses available

---

### Problem: Wi-Fi Keeps Disconnecting

**Symptoms:**
- Connection drops intermittently
- Telemetry stops working

**Solutions:**

1. **Disable power save:**
```cpp
WiFi.setSleep(false);
```

2. **Increase timeout:**
```cpp
#define WIFI_TIMEOUT 20000  // 20 seconds
```

3. **Check signal strength:**
```cpp
int rssi = WiFi.RSSI();
// Should be > -70 dBm for stable connection
```

4. **Auto-reconnect enabled:**
WiFiManager already handles this, but verify:
```cpp
WiFi.setAutoReconnect(true);
```

---

## HTTP API Issues

### Problem: Can't Access /telemetry Endpoint

**Symptoms:**
- Browser shows "Connection refused"
- cURL times out
- Streamlit UI can't connect

**Diagnosis:**

1. **Ping ESP32:**
```bash
ping 192.168.1.100
```

2. **Check if server is running:**
Serial output should show:
```
[INIT] ✓ Web Server ready
[WebServer] Server started on port 80
```

**Solutions:**

1. **Verify ESP32 IP address:**
Check Serial monitor output during boot

2. **Firewall:**
```bash
# Temporarily disable (for testing)
sudo ufw disable

# Or allow port 80
sudo ufw allow 80/tcp
```

3. **Check same network:**
ESP32 and computer must be on same subnet

4. **Test with cURL:**
```bash
curl -v http://192.168.1.100/telemetry
```

5. **Browser console:**
Press F12, check for CORS or network errors

---

### Problem: Commands Not Executing

**Symptoms:**
- POST /command returns error
- Motors/servo don't respond
- Serial shows "Command execution failed"

**Solutions:**

1. **Check JSON format:**
```bash
# Valid
curl -X POST http://ESP32_IP/command \
  -H "Content-Type: application/json" \
  -d '{"command":"move_forward","speed":150}'

# Invalid (missing Content-Type header)
curl -X POST http://ESP32_IP/command \
  -d '{"command":"move_forward","speed":150}'
```

2. **Validate JSON:**
Use online JSON validator or:
```python
import json
json.loads('{"command":"test"}')  # Should not throw error
```

3. **Check subsystem status:**
```bash
curl http://ESP32_IP/telemetry | jq '.status'
```

4. **Serial monitor:**
Watch for error messages:
```
[CommandProcessor] ✗ Locomotion not available
```

5. **Enable subsystem:**
Some subsystems require explicit enable:
```json
{"command": "enable_motors"}
```

---

### Problem: Slow Response Times

**Symptoms:**
- Telemetry takes >1 second to fetch
- UI feels laggy
- Timeouts occur

**Solutions:**

1. **Check ESP32 heap:**
```bash
curl http://ESP32_IP/telemetry | jq '.free_heap'
# Should be > 100000
```

2. **Reduce polling rate:**
In Streamlit UI, increase `sleep(0.5)` to `sleep(1.0)`

3. **Wi-Fi signal:**
```bash
curl http://ESP32_IP/telemetry | jq '.wifi_rssi'
# Should be > -70
```

4. **Limit simultaneous connections:**
ESP32 can handle 1-2 clients max

5. **Check network congestion:**
```bash
ping -c 10 192.168.1.100
# Latency should be <10ms
```

---

## Subsystem Failures

### Problem: Servo Not Moving

**Symptoms:**
- `servo.isReady() = false`
- Set angle commands accepted but no movement

**Solutions:**

1. **Check Serial output:**
```
[INIT] ✗ ServoArm initialization failed
```

2. **Verify pin assignment:**
```cpp
#define SERVO_ARM_PIN 18
```

3. **Power supply:**
- Servos draw significant current
- Use external 5V supply, not USB

4. **Test servo:**
```bash
curl -X POST http://ESP32_IP/command \
  -H "Content-Type: application/json" \
  -d '{"command":"set_servo_angle","value":90}'
```

5. **PWM conflict:**
Check if another peripheral uses same pin

---

### Problem: I2C Sensor Not Found

**Symptoms:**
```
[INIT] ⚠ I2C Sensor initialization failed
[I2CSensor] Device not found at address 0x76
```

**Solutions:**

1. **I2C scanner:**
```cpp
// Add to setup()
Wire.begin();
for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
        Serial.printf("Device found at 0x%02X\n", addr);
    }
}
```

2. **Check wiring:**
```
BME280    ESP32
VCC   ->  3.3V
GND   ->  GND
SDA   ->  GPIO21
SCL   ->  GPIO22
```

3. **Pull-up resistors:**
Add 4.7kΩ resistors from SDA/SCL to 3.3V

4. **Address:**
BME280 can be 0x76 or 0x77. Try both:
```cpp
#define BME280_I2C_ADDRESS 0x77
```

5. **Power:**
Ensure sensor powered properly (3.3V, not 5V)

---

### Problem: Motors Not Running

**Symptoms:**
- `motorsEnabled = false` in telemetry
- No response to movement commands

**Solutions:**

1. **Enable motors:**
```json
{"command": "enable_motors"}
```

2. **Check motor driver power:**
- Separate power supply for motors (6-12V)
- Common ground with ESP32

3. **Verify pin assignments:**
```cpp
#define MOTOR_A_PWM_PIN 25
#define MOTOR_A_DIR_PIN 26
#define MOTOR_B_PWM_PIN 27
#define MOTOR_B_DIR_PIN 14
```

4. **Test individual motor:**
```json
{
  "command": "set_motor_speed",
  "motor": "A",
  "speed": 128,
  "forward": true
}
```

5. **Motor driver enable pin:**
Some drivers (L298N) have enable pin - connect to 5V

---

## Power & Performance Issues

### Problem: ESP32 Keeps Resetting

**Symptoms:**
- Random reboots
- Brown-out detector triggered
- Serial shows "rst:0x10 (RTCWDT_RTC_RESET)"

**Solutions:**

1. **Power supply:**
- Use 5V 2A power supply minimum
- USB ports may not provide enough current

2. **Capacitor:**
Add 100µF capacitor across 5V and GND

3. **Reduce load:**
```cpp
// Disable Wi-Fi temporarily
#define ENABLE_WIFI false
```

4. **Check for short circuits:**
Inspect wiring and connections

5. **Serial monitor baud rate:**
Ensure match in platformio.ini:
```ini
monitor_speed = 115200
```

---

### Problem: High Memory Usage

**Symptoms:**
- `free_heap` < 50000 in telemetry
- System unstable
- Crashes during operation

**Solutions:**

1. **Check heap continuously:**
```cpp
Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
```

2. **Reduce JSON buffer sizes:**
```cpp
StaticJsonDocument<512> doc;  // Instead of 1536
```

3. **Disable debug output:**
```cpp
#define DEBUG_VERBOSE false
```

4. **Remove unused subsystems:**
Comment out subsystems not needed

5. **Static vs Dynamic allocation:**
Use stack allocation where possible

---

## Streamlit UI Problems

### Problem: UI Won't Start

**Symptoms:**
```
streamlit: command not found
```

**Solutions:**

1. **Install Streamlit:**
```bash
pip install streamlit
```

2. **Use provided script:**
```bash
./scripts/run_ui.sh
```

3. **Activate venv:**
```bash
source venv/bin/activate
streamlit run ui/robot_console.py
```

4. **Check Python version:**
```bash
python3 --version
# Requires Python 3.7+
```

---

### Problem: UI Can't Connect to ESP32

**Symptoms:**
- "Connection Error" in UI
- Telemetry shows all zeros
- Error messages in Streamlit

**Solutions:**

1. **Update ESP32 IP:**
In UI sidebar, enter correct IP address

2. **Verify network connectivity:**
```bash
ping <ESP32_IP>
curl http://<ESP32_IP>/telemetry
```

3. **Check timeout:**
Increase timeout in `robot_console.py`:
```python
resp = requests.get(url, timeout=10)  # 10 seconds
```

4. **CORS:**
Already enabled in firmware, but verify

5. **Firewall:**
Allow outbound HTTP from Python

---

### Problem: Telemetry Not Updating

**Symptoms:**
- Stale data in UI
- Timestamp not changing

**Solutions:**

1. **Check auto-refresh:**
Streamlit's `st.experimental_rerun()` should be active

2. **Polling interval:**
Adjust sleep time:
```python
time.sleep(0.5)  # 500ms = 2 Hz
```

3. **ESP32 busy:**
Too many requests can overwhelm ESP32

4. **Serial monitor:**
Check if ESP32 actually updating

---

## Hardware Debugging

### Problem: No Serial Output

**Symptoms:**
- Serial monitor blank
- No initialization messages

**Solutions:**

1. **Correct baud rate:**
```bash
pio device monitor --baud 115200
```

2. **Select correct port:**
```bash
pio device monitor --port /dev/ttyUSB0
```

3. **Press EN/RST button:**
Reset ESP32 to see boot messages

4. **TX/RX pins:**
Don't use GPIO1 (TX) and GPIO3 (RX) for other purposes

---

### Problem: Erratic Behavior

**Symptoms:**
- Random values in sensors
- Unexpected movements
- Interference

**Solutions:**

1. **Ground loops:**
Ensure single common ground

2. **Power supply noise:**
Add capacitors (100µF + 0.1µF) near ESP32

3. **EMI:**
- Keep motor wires away from ESP32
- Use shielded cables for long runs
- Add flyback diodes to motors

4. **Breadboard issues:**
- Loose connections
- Use soldered connections for production

---

## Common Error Codes

| Code | Meaning | Solution |
|------|---------|----------|
| 100 | Wi-Fi connection failed | Check SSID/password in config.h |
| 101 | Wi-Fi disconnected | Check signal strength, router |
| 200 | I2C sensor not found | Verify wiring, pull-ups, address |
| 201 | I2C communication error | Check for bus conflicts, speed |
| 300 | Servo attach failed | Verify pin, check for conflicts |
| 301 | Motor driver error | Check power supply, connections |
| 400 | Low memory | Reduce buffer sizes, disable features |
| 500 | Invalid command | Check JSON format, command name |
| 502 | Command execution failed | Check subsystem availability |

---

## Diagnostic Tools

### Serial Monitor Commands

While in serial monitor, you can observe:

```
[INIT] messages - Initialization status
[WiFi] messages - Network activity
[CommandProcessor] - Command execution
[ErrorManager] - Error reports
--- TELEMETRY --- - JSON telemetry dumps
```

### Quick Health Check

```bash
# Full diagnostic
curl http://ESP32_IP/telemetry | jq '{
  heap: .free_heap,
  health: .errors.system_health,
  wifi_connected: (.free_heap > 0),
  subsystems: .status.subsystems_ready
}'
```

### LED Indicators

- **Blinking (0.5s interval):** Normal operation
- **Solid ON:** Stuck in initialization
- **Solid OFF:** Power issue or crashed
- **Fast blink:** Entering sleep mode

---

## Getting Help

If problems persist:

1. **Check Serial Output:** Most issues visible in logs
2. **GitHub Issues:** Report bugs with logs
3. **Documentation:** Review firmware/README.md
4. **Community:** Ask in discussions with:
   - ESP32 board type
   - Firmware version (Phase X)
   - Serial output
   - Steps to reproduce

---

## Preventive Maintenance

To avoid issues:

✅ **Regular backups** of working configurations
✅ **Document changes** in commit messages
✅ **Test incrementally** - don't change everything at once
✅ **Monitor heap** in telemetry
✅ **Keep libraries updated** via `pio lib update`
✅ **Use version control** (git) religiously
✅ **Label wires** for easy debugging

---

For architecture details, see `docs/architecture.md`
For API reference, see `docs/api.md`
