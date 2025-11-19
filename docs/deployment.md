# ESP32 Robotics Firmware - Deployment Guide

**Production Deployment Checklist and Best Practices**

---

## Pre-Deployment Checklist

### ✅ Code Quality

- [ ] All unit tests passing (`./scripts/test_firmware.sh`)
- [ ] No compiler warnings
- [ ] Code reviewed and documented
- [ ] Memory leaks checked (monitor `free_heap` over time)
- [ ] All debug output disabled or conditional

### ✅ Configuration

- [ ] Wi-Fi credentials set in `config.h`
- [ ] Correct GPIO pin assignments
- [ ] Power timeouts configured appropriately
- [ ] Debug flags set to `false` for production

### ✅ Hardware

- [ ] All connections tested and secure
- [ ] Power supply adequate (5V 2A minimum)
- [ ] Capacitors installed for stability
- [ ] Flyback diodes on motors
- [ ] Proper grounding established

### ✅ Testing

- [ ] End-to-end system test completed
- [ ] All commands tested via API
- [ ] Telemetry verified accurate
- [ ] Error recovery tested
- [ ] Power modes tested
- [ ] Long-duration stability test (24+ hours)

### ✅ Documentation

- [ ] Hardware setup documented
- [ ] Pin assignments documented
- [ ] API endpoints documented
- [ ] Troubleshooting procedures written
- [ ] User manual created

---

## Deployment Steps

### Step 1: Final Build

```bash
cd firmware

# Clean build
pio run --target clean

# Production build
pio run --environment release

# Verify size
pio run --target size
```

**Check:**
- Firmware size < 1.5MB
- RAM usage reasonable
- No warnings

### Step 2: Upload to ESP32

```bash
# Upload
./scripts/upload_firmware.sh /dev/ttyUSB0

# Verify boot
pio device monitor
```

**Verify:**
- All subsystems initialize successfully
- Wi-Fi connects
- Web server starts
- No error messages

### Step 3: Deploy UI

```bash
# Install dependencies
cd ui
pip install -r requirements.txt

# Test locally
streamlit run robot_console.py

# Configure for production
# Edit robot_console.py:
# - Set default ESP32_IP
# - Disable debug logging
# - Set appropriate timeouts
```

### Step 4: Network Configuration

**Static IP (Recommended for Production):**

In `WiFiManager.cpp`:
```cpp
IPAddress local_IP(192, 168, 1, 100);  # Fixed IP
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

WiFi.config(local_IP, gateway, subnet);
```

**DHCP Reservation:**
- Add MAC address to router's DHCP reservation list
- Ensures same IP every boot

### Step 5: Security Hardening

**⚠️ Current Implementation - Educational Only**

The current firmware has NO authentication or encryption.

**Production Recommendations:**

1. **Add API Key Authentication:**
```cpp
// In RobotWebServer.h
bool validateApiKey(const String& key) {
    return (key == API_SECRET_KEY);
}
```

2. **Enable HTTPS:**
```cpp
WiFiClientSecure client;
// Load certificates
```

3. **Restrict CORS:**
```cpp
// Only allow specific origins
server.sendHeader("Access-Control-Allow-Origin", "https://your-domain.com");
```

4. **Rate Limiting:**
```cpp
// Limit requests per IP
unsigned long lastRequest[MAX_CLIENTS];
```

5. **Firmware Encryption:**
Enable flash encryption in ESP32 settings

### Step 6: Monitoring Setup

**Serial Logging:**
```bash
# Continuous logging to file
pio device monitor > robot_log.txt 2>&1 &
```

**Remote Logging:**
- Send errors to syslog server
- Push telemetry to time-series database (InfluxDB)
- Set up alerts for critical errors

**Health Checks:**
- Ping ESP32 every 60 seconds
- Monitor `system_health` metric
- Alert if `free_heap` < 50000

### Step 7: Backup and Recovery

**Save Configuration:**
```bash
# Backup config.h
cp firmware/include/config.h firmware/config.h.backup

# Save flash contents
esptool.py --port /dev/ttyUSB0 read_flash 0 0x400000 firmware_backup.bin
```

**OTA Update Setup:**
```cpp
// Add to main.cpp
#include <ArduinoOTA.h>

void setupOTA() {
    ArduinoOTA.begin();
}

void loop() {
    ArduinoOTA.handle();
    // ... rest of loop
}
```

---

## Production Optimizations

### Memory Optimization

```cpp
// Reduce JSON buffer sizes
StaticJsonDocument<512> doc;  // Smaller buffers

// Disable verbose logging
#define DEBUG_VERBOSE false
#define DEBUG_SUBSYSTEM_STATUS false

// Reduce telemetry rate
#define TELEMETRY_INTERVAL 500  // 2 Hz instead of 10 Hz
```

### Power Optimization

```cpp
// Aggressive power management
PowerManager* powerMgr = new PowerManager(
    10000,  // Idle after 10s
    30000,  // Low-power after 30s
    120000  // Sleep after 2 minutes
);

// Enable Wi-Fi power save
powerMgr->enableWifiPowerSave(true);
```

### Performance Tuning

```cpp
// Reduce main loop delay
delay(5);  // 5ms instead of 10ms for faster response

// Increase CPU frequency
setCpuFrequencyMhz(240);  // Max performance
```

---

## Monitoring & Maintenance

### Key Metrics to Track

| Metric | Normal Range | Alert Threshold |
|--------|--------------|-----------------|
| Free Heap | > 200000 | < 100000 |
| System Health | 90-100% | < 80% |
| Wi-Fi RSSI | > -70 dBm | < -80 dBm |
| Uptime | Continuous | Unexpected resets |
| Error Count | 0-5 | > 10 |
| Temperature | 20-50°C | > 60°C |

### Automated Monitoring Script

```python
#!/usr/bin/env python3
import requests
import time

ESP32_IP = "192.168.1.100"

while True:
    try:
        resp = requests.get(f"http://{ESP32_IP}/telemetry", timeout=5)
        data = resp.json()

        # Check critical metrics
        if data['free_heap'] < 100000:
            print(f"⚠️ LOW MEMORY: {data['free_heap']}")

        if data['errors']['system_health'] < 80:
            print(f"⚠️ LOW HEALTH: {data['errors']['system_health']}%")

        # Log to file
        with open('robot_health.log', 'a') as f:
            f.write(f"{time.time()},{data['free_heap']},{data['errors']['system_health']}\n")

    except Exception as e:
        print(f"❌ ERROR: {e}")

    time.sleep(60)  # Check every minute
```

### Maintenance Schedule

**Daily:**
- Check error logs
- Verify connectivity
- Review system health

**Weekly:**
- Analyze telemetry trends
- Check for firmware updates
- Test backup procedures

**Monthly:**
- Deep clean hardware
- Check connections
- Update libraries
- Performance benchmark

---

## Scaling Considerations

### Multiple Robots

**Unique Configuration per Robot:**
```cpp
#define ROBOT_ID "ROBOT_01"
#define ROBOT_NAME "Explorer"
```

**Central Management:**
- Use MQTT for fleet communication
- Implement robot discovery protocol
- Central telemetry aggregation

**Load Balancing:**
- Distribute UI clients across robots
- Round-robin command routing

### Cloud Integration

**AWS IoT Core:**
```cpp
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

WiFiClientSecure net;
PubSubClient client(net);

void publishToCloud() {
    String payload = telemetry.toJson();
    client.publish("robot/telemetry", payload.c_str());
}
```

**Google Cloud IoT:**
Similar MQTT-based integration

**Azure IoT Hub:**
Device SDK available for ESP32

---

## Troubleshooting Deployment Issues

### Firmware Won't Start After Deploy

1. Check Serial output for boot errors
2. Verify power supply adequate
3. Try erasing flash: `pio run --target erase`
4. Re-upload known-good firmware

### Intermittent Wi-Fi Disconnects

1. Check RSSI strength
2. Reduce TX power if too close to AP
3. Disable Wi-Fi power save
4. Check for channel congestion

### Memory Leaks in Production

1. Monitor `free_heap` over 24+ hours
2. Check for String concatenation in loops
3. Verify proper cleanup in subsystems
4. Use static allocation where possible

### Performance Degradation

1. Check for excessive logging
2. Monitor CPU load (if available)
3. Reduce telemetry rate
4. Profile slow functions

---

## Rollback Procedures

### Quick Rollback

```bash
# Flash saved backup
esptool.py --port /dev/ttyUSB0 write_flash 0 firmware_backup.bin

# Or use git
git checkout previous-stable-version
./scripts/build_firmware.sh
./scripts/upload_firmware.sh
```

### Disaster Recovery

1. **Have spare ESP32** with working firmware
2. **Keep backup binaries** of stable versions
3. **Document configuration** in version control
4. **Test rollback procedure** before deployment

---

## Best Practices Summary

✅ **Version Everything:**  Tag releases, document changes
✅ **Test Thoroughly:** Unit tests, integration tests, stress tests
✅ **Monitor Actively:** Automated health checks, alerting
✅ **Update Cautiously:** Staged rollouts, rollback plan
✅ **Document Everything:** Code, procedures, troubleshooting
✅ **Backup Regularly:** Firmware, configuration, data
✅ **Secure by Default:** Authentication, encryption, validation
✅ **Plan for Failure:** Graceful degradation, automatic recovery

---

## Production Checklist Template

```
Deployment: ESP32 Robot #___
Date: __________
Deployed by: __________

Pre-Deployment:
[ ] Tests passing
[ ] Hardware verified
[ ] Configuration set
[ ] Backup created

Deployment:
[ ] Firmware uploaded
[ ] Boot successful
[ ] Wi-Fi connected: IP = __________
[ ] API accessible
[ ] UI functional

Post-Deployment:
[ ] 1-hour stability test passed
[ ] Telemetry verified
[ ] Error logs clean
[ ] Documentation updated

Sign-off:
Name: __________
Signature: __________
Date: __________
```

---

For detailed troubleshooting, see `docs/troubleshooting.md`
For API reference, see `docs/api.md`
For architecture details, see `docs/architecture.md`
