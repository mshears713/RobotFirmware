# ESP32 Robotics Firmware - Quick Start Guide

**Get up and running in 15 minutes!**

---

## Prerequisites

- ESP32 development board
- USB cable
- Computer with Python 3.7+ and PlatformIO
- Wi-Fi network (2.4GHz)

---

## Step 1: Clone Repository

```bash
git clone https://github.com/yourusername/RobotFirmware.git
cd RobotFirmware
```

---

## Step 2: Configure Wi-Fi

Edit `firmware/include/config.h`:

```cpp
#define WIFI_SSID "YourNetworkName"
#define WIFI_PASSWORD "YourPassword"
```

---

## Step 3: Build & Upload Firmware

```bash
# Build firmware
./scripts/build_firmware.sh

# Upload to ESP32 (connect via USB first)
./scripts/upload_firmware.sh

# Monitor serial output
cd firmware
pio device monitor
```

**Look for:** `Wi-Fi Connected! IP Address: 192.168.x.x`

**Copy the IP address** - you'll need it for the UI!

---

## Step 4: Launch Streamlit UI

```bash
# Install and run UI
./scripts/run_ui.sh
```

Browser should open automatically to `http://localhost:8501`

**In the UI sidebar:**
1. Enter the ESP32 IP address (from Step 3)
2. Click "Connect"
3. Telemetry should start appearing!

---

## Step 5: Test Robot Control

In the Streamlit UI:

1. **Servo Control:** Set angle to 90° and click "Set Angle"
2. **Motor Control:** Click "Enable Motors"
3. **Movement:** Click "Move Forward" for 2 seconds
4. **Stop:** Click "Stop Motors"

---

## Quick Verification

### ✅ Firmware Working?

Serial output should show:
```
[INIT] ✓ ServoArm ready
[INIT] ✓ WiFi Manager ready
[INIT] ✓ Web Server ready
[INIT] ✓ Power Manager ready
...
Initialization Complete!
```

### ✅ API Working?

Test from command line:
```bash
curl http://192.168.1.100/telemetry
```

Should return JSON with telemetry data.

### ✅ UI Working?

- Telemetry display shows live data
- System Health shows 100%
- No error messages

---

## Minimal Hardware Test

**Just want to see it work without hardware?**

The firmware runs fine without external components:

- ✅ Servo subsystem initializes (no servo needed)
- ✅ I2C sensor fails gracefully (warning only)
- ✅ Motors disabled by default (safe without driver)
- ✅ SPI device emulated
- ✅ All Phase 4 features functional

**Test with just ESP32 + USB!**

---

## Common First-Run Issues

### ESP32 won't connect to Wi-Fi

- Verify SSID/password in `config.h`
- Ensure 2.4GHz network (ESP32 doesn't support 5GHz)
- Check if network has MAC filtering

### Can't upload firmware

- Press BOOT button during upload
- Check USB cable (must be data cable, not charge-only)
- Verify correct port: `pio device list`

### UI can't connect

- Check ESP32 IP in Serial monitor
- Ensure computer and ESP32 on same network
- Try: `ping <ESP32_IP>`

---

## Next Steps

Once everything works:

1. **Hardware:** Connect servo, sensors, motors (see hardware guide)
2. **Examples:** Try `examples/test_sequence.py`
3. **Documentation:** Read `docs/architecture.md` for details
4. **Customize:** Modify commands in `CommandProcessor.h`
5. **Advanced:** Explore power management modes

---

## Project Structure

```
RobotFirmware/
├── firmware/           # ESP32 C++ code
│   ├── src/main.cpp   # Main firmware
│   └── include/       # Headers
├── ui/                # Streamlit Python UI
│   └── robot_console.py
├── scripts/           # Build/upload scripts
├── examples/          # Example code
├── docs/              # Documentation
└── QUICKSTART.md      # This file
```

---

## Help & Support

- **Troubleshooting:** See `docs/troubleshooting.md`
- **API Reference:** See `docs/api.md`
- **Architecture:** See `docs/architecture.md`
- **Issues:** Report on GitHub

---

## What's Included - All Phases Complete!

✅ **Phase 1:** GPIO control, servo basics
✅ **Phase 2:** I2C sensors, SPI, motors, telemetry
✅ **Phase 3:** Wi-Fi, HTTP API, Streamlit UI
✅ **Phase 4:** Power management, sensor fusion, error handling
✅ **Phase 5:** Documentation, examples, deployment tools

**You have a complete robotics firmware framework!**

---

## Quick Command Reference

```bash
# Build
./scripts/build_firmware.sh

# Upload
./scripts/upload_firmware.sh

# Test
./scripts/test_firmware.sh

# Run UI
./scripts/run_ui.sh

# Monitor serial
cd firmware && pio device monitor

# Clean build
cd firmware && pio run --target clean
```

---

**Ready to build your robot? Let's go! 🤖**
