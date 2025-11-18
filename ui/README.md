# Robot Control Console - Streamlit UI

Phase 3: Wi-Fi Telemetry & Streamlit UI Integration

## Overview

This is a web-based control console for the ESP32 robot, built with Streamlit. It provides real-time telemetry monitoring, robot control, historical data visualization, and SQLite-based telemetry logging.

## Features

### Real-Time Telemetry Monitoring
- **System Information:** Uptime, free heap memory, operational mode
- **Servo Status:** Current angle, target angle, enabled state
- **Environmental Data:** Temperature, humidity, pressure, altitude from BME280 sensor
- **Motor Status:** Speed, direction, and enabled state for both motors
- **SPI Device:** Transfer count and test values
- **System Status:** Error codes, subsystem readiness

### Robot Control Panel
- **Servo Control:** Interactive slider for precise angle control (0-180°)
- **Motor Control:** Individual motor speed and direction control
- **Quick Commands:** Pre-configured movement buttons
  - Move Forward/Backward
  - Turn Left/Right
  - Rotate Clockwise/Counter-clockwise
  - Stop Motors
  - Enable/Disable Motors

### Data Visualization
- **Temperature Chart:** Historical temperature readings with Plotly
- **Motor Speed Chart:** Track motor A and B speeds over time
- **Auto-refresh:** Configurable refresh rate (1-10 seconds)

### Database Logging
- **SQLite Integration:** All telemetry data logged to `robot_telemetry.db`
- **Telemetry History:** View past telemetry records
- **Command History:** Track last 10 commands sent to robot

## Requirements

### Hardware
- ESP32 robot with Phase 3 firmware uploaded
- ESP32 connected to Wi-Fi network
- Computer on same network as ESP32

### Software
- Python 3.8 or higher
- pip (Python package manager)

## Installation

### 1. Install Python Dependencies

Navigate to the `ui/` directory and install requirements:

```bash
cd ui
pip install -r requirements.txt
```

This will install:
- `streamlit` - Web application framework
- `requests` - HTTP client for API communication
- `pandas` - Data manipulation
- `plotly` - Interactive charts
- `altair` - Alternative charting library

### 2. Verify Installation

Check that Streamlit is installed correctly:

```bash
streamlit --version
```

You should see output like: `Streamlit, version 1.28.0`

## Usage

### 1. Start Your ESP32 Robot

1. Upload Phase 3 firmware to your ESP32
2. Configure Wi-Fi credentials in `firmware/include/config.h`
3. Open serial monitor and note the IP address:
   ```
   [WiFiManager] ✓ Connected to YourWiFiSSID
   [WiFiManager] IP Address: 192.168.1.123
   ```

### 2. Launch the Streamlit UI

From the `ui/` directory:

```bash
streamlit run robot_console.py
```

The application will open automatically in your browser at `http://localhost:8501`

### 3. Connect to Your Robot

In the sidebar:
1. Enter your ESP32's IP address (e.g., `192.168.1.123`)
2. Click "Test Connection" to verify connectivity
3. Enable "Auto-refresh telemetry" for real-time updates
4. Adjust refresh rate as needed (default: 2 seconds)

### 4. Monitor Telemetry

The main panel displays real-time data from your robot:
- System metrics (uptime, memory)
- Servo position and status
- Environmental readings
- Motor speeds and directions

### 5. Control Your Robot

Use the control panel tabs:

**Servo Control:**
- Drag the slider to set desired servo angle
- Click "Set Servo Angle" to send command

**Motor Control:**
- Select motor (A, B, or Both)
- Set speed (0-255)
- Choose direction (Forward/Reverse)
- Click "Set Motor Speed"

**Quick Commands:**
- Click movement buttons for instant actions
- "Stop Motors" for emergency stop
- "Enable/Disable Motors" for safety

### 6. View Historical Data

**Telemetry Charts:**
- Temperature over time (last 50 readings)
- Motor speeds over time (both motors)

**Telemetry History:**
- Expand "Show Telemetry History" to view database records
- Last 20 telemetry entries displayed

**Command History:**
- View last 10 commands sent to robot
- Includes timestamps and command details

## API Communication

The UI communicates with the ESP32 via HTTP REST API:

### Fetching Telemetry
```python
response = requests.get(f"http://{esp32_ip}/telemetry", timeout=3)
telemetry_data = response.json()
```

### Sending Commands
```python
command = {
    "command": "set_servo_angle",
    "value": 90
}
response = requests.post(
    f"http://{esp32_ip}/command",
    json=command,
    timeout=3
)
```

## Database Schema

The SQLite database (`robot_telemetry.db`) stores telemetry with this schema:

```sql
CREATE TABLE telemetry (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp TEXT,
    uptime INTEGER,
    free_heap INTEGER,
    mode TEXT,
    servo_current REAL,
    servo_target REAL,
    servo_enabled INTEGER,
    servo_at_target INTEGER,
    temperature REAL,
    humidity REAL,
    pressure REAL,
    altitude REAL,
    motor_a_speed INTEGER,
    motor_b_speed INTEGER,
    motor_a_direction TEXT,
    motor_b_direction TEXT,
    motors_enabled INTEGER,
    error_code INTEGER,
    error_message TEXT
)
```

## Troubleshooting

### Cannot Connect to Robot

**Problem:** "Connection failed" error when testing connection

**Solutions:**
1. Verify ESP32 IP address is correct
2. Ensure ESP32 is connected to Wi-Fi (check serial monitor)
3. Ping the ESP32: `ping 192.168.1.XXX`
4. Check firewall settings on your computer
5. Ensure both devices on same network (not guest network)
6. Try accessing `http://ESP32_IP/` in web browser

### Streamlit Won't Start

**Problem:** `streamlit: command not found`

**Solutions:**
1. Verify Python installation: `python --version`
2. Reinstall streamlit: `pip install --upgrade streamlit`
3. Try: `python -m streamlit run robot_console.py`
4. Check PATH includes Python Scripts directory

### Database Errors

**Problem:** SQLite database errors

**Solutions:**
1. Delete `robot_telemetry.db` and restart application
2. Check file permissions in ui/ directory
3. Ensure sufficient disk space

### Charts Not Displaying

**Problem:** Graphs are blank or not updating

**Solutions:**
1. Ensure telemetry is being received (check main panel)
2. Database must have at least 2 entries for charts
3. Try refreshing the browser (F5)
4. Clear browser cache

### Slow Performance

**Problem:** UI is laggy or unresponsive

**Solutions:**
1. Increase auto-refresh interval to 5-10 seconds
2. Disable auto-refresh when not needed
3. Clear telemetry history periodically
4. Close other browser tabs
5. Reduce chart data points (edit `robot_console.py`)

## Configuration

### Changing Default IP Address

Edit `robot_console.py` to set a default IP:

```python
# Around line 150
esp32_ip = st.text_input(
    "ESP32 IP Address",
    value="192.168.1.123",  # Change this default
    help="Enter the IP address of your ESP32"
)
```

### Adjusting Refresh Rate

Default refresh rate is 2 seconds. To change:

```python
# Around line 160
refresh_rate = st.slider(
    "Refresh rate (seconds)",
    min_value=1,
    max_value=10,
    value=2  # Change this default
)
```

### Database Location

By default, database is created as `robot_telemetry.db` in the ui/ directory. To change:

```python
# Around line 20 in init_database()
conn = sqlite3.connect('robot_telemetry.db')  # Change filename here
```

## Advanced Usage

### Running on Different Port

```bash
streamlit run robot_console.py --server.port 8080
```

### Running on Network (Accessible from Other Devices)

```bash
streamlit run robot_console.py --server.address 0.0.0.0
```

Then access from other devices at `http://YOUR_COMPUTER_IP:8501`

### Exporting Telemetry Data

Use any SQLite browser or Python:

```python
import sqlite3
import pandas as pd

conn = sqlite3.connect('robot_telemetry.db')
df = pd.read_sql_query("SELECT * FROM telemetry", conn)
df.to_csv('telemetry_export.csv', index=False)
```

## Development

### Project Structure

```
ui/
├── robot_console.py      # Main Streamlit application
├── requirements.txt      # Python dependencies
├── README.md            # This file
└── robot_telemetry.db   # SQLite database (created on first run)
```

### Extending the UI

The application uses Streamlit session state for:
- Connection status tracking
- Command history management
- Auto-refresh control

To add new features:

1. **Add new telemetry fields:** Update the telemetry display section
2. **Add new commands:** Create new buttons in the control panel
3. **Add new charts:** Use Plotly or Altair for visualizations
4. **Modify database:** Update schema in `init_database()` function

### Code Organization

- `init_database()` - Initialize SQLite database
- `log_telemetry()` - Insert telemetry into database
- `fetch_telemetry()` - GET request to /telemetry endpoint
- `send_command()` - POST request to /command endpoint
- `display_telemetry()` - Format and show telemetry data
- `control_panel()` - Robot control interface
- Main section - Layout and auto-refresh logic

## Resources

- [Streamlit Documentation](https://docs.streamlit.io/)
- [Plotly Python Guide](https://plotly.com/python/)
- [Requests Library](https://requests.readthedocs.io/)
- [SQLite Documentation](https://www.sqlite.org/docs.html)
- [Firmware API Reference](../firmware/README.md#api-documentation)

## Tips

1. **Use Auto-Refresh Wisely:** Only enable when actively monitoring
2. **Check Command History:** Verify commands were sent successfully
3. **Monitor Temperature:** Watch for overheating during extended operation
4. **Database Maintenance:** Periodically export and clear old data
5. **Network Stability:** Use wired Ethernet for most reliable connection
6. **Error Handling:** Watch for error codes in telemetry status

## License

Educational project for learning robotics and web development.

---

**Happy Robot Controlling! 🤖**

*The future is autonomous, but the controls are in your hands!*
