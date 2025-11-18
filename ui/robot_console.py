"""
ESP32 Robotics Console - Streamlit UI
WALL-E & EVE Apprenticeship - Phase 3

This Streamlit application provides a web-based interface for monitoring
and controlling the ESP32 robot firmware.

Features:
- Real-time telemetry display
- Robot control commands
- Telemetry history and logging
- System status monitoring
- Interactive charts and visualizations
"""

import streamlit as st
import requests
import json
import time
import sqlite3
import pandas as pd
from datetime import datetime
import plotly.graph_objects as go

# ============================================================================
# PAGE CONFIGURATION
# ============================================================================

st.set_page_config(
    page_title="🤖 ESP32 Robot Console",
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="expanded"
)

# ============================================================================
# SESSION STATE INITIALIZATION
# ============================================================================

if 'esp32_ip' not in st.session_state:
    st.session_state.esp32_ip = "192.168.1.100"  # Default IP

if 'last_telemetry' not in st.session_state:
    st.session_state.last_telemetry = None

if 'connected' not in st.session_state:
    st.session_state.connected = False

if 'command_history' not in st.session_state:
    st.session_state.command_history = []

# ============================================================================
# DATABASE FUNCTIONS
# ============================================================================

def init_database():
    """Initialize SQLite database for telemetry logging"""
    conn = sqlite3.connect('robot_telemetry.db')
    cursor = conn.cursor()

    cursor.execute('''
        CREATE TABLE IF NOT EXISTS telemetry (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp_local TEXT,
            timestamp_robot INTEGER,
            free_heap INTEGER,
            mode TEXT,
            servo_current_angle REAL,
            servo_target_angle REAL,
            servo_enabled INTEGER,
            temperature REAL,
            humidity REAL,
            pressure REAL,
            altitude REAL,
            motor_a_speed INTEGER,
            motor_b_speed INTEGER,
            motor_a_direction TEXT,
            motor_b_direction TEXT,
            motors_enabled INTEGER
        )
    ')

    conn.commit()
    conn.close()

def log_telemetry(telemetry_data):
    """Log telemetry data to database"""
    try:
        conn = sqlite3.connect('robot_telemetry.db')
        cursor = conn.cursor()

        cursor.execute('''
            INSERT INTO telemetry (
                timestamp_local, timestamp_robot, free_heap, mode,
                servo_current_angle, servo_target_angle, servo_enabled,
                temperature, humidity, pressure, altitude,
                motor_a_speed, motor_b_speed,
                motor_a_direction, motor_b_direction, motors_enabled
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        ''', (
            datetime.now().isoformat(),
            telemetry_data.get('timestamp', 0),
            telemetry_data.get('free_heap', 0),
            telemetry_data.get('mode', 'unknown'),
            telemetry_data.get('servo', {}).get('current_angle', 0),
            telemetry_data.get('servo', {}).get('target_angle', 0),
            1 if telemetry_data.get('servo', {}).get('enabled', False) else 0,
            telemetry_data.get('environment', {}).get('temperature', 0),
            telemetry_data.get('environment', {}).get('humidity', 0),
            telemetry_data.get('environment', {}).get('pressure', 0),
            telemetry_data.get('environment', {}).get('altitude', 0),
            telemetry_data.get('motors', {}).get('motor_a_speed', 0),
            telemetry_data.get('motors', {}).get('motor_b_speed', 0),
            telemetry_data.get('motors', {}).get('motor_a_direction', 'forward'),
            telemetry_data.get('motors', {}).get('motor_b_direction', 'forward'),
            1 if telemetry_data.get('motors', {}).get('enabled', False) else 0
        ))

        conn.commit()
        conn.close()
        return True
    except Exception as e:
        st.error(f"Database error: {e}")
        return False

def get_telemetry_history(limit=100):
    """Retrieve telemetry history from database"""
    try:
        conn = sqlite3.connect('robot_telemetry.db')
        df = pd.read_sql_query(f'''
            SELECT * FROM telemetry
            ORDER BY id DESC
            LIMIT {limit}
        ''', conn)
        conn.close()
        return df
    except Exception as e:
        st.error(f"Database read error: {e}")
        return pd.DataFrame()

# ============================================================================
# ESP32 COMMUNICATION FUNCTIONS
# ============================================================================

def fetch_telemetry(ip_address):
    """Fetch telemetry from ESP32"""
    try:
        response = requests.get(
            f"http://{ip_address}/telemetry",
            timeout=3
        )
        response.raise_for_status()
        return response.json()
    except requests.exceptions.Timeout:
        st.error("⏱️ Timeout: ESP32 not responding")
        return None
    except requests.exceptions.ConnectionError:
        st.error("🔌 Connection Error: Cannot reach ESP32")
        return None
    except requests.exceptions.HTTPError as e:
        st.error(f"HTTP Error: {e}")
        return None
    except Exception as e:
        st.error(f"Unexpected error: {e}")
        return None

def send_command(ip_address, command_dict):
    """Send command to ESP32"""
    try:
        response = requests.post(
            f"http://{ip_address}/command",
            json=command_dict,
            headers={'Content-Type': 'application/json'},
            timeout=3
        )
        response.raise_for_status()
        return response.json()
    except requests.exceptions.Timeout:
        st.error("⏱️ Timeout: Command not acknowledged")
        return None
    except requests.exceptions.ConnectionError:
        st.error("🔌 Connection Error: Cannot reach ESP32")
        return None
    except Exception as e:
        st.error(f"Command error: {e}")
        return None

def check_connection(ip_address):
    """Quick connection check"""
    try:
        response = requests.get(f"http://{ip_address}/status", timeout=2)
        return response.status_code == 200
    except:
        return False

# ============================================================================
# UI COMPONENTS
# ============================================================================

def render_sidebar():
    """Render sidebar with configuration"""
    with st.sidebar:
        st.header("⚙️ Configuration")

        # ESP32 IP Address
        ip = st.text_input(
            "ESP32 IP Address",
            value=st.session_state.esp32_ip,
            help="Enter the IP address of your ESP32 (shown in Serial output)"
        )

        if ip != st.session_state.esp32_ip:
            st.session_state.esp32_ip = ip

        # Connection status
        if st.button("🔄 Test Connection", use_container_width=True):
            with st.spinner("Connecting..."):
                st.session_state.connected = check_connection(st.session_state.esp32_ip)

        if st.session_state.connected:
            st.success("✓ Connected to ESP32")
        else:
            st.warning("⚠ Not connected")

        st.divider()

        # Auto-refresh settings
        st.subheader("🔄 Auto-Refresh")
        auto_refresh = st.checkbox("Enable auto-refresh", value=True)

        if auto_refresh:
            refresh_rate = st.slider(
                "Refresh rate (seconds)",
                min_value=0.5,
                max_value=5.0,
                value=1.0,
                step=0.5
            )
        else:
            refresh_rate = None

        st.divider()

        # Database management
        st.subheader("💾 Database")
        if st.button("📊 View History", use_container_width=True):
            st.session_state.show_history = True

        if st.button("🗑️ Clear History", use_container_width=True):
            try:
                conn = sqlite3.connect('robot_telemetry.db')
                cursor = conn.cursor()
                cursor.execute('DELETE FROM telemetry')
                conn.commit()
                conn.close()
                st.success("History cleared")
            except Exception as e:
                st.error(f"Error: {e}")

        return auto_refresh, refresh_rate

def render_telemetry_display(telemetry):
    """Render telemetry data display"""
    if telemetry is None:
        st.warning("No telemetry data available")
        return

    st.subheader("📊 System Telemetry")

    # System Info
    col1, col2, col3 = st.columns(3)

    with col1:
        uptime = telemetry.get('timestamp', 0) / 1000  # Convert ms to seconds
        st.metric("Uptime", f"{uptime:.1f} s")

    with col2:
        heap = telemetry.get('free_heap', 0)
        st.metric("Free Heap", f"{heap:,} bytes")

    with col3:
        mode = telemetry.get('mode', 'unknown')
        st.metric("Mode", mode)

    st.divider()

    # Servo Data
    servo = telemetry.get('servo', {})
    st.subheader("🔧 Servo Arm")

    col1, col2, col3 = st.columns(3)

    with col1:
        current = servo.get('current_angle', 0)
        st.metric("Current Angle", f"{current:.1f}°")

    with col2:
        target = servo.get('target_angle', 0)
        st.metric("Target Angle", f"{target:.1f}°")

    with col3:
        enabled = servo.get('enabled', False)
        status = "✓ Enabled" if enabled else "✗ Disabled"
        st.metric("Status", status)

    # Servo progress bar
    st.progress(int(current) / 180)

    st.divider()

    # Environmental Sensors
    env = telemetry.get('environment', {})
    st.subheader("🌡️ Environmental Sensors")

    col1, col2, col3, col4 = st.columns(4)

    with col1:
        temp = env.get('temperature', 0)
        st.metric("Temperature", f"{temp:.1f} °C")

    with col2:
        humidity = env.get('humidity', 0)
        st.metric("Humidity", f"{humidity:.1f} %")

    with col3:
        pressure = env.get('pressure', 0)
        st.metric("Pressure", f"{pressure:.1f} hPa")

    with col4:
        altitude = env.get('altitude', 0)
        st.metric("Altitude", f"{altitude:.1f} m")

    st.divider()

    # Motor Status
    motors = telemetry.get('motors', {})
    st.subheader("⚙️ Motors")

    col1, col2 = st.columns(2)

    with col1:
        st.write("**Motor A**")
        speed_a = motors.get('motor_a_speed', 0)
        dir_a = motors.get('motor_a_direction', 'forward')
        st.metric("Speed", f"{speed_a}/255")
        st.metric("Direction", dir_a)
        st.progress(speed_a / 255)

    with col2:
        st.write("**Motor B**")
        speed_b = motors.get('motor_b_speed', 0)
        dir_b = motors.get('motor_b_direction', 'forward')
        st.metric("Speed", f"{speed_b}/255")
        st.metric("Direction", dir_b)
        st.progress(speed_b / 255)

    motors_enabled = motors.get('enabled', False)
    if motors_enabled:
        st.success("✓ Motors Enabled")
    else:
        st.warning("⚠ Motors Disabled")

def render_command_panel():
    """Render command control panel"""
    st.subheader("🎮 Robot Control")

    tab1, tab2, tab3 = st.tabs(["Servo Control", "Motor Control", "Quick Commands"])

    with tab1:
        st.write("**Servo Arm Control**")

        angle = st.slider(
            "Target Angle",
            min_value=0,
            max_value=180,
            value=90,
            help="Set servo arm angle (0-180 degrees)"
        )

        if st.button("📐 Set Servo Angle", use_container_width=True):
            cmd = {"command": "set_servo_angle", "value": angle}
            result = send_command(st.session_state.esp32_ip, cmd)
            if result:
                st.success(f"✓ Servo set to {angle}°")
                st.session_state.command_history.append(cmd)

    with tab2:
        st.write("**Locomotion Control**")

        col1, col2 = st.columns(2)

        with col1:
            speed = st.slider("Speed", 0, 255, 128, help="Motor speed (0-255)")

        with col2:
            turn_rate = st.slider("Turn Rate", 0, 255, 64, help="Turning sharpness")

        col1, col2, col3 = st.columns(3)

        with col1:
            if st.button("⬆️ Forward", use_container_width=True):
                cmd = {"command": "move_forward", "speed": speed}
                result = send_command(st.session_state.esp32_ip, cmd)
                if result:
                    st.success("Moving forward")
                    st.session_state.command_history.append(cmd)

        with col2:
            if st.button("⬇️ Backward", use_container_width=True):
                cmd = {"command": "move_backward", "speed": speed}
                result = send_command(st.session_state.esp32_ip, cmd)
                if result:
                    st.success("Moving backward")
                    st.session_state.command_history.append(cmd)

        with col3:
            if st.button("🛑 Stop", use_container_width=True):
                cmd = {"command": "stop_motors"}
                result = send_command(st.session_state.esp32_ip, cmd)
                if result:
                    st.success("Motors stopped")
                    st.session_state.command_history.append(cmd)

        col1, col2 = st.columns(2)

        with col1:
            if st.button("⬅️ Turn Left", use_container_width=True):
                cmd = {"command": "turn_left", "speed": speed, "turn_rate": turn_rate}
                result = send_command(st.session_state.esp32_ip, cmd)
                if result:
                    st.success("Turning left")
                    st.session_state.command_history.append(cmd)

        with col2:
            if st.button("➡️ Turn Right", use_container_width=True):
                cmd = {"command": "turn_right", "speed": speed, "turn_rate": turn_rate}
                result = send_command(st.session_state.esp32_ip, cmd)
                if result:
                    st.success("Turning right")
                    st.session_state.command_history.append(cmd)

        col1, col2 = st.columns(2)

        with col1:
            if st.button("🔄 Rotate CW", use_container_width=True):
                cmd = {"command": "rotate", "speed": speed, "clockwise": True}
                result = send_command(st.session_state.esp32_ip, cmd)
                if result:
                    st.success("Rotating clockwise")
                    st.session_state.command_history.append(cmd)

        with col2:
            if st.button("🔃 Rotate CCW", use_container_width=True):
                cmd = {"command": "rotate", "speed": speed, "clockwise": False}
                result = send_command(st.session_state.esp32_ip, cmd)
                if result:
                    st.success("Rotating counter-clockwise")
                    st.session_state.command_history.append(cmd)

    with tab3:
        st.write("**Quick Actions**")

        col1, col2 = st.columns(2)

        with col1:
            if st.button("✅ Enable Motors", use_container_width=True):
                cmd = {"command": "enable_motors"}
                result = send_command(st.session_state.esp32_ip, cmd)
                if result:
                    st.success("Motors enabled")
                    st.session_state.command_history.append(cmd)

        with col2:
            if st.button("❌ Disable Motors", use_container_width=True):
                cmd = {"command": "disable_motors"}
                result = send_command(st.session_state.esp32_ip, cmd)
                if result:
                    st.success("Motors disabled")
                    st.session_state.command_history.append(cmd)

def render_telemetry_charts(df):
    """Render telemetry history charts"""
    if df.empty:
        st.warning("No historical data available")
        return

    st.subheader("📈 Telemetry History")

    # Temperature over time
    fig_temp = go.Figure()
    fig_temp.add_trace(go.Scatter(
        y=df['temperature'][::-1],  # Reverse for chronological order
        mode='lines+markers',
        name='Temperature',
        line=dict(color='red')
    ))
    fig_temp.update_layout(
        title="Temperature Over Time",
        yaxis_title="Temperature (°C)",
        xaxis_title="Samples"
    )
    st.plotly_chart(fig_temp, use_container_width=True)

    # Motor speeds over time
    fig_motors = go.Figure()
    fig_motors.add_trace(go.Scatter(
        y=df['motor_a_speed'][::-1],
        mode='lines',
        name='Motor A',
        line=dict(color='blue')
    ))
    fig_motors.add_trace(go.Scatter(
        y=df['motor_b_speed'][::-1],
        mode='lines',
        name='Motor B',
        line=dict(color='green')
    ))
    fig_motors.update_layout(
        title="Motor Speeds Over Time",
        yaxis_title="Speed (0-255)",
        xaxis_title="Samples"
    )
    st.plotly_chart(fig_motors, use_container_width=True)

# ============================================================================
# MAIN APPLICATION
# ============================================================================

def main():
    """Main application"""

    # Initialize database
    init_database()

    # Title
    st.title("🤖 ESP32 Robotics Console")
    st.markdown("**WALL-E & EVE Apprenticeship | Phase 3: Wi-Fi Telemetry & Streamlit UI**")

    # Sidebar
    auto_refresh, refresh_rate = render_sidebar()

    # Main content
    col1, col2 = st.columns([3, 2])

    with col1:
        # Telemetry display
        if st.button("🔄 Refresh Now") or auto_refresh:
            telemetry = fetch_telemetry(st.session_state.esp32_ip)
            if telemetry:
                st.session_state.last_telemetry = telemetry
                st.session_state.connected = True
                log_telemetry(telemetry)

        render_telemetry_display(st.session_state.last_telemetry)

    with col2:
        # Command panel
        render_command_panel()

        # Command history
        if st.session_state.command_history:
            with st.expander("📜 Command History"):
                for i, cmd in enumerate(reversed(st.session_state.command_history[-10:])):
                    st.code(json.dumps(cmd, indent=2), language="json")

    # Telemetry history charts
    if st.checkbox("📊 Show Telemetry History"):
        df = get_telemetry_history(100)
        render_telemetry_charts(df)

        # Raw data table
        if st.checkbox("📋 Show Raw Data"):
            st.dataframe(df, use_container_width=True)

    # Auto-refresh
    if auto_refresh and refresh_rate:
        time.sleep(refresh_rate)
        st.rerun()

    # Footer
    st.divider()
    st.caption("ESP32 Robotics Firmware | Powered by Streamlit")

if __name__ == "__main__":
    main()
