#!/usr/bin/env python3
"""
ESP32 Robot Test Sequence
Demonstrates complete robot control flow
"""

import requests
import time
import json

# Configuration
ESP32_IP = "192.168.1.100"  # Change to your ESP32 IP
BASE_URL = f"http://{ESP32_IP}"

def send_command(command_data):
    """Send command to ESP32 and print response"""
    try:
        resp = requests.post(
            f"{BASE_URL}/command",
            json=command_data,
            timeout=5
        )
        print(f"✓ {command_data['command']}: {resp.json()}")
        return True
    except Exception as e:
        print(f"✗ {command_data['command']}: {e}")
        return False

def get_telemetry():
    """Fetch current telemetry"""
    try:
        resp = requests.get(f"{BASE_URL}/telemetry", timeout=5)
        return resp.json()
    except Exception as e:
        print(f"✗ Telemetry fetch failed: {e}")
        return None

def print_status(telemetry):
    """Print key telemetry values"""
    if not telemetry:
        return

    print("\n" + "="*50)
    print(f"Timestamp: {telemetry['timestamp']} ms")
    print(f"Mode: {telemetry['mode']}")
    print(f"Servo: {telemetry['servo']['current_angle']}°")
    print(f"Temp: {telemetry['environment']['temperature']}°C")
    print(f"Motors: {'ENABLED' if telemetry['motors']['enabled'] else 'DISABLED'}")
    print(f"System Health: {telemetry['errors']['system_health']}%")
    print("="*50 + "\n")

def test_sequence():
    """Run complete test sequence"""

    print("\n🤖 ESP32 Robot Test Sequence")
    print("="*50)

    # Test 1: System Health Check
    print("\n[Test 1] System Health Check")
    telemetry = get_telemetry()
    if telemetry:
        print_status(telemetry)
    else:
        print("❌ Failed to connect to ESP32")
        return

    # Test 2: Servo Control
    print("\n[Test 2] Servo Control")
    angles = [0, 45, 90, 135, 180, 90]
    for angle in angles:
        send_command({"command": "set_servo_angle", "value": angle})
        time.sleep(1)

    # Test 3: Motor Enable/Disable
    print("\n[Test 3] Motor Safety")
    send_command({"command": "enable_motors"})
    time.sleep(0.5)

    # Test 4: Basic Movement
    print("\n[Test 4] Basic Movement")
    movements = [
        {"command": "move_forward", "speed": 120},
        {"command": "move_backward", "speed": 120},
        {"command": "turn_left", "speed": 100, "turn_rate": 50},
        {"command": "turn_right", "speed": 100, "turn_rate": 50},
        {"command": "rotate", "speed": 80, "clockwise": True},
        {"command": "rotate", "speed": 80, "clockwise": False},
        {"command": "stop_motors"}
    ]

    for move in movements:
        send_command(move)
        time.sleep(2)  # Run each movement for 2 seconds

    # Test 5: Motor Disable
    print("\n[Test 5] Disable Motors")
    send_command({"command": "disable_motors"})
    time.sleep(0.5)

    # Test 6: Power Management
    print("\n[Test 6] Power Management")
    send_command({"command": "set_power_mode", "mode": "idle"})
    time.sleep(1)
    send_command({"command": "set_power_mode", "mode": "active"})
    time.sleep(1)

    # Test 7: Error Management
    print("\n[Test 7] Error Management")
    send_command({"command": "perform_health_check"})
    time.sleep(0.5)
    send_command({"command": "clear_errors"})

    # Final Status
    print("\n[Final Status]")
    telemetry = get_telemetry()
    print_status(telemetry)

    print("\n✅ Test sequence complete!")
    print("="*50 + "\n")

if __name__ == "__main__":
    test_sequence()
