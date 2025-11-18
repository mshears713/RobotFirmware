/**
 * @file WiFiManager.h
 * @brief Wi-Fi connectivity management for ESP32
 *
 * EDUCATIONAL NOTE: Wi-Fi is essential for IoT and robotics applications,
 * enabling remote monitoring, control, and data collection.
 *
 * ESP32 Wi-Fi Capabilities:
 * - 2.4GHz 802.11 b/g/n (Wi-Fi 4)
 * - Station mode (connects to existing network)
 * - Access Point mode (creates its own network)
 * - Station+AP mode (both simultaneously)
 * - Wi-Fi Direct (peer-to-peer)
 *
 * Station Mode (STA):
 * - ESP32 connects to existing Wi-Fi router
 * - Receives IP address from DHCP
 * - Can communicate with other devices on network
 * - Most common mode for IoT applications
 *
 * This class manages Wi-Fi connection lifecycle, handles reconnection,
 * and provides connection status monitoring.
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include "RobotSubsystem.h"
#include "config.h"

/**
 * @class WiFiManager
 * @brief Manages ESP32 Wi-Fi connectivity in station mode
 *
 * EDUCATIONAL: This class demonstrates Wi-Fi lifecycle management:
 * - Connection establishment
 * - Connection monitoring
 * - Automatic reconnection on failure
 * - Signal strength monitoring
 * - IP address management
 */
class WiFiManager : public RobotSubsystem {
public:
    /**
     * @brief Connection status enum
     *
     * EDUCATIONAL: Using enums makes code more readable than magic numbers.
     */
    enum ConnectionStatus {
        DISCONNECTED = 0,
        CONNECTING = 1,
        CONNECTED = 2,
        FAILED = 3
    };

    /**
     * @brief Construct WiFiManager
     *
     * EDUCATIONAL: Constructor stores credentials but doesn't connect yet.
     * Actual connection happens in initialize().
     *
     * @param ssid Wi-Fi network name
     * @param password Wi-Fi password
     * @param timeout Connection timeout in milliseconds
     */
    WiFiManager(const char* ssid = WIFI_SSID,
                const char* password = WIFI_PASSWORD,
                unsigned long timeout = WIFI_TIMEOUT)
        : _ssid(ssid),
          _password(password),
          _timeout(timeout),
          _status(DISCONNECTED),
          _lastConnectionAttempt(0),
          _reconnectInterval(5000),  // Try reconnect every 5 seconds
          _connectionAttempts(0),
          _rssi(0)
    {
        // Constructor initializes configuration only
    }

    /**
     * @brief Initialize Wi-Fi and connect to network
     *
     * EDUCATIONAL: This method:
     * 1. Sets Wi-Fi mode to station (STA)
     * 2. Initiates connection to specified SSID
     * 3. Waits for connection (with timeout)
     * 4. Retrieves IP address from DHCP
     *
     * @return true if connection successful
     */
    bool initialize() override {
        Serial.println("[WiFiManager] Initializing Wi-Fi...");

        // Set Wi-Fi mode to Station (client mode)
        // EDUCATIONAL: Other modes: WIFI_AP (access point), WIFI_AP_STA (both)
        WiFi.mode(WIFI_STA);

        Serial.print("[WiFiManager] Wi-Fi mode set to STATION");
        Serial.println();

        // Attempt to connect
        bool connected = connect();

        if (connected) {
            _isInitialized = true;
            return true;
        } else {
            Serial.println("[WiFiManager] ⚠ Initial connection failed");
            Serial.println("[WiFiManager] Will retry in background");
            _isInitialized = true;  // Still mark as initialized
            return false;
        }
    }

    /**
     * @brief Update Wi-Fi status and handle reconnection
     *
     * EDUCATIONAL: This is called every loop iteration to:
     * - Monitor connection status
     * - Attempt reconnection if disconnected
     * - Update signal strength (RSSI)
     */
    void update() override {
        if (!_isInitialized || !_isEnabled) {
            return;
        }

        // Update current status
        updateStatus();

        // Handle reconnection if disconnected
        if (_status == DISCONNECTED || _status == FAILED) {
            unsigned long now = millis();
            if (now - _lastConnectionAttempt >= _reconnectInterval) {
                Serial.println("[WiFiManager] Attempting reconnection...");
                connect();
                _lastConnectionAttempt = now;
            }
        }

        // Update RSSI (signal strength) periodically
        if (_status == CONNECTED && hasIntervalElapsed(10000)) {
            _rssi = WiFi.RSSI();
            updateTimestamp();
        }
    }

    /**
     * @brief Check if Wi-Fi is connected
     *
     * @return true if connected to network
     */
    bool isReady() override {
        return _isInitialized && (_status == CONNECTED);
    }

    /**
     * @brief Get status string
     *
     * @return Status with IP address and signal strength
     */
    String getStatus() override {
        if (!_isInitialized) {
            return "WiFiManager: Not initialized";
        }

        char buffer[128];
        switch (_status) {
            case CONNECTED:
                snprintf(buffer, sizeof(buffer),
                         "WiFiManager: Connected to '%s' (%s, RSSI: %d dBm)",
                         _ssid, getIPAddress().c_str(), _rssi);
                break;
            case CONNECTING:
                snprintf(buffer, sizeof(buffer),
                         "WiFiManager: Connecting to '%s'...", _ssid);
                break;
            case FAILED:
                snprintf(buffer, sizeof(buffer),
                         "WiFiManager: Connection failed (attempts: %d)", _connectionAttempts);
                break;
            case DISCONNECTED:
                snprintf(buffer, sizeof(buffer),
                         "WiFiManager: Disconnected");
                break;
        }
        return String(buffer);
    }

    /**
     * @brief Get subsystem name
     */
    const char* getName() const override {
        return "WiFiManager";
    }

    /**
     * @brief Get current IP address
     *
     * EDUCATIONAL: IP address is assigned by DHCP server (usually your router).
     * Format: xxx.xxx.xxx.xxx (e.g., 192.168.1.100)
     *
     * @return IP address as string, or "0.0.0.0" if not connected
     */
    String getIPAddress() const {
        if (_status == CONNECTED) {
            return WiFi.localIP().toString();
        }
        return "0.0.0.0";
    }

    /**
     * @brief Get signal strength (RSSI)
     *
     * EDUCATIONAL: RSSI (Received Signal Strength Indicator) in dBm:
     * - -30 dBm = Excellent signal (very close to router)
     * - -50 dBm = Good signal
     * - -60 dBm = Fair signal
     * - -70 dBm = Weak signal (may have issues)
     * - -80 dBm = Very weak (unreliable)
     * - -90 dBm = Barely usable
     *
     * @return Signal strength in dBm
     */
    int getRSSI() const {
        return _rssi;
    }

    /**
     * @brief Get connection status
     *
     * @return Current connection status
     */
    ConnectionStatus getConnectionStatus() const {
        return _status;
    }

    /**
     * @brief Get SSID (network name)
     *
     * @return Network SSID
     */
    const char* getSSID() const {
        return _ssid;
    }

    /**
     * @brief Get number of connection attempts
     *
     * @return Connection attempt counter
     */
    unsigned long getConnectionAttempts() const {
        return _connectionAttempts;
    }

    /**
     * @brief Disconnect from Wi-Fi
     *
     * EDUCATIONAL: Gracefully disconnect from network.
     * Useful for power saving or network switching.
     */
    void disconnect() {
        if (_status == CONNECTED) {
            WiFi.disconnect();
            _status = DISCONNECTED;
            Serial.println("[WiFiManager] Disconnected from Wi-Fi");
        }
    }

    /**
     * @brief Manually trigger reconnection
     *
     * @return true if connection successful
     */
    bool reconnect() {
        Serial.println("[WiFiManager] Manual reconnection requested");
        disconnect();
        delay(100);
        return connect();
    }

private:
    /**
     * @brief Attempt to connect to Wi-Fi network
     *
     * EDUCATIONAL: Connection process:
     * 1. WiFi.begin() starts connection attempt
     * 2. Wait for WL_CONNECTED status (with timeout)
     * 3. Retrieve IP address if successful
     *
     * @return true if connection successful
     */
    bool connect() {
        _connectionAttempts++;
        _status = CONNECTING;

        Serial.print("[WiFiManager] Connecting to '");
        Serial.print(_ssid);
        Serial.println("'...");

        // EDUCATIONAL: WiFi.begin() initiates connection to specified network
        // ESP32 will attempt authentication and DHCP in background
        WiFi.begin(_ssid, _password);

        // Wait for connection with timeout
        unsigned long startTime = millis();
        while (WiFi.status() != WL_CONNECTED) {
            if (millis() - startTime >= _timeout) {
                Serial.println("[WiFiManager] ✗ Connection timeout");
                _status = FAILED;
                return false;
            }

            delay(500);
            Serial.print(".");
        }

        Serial.println();

        // Connection successful
        _status = CONNECTED;
        _rssi = WiFi.RSSI();

        Serial.println("[WiFiManager] ✓ Connected!");
        Serial.print("[WiFiManager] IP Address: ");
        Serial.println(WiFi.localIP());
        Serial.print("[WiFiManager] Signal Strength: ");
        Serial.print(_rssi);
        Serial.println(" dBm");
        Serial.print("[WiFiManager] MAC Address: ");
        Serial.println(WiFi.macAddress());

        return true;
    }

    /**
     * @brief Update connection status based on WiFi library status
     *
     * EDUCATIONAL: WiFi.status() returns current connection state:
     * - WL_CONNECTED: Successfully connected
     * - WL_DISCONNECTED: Not connected
     * - WL_CONNECT_FAILED: Connection attempt failed
     * - WL_CONNECTION_LOST: Was connected, now lost
     * - WL_IDLE_STATUS: Idle, no connection attempt
     */
    void updateStatus() {
        wl_status_t wifiStatus = WiFi.status();

        switch (wifiStatus) {
            case WL_CONNECTED:
                if (_status != CONNECTED) {
                    _status = CONNECTED;
                    Serial.println("[WiFiManager] ✓ Connection established");
                }
                break;

            case WL_CONNECT_FAILED:
                if (_status != FAILED) {
                    _status = FAILED;
                    Serial.println("[WiFiManager] ✗ Connection failed");
                }
                break;

            case WL_CONNECTION_LOST:
            case WL_DISCONNECTED:
                if (_status != DISCONNECTED) {
                    _status = DISCONNECTED;
                    Serial.println("[WiFiManager] ⚠ Connection lost");
                }
                break;

            default:
                // WL_IDLE_STATUS or other states
                break;
        }
    }

    // Private member variables
    const char* _ssid;              ///< Wi-Fi network SSID
    const char* _password;          ///< Wi-Fi password
    unsigned long _timeout;         ///< Connection timeout (ms)
    ConnectionStatus _status;       ///< Current connection status
    unsigned long _lastConnectionAttempt;  ///< Last reconnection attempt time
    unsigned long _reconnectInterval;      ///< Time between reconnect attempts
    unsigned long _connectionAttempts;     ///< Connection attempt counter
    int _rssi;                      ///< Signal strength in dBm
};

#endif // WIFI_MANAGER_H
