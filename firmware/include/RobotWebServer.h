/**
 * @file RobotWebServer.h
 * @brief HTTP web server for robot telemetry and control
 *
 * EDUCATIONAL NOTE: Web servers enable remote interaction with embedded devices.
 * The ESP32 can run a lightweight HTTP server, making it accessible from:
 * - Web browsers
 * - Mobile apps
 * - Desktop applications (like our Streamlit UI)
 * - Other IoT devices
 *
 * REST API Design:
 * - GET /telemetry - Retrieve current robot state as JSON
 * - POST /command - Send control commands as JSON
 * - GET /status - Quick health check
 * - GET / - Optional: Serve basic web interface
 *
 * HTTP Basics:
 * - Request: Client asks for resource (GET, POST, PUT, DELETE)
 * - Response: Server sends data and status code (200=OK, 404=Not Found, etc.)
 * - Headers: Metadata about request/response (Content-Type, etc.)
 * - Body: Actual data payload (JSON in our case)
 *
 * This class wraps the ESP32 WebServer library and integrates with
 * our telemetry and command systems.
 */

#ifndef ROBOT_WEB_SERVER_H
#define ROBOT_WEB_SERVER_H

#include <Arduino.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "RobotSubsystem.h"
#include "TelemetryData.h"
#include "config.h"

/**
 * @class RobotWebServer
 * @brief HTTP server providing REST API for robot control and monitoring
 *
 * EDUCATIONAL: This demonstrates building a RESTful API on embedded hardware.
 * REST (Representational State Transfer) principles:
 * - Stateless: Each request is independent
 * - Resource-based: URLs represent resources (/telemetry, /command)
 * - Standard methods: GET (retrieve), POST (create/update), etc.
 * - JSON format: Human-readable, widely supported
 */
class RobotWebServer : public RobotSubsystem {
public:
    /**
     * @brief Command callback function type
     *
     * EDUCATIONAL: Function pointers allow flexible callback registration.
     * The main code can register a handler without WebServer knowing details.
     */
    typedef void (*CommandCallback)(const JsonDocument& command);

    /**
     * @brief Telemetry provider function type
     *
     * EDUCATIONAL: Function returns telemetry data when requested.
     */
    typedef TelemetryData (*TelemetryProvider)();

    /**
     * @brief Construct RobotWebServer
     *
     * @param port HTTP server port (default 80)
     */
    RobotWebServer(int port = HTTP_SERVER_PORT)
        : _server(port),
          _port(port),
          _requestCount(0),
          _telemetryProvider(nullptr),
          _commandCallback(nullptr)
    {
        // Constructor initializes server object
    }

    /**
     * @brief Initialize web server and register endpoints
     *
     * EDUCATIONAL: This method sets up HTTP route handlers.
     * Each route (URL path) is mapped to a handler function.
     *
     * @return true if initialization successful
     */
    bool initialize() override {
        Serial.println("[WebServer] Initializing HTTP server...");

        // Register endpoint handlers
        // EDUCATIONAL: Lambda functions capture 'this' pointer to access members
        _server.on("/", HTTP_GET, [this]() {
            handleRoot();
        });

        _server.on("/telemetry", HTTP_GET, [this]() {
            handleTelemetry();
        });

        _server.on("/command", HTTP_POST, [this]() {
            handleCommand();
        });

        _server.on("/command", HTTP_OPTIONS, [this]() {
            handleCORS();
        });

        _server.on("/status", HTTP_GET, [this]() {
            handleStatus();
        });

        // Handle 404 Not Found
        _server.onNotFound([this]() {
            handleNotFound();
        });

        // Start server
        _server.begin();

        Serial.print("[WebServer] ✓ HTTP server started on port ");
        Serial.println(_port);
        Serial.println("[WebServer] Available endpoints:");
        Serial.println("[WebServer]   GET  /           - Root (info page)");
        Serial.println("[WebServer]   GET  /telemetry  - Robot telemetry (JSON)");
        Serial.println("[WebServer]   POST /command    - Send command (JSON)");
        Serial.println("[WebServer]   GET  /status     - Server status");

        _isInitialized = true;
        return true;
    }

    /**
     * @brief Update server (handle incoming requests)
     *
     * EDUCATIONAL: Must be called frequently to process HTTP requests.
     * The WebServer library handles request parsing, but we need to
     * call handleClient() to trigger our endpoint handlers.
     */
    void update() override {
        if (!_isInitialized || !_isEnabled) {
            return;
        }

        // Process any pending HTTP requests
        // EDUCATIONAL: handleClient() checks for new requests and calls
        // registered handler functions (handleTelemetry, handleCommand, etc.)
        _server.handleClient();
    }

    /**
     * @brief Check if server is ready
     */
    bool isReady() override {
        return _isInitialized;
    }

    /**
     * @brief Get status string
     */
    String getStatus() override {
        if (!_isInitialized) {
            return "WebServer: Not initialized";
        }

        char buffer[128];
        snprintf(buffer, sizeof(buffer),
                 "WebServer: Running on port %d [requests: %lu]",
                 _port, _requestCount);
        return String(buffer);
    }

    /**
     * @brief Get subsystem name
     */
    const char* getName() const override {
        return "WebServer";
    }

    /**
     * @brief Register telemetry provider function
     *
     * EDUCATIONAL: Dependency injection pattern - we don't hardcode where
     * telemetry comes from. Main code provides the function.
     *
     * @param provider Function that returns telemetry data
     */
    void setTelemetryProvider(TelemetryProvider provider) {
        _telemetryProvider = provider;
        Serial.println("[WebServer] Telemetry provider registered");
    }

    /**
     * @brief Register command callback function
     *
     * @param callback Function called when command received
     */
    void setCommandCallback(CommandCallback callback) {
        _commandCallback = callback;
        Serial.println("[WebServer] Command callback registered");
    }

    /**
     * @brief Get total request count
     */
    unsigned long getRequestCount() const {
        return _requestCount;
    }

private:
    /**
     * @brief Handle GET / (root endpoint)
     *
     * EDUCATIONAL: Returns HTML info page viewable in web browser.
     * Demonstrates serving different content types (HTML vs JSON).
     */
    void handleRoot() {
        _requestCount++;

        String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 Robot API</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 40px; }
        h1 { color: #333; }
        code { background: #f4f4f4; padding: 2px 6px; border-radius: 3px; }
        ul { line-height: 1.8; }
    </style>
</head>
<body>
    <h1>🤖 ESP32 Robotics API</h1>
    <p>Welcome to the ESP32 Robot REST API server.</p>

    <h2>Available Endpoints:</h2>
    <ul>
        <li><code>GET /telemetry</code> - Retrieve current robot telemetry (JSON)</li>
        <li><code>POST /command</code> - Send control command (JSON)</li>
        <li><code>GET /status</code> - Server health check</li>
    </ul>

    <h2>Example Usage:</h2>
    <h3>Get Telemetry:</h3>
    <pre>curl http://)" + String(WiFi.localIP().toString()) + R"(/telemetry</pre>

    <h3>Send Command:</h3>
    <pre>curl -X POST http://)" + String(WiFi.localIP().toString()) + R"(/command \
  -H "Content-Type: application/json" \
  -d '{"command":"set_servo_angle","value":90}'</pre>

    <p><em>Powered by ESP32 | WALL-E & EVE Apprenticeship</em></p>
</body>
</html>
)";

        _server.send(200, "text/html", html);
    }

    /**
     * @brief Handle GET /telemetry
     *
     * EDUCATIONAL: This is the main data endpoint. Returns JSON telemetry
     * that the Streamlit UI will poll periodically for live updates.
     */
    void handleTelemetry() {
        _requestCount++;

        // Set CORS headers for cross-origin requests
        _server.sendHeader("Access-Control-Allow-Origin", "*");
        _server.sendHeader("Access-Control-Allow-Methods", "GET");

        if (_telemetryProvider != nullptr) {
            // Get current telemetry from provider function
            TelemetryData telemetry = _telemetryProvider();

            // Convert to JSON and send
            String json = telemetry.toJson();
            _server.send(200, "application/json", json);

            if (DEBUG_VERBOSE) {
                Serial.println("[WebServer] Telemetry request served");
            }
        } else {
            // No provider registered
            _server.send(500, "application/json",
                        "{\"error\":\"Telemetry provider not registered\"}");
            Serial.println("[WebServer] ✗ Telemetry request failed - no provider");
        }
    }

    /**
     * @brief Handle POST /command
     *
     * EDUCATIONAL: Receives JSON commands from UI and forwards to callback.
     * Command format: {"command": "action_name", "value": parameter}
     */
    void handleCommand() {
        _requestCount++;

        // Set CORS headers
        _server.sendHeader("Access-Control-Allow-Origin", "*");
        _server.sendHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
        _server.sendHeader("Access-Control-Allow-Headers", "Content-Type");

        if (!_server.hasArg("plain")) {
            // No request body
            _server.send(400, "application/json",
                        "{\"error\":\"Missing request body\"}");
            return;
        }

        // Get request body (JSON string)
        String body = _server.arg("plain");

        // Parse JSON
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, body);

        if (error) {
            // JSON parsing failed
            String response = "{\"error\":\"Invalid JSON: ";
            response += error.c_str();
            response += "\"}";
            _server.send(400, "application/json", response);
            Serial.print("[WebServer] ✗ Invalid JSON in command: ");
            Serial.println(error.c_str());
            return;
        }

        // JSON parsed successfully
        if (_commandCallback != nullptr) {
            // Forward to command handler
            _commandCallback(doc);

            // Send success response
            _server.send(200, "application/json",
                        "{\"status\":\"ok\",\"message\":\"Command received\"}");

            Serial.print("[WebServer] ✓ Command executed: ");
            Serial.println(body);
        } else {
            _server.send(500, "application/json",
                        "{\"error\":\"Command callback not registered\"}");
            Serial.println("[WebServer] ✗ Command failed - no callback");
        }
    }

    /**
     * @brief Handle OPTIONS /command (CORS preflight)
     *
     * EDUCATIONAL: CORS (Cross-Origin Resource Sharing) allows web pages
     * from different domains to access this API. Browsers send OPTIONS
     * requests first to check permissions.
     */
    void handleCORS() {
        _server.sendHeader("Access-Control-Allow-Origin", "*");
        _server.sendHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
        _server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
        _server.send(200, "text/plain", "");
    }

    /**
     * @brief Handle GET /status
     *
     * EDUCATIONAL: Quick health check endpoint. Returns minimal info
     * to verify server is responsive.
     */
    void handleStatus() {
        _requestCount++;

        _server.sendHeader("Access-Control-Allow-Origin", "*");

        StaticJsonDocument<128> doc;
        doc["status"] = "ok";
        doc["uptime"] = millis() / 1000;
        doc["requests"] = _requestCount;
        doc["free_heap"] = ESP.getFreeHeap();

        String response;
        serializeJson(doc, response);
        _server.send(200, "application/json", response);
    }

    /**
     * @brief Handle 404 Not Found
     *
     * EDUCATIONAL: Called when client requests non-existent endpoint.
     */
    void handleNotFound() {
        _requestCount++;

        String message = "{\"error\":\"Endpoint not found\",\"path\":\"";
        message += _server.uri();
        message += "\"}";

        _server.send(404, "application/json", message);

        Serial.print("[WebServer] 404 Not Found: ");
        Serial.println(_server.uri());
    }

    // Private member variables
    WebServer _server;              ///< ESP32 WebServer instance
    int _port;                      ///< HTTP server port
    unsigned long _requestCount;    ///< Total requests served
    TelemetryProvider _telemetryProvider;  ///< Telemetry data source
    CommandCallback _commandCallback;      ///< Command handler
};

#endif // ROBOT_WEB_SERVER_H
