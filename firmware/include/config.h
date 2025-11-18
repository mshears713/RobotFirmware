/**
 * @file config.h
 * @brief Central configuration file for ESP32 Robotics Firmware
 *
 * EDUCATIONAL NOTE: Centralizing configuration in a header file makes the
 * firmware easier to maintain and adapt to different hardware setups.
 * All pin assignments, timing constants, and system parameters are defined here.
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// SERIAL CONFIGURATION
// ============================================================================

/**
 * @brief Serial baud rate for debugging output
 *
 * EDUCATIONAL: Baud rate is the speed of serial communication measured in
 * bits per second. 115200 is a common rate that balances speed with reliability.
 * This MUST match the monitor_speed setting in platformio.ini!
 */
#define SERIAL_BAUD_RATE 115200

// ============================================================================
// GPIO PIN ASSIGNMENTS
// ============================================================================

/**
 * @brief Built-in LED pin (usually GPIO 2 on ESP32 dev boards)
 *
 * EDUCATIONAL: The ESP32 development board typically has an LED connected
 * to GPIO 2. This is perfect for testing basic output control.
 */
#define LED_BUILTIN_PIN 2

/**
 * @brief External status LED pin
 *
 * EDUCATIONAL: You can connect an external LED with a current-limiting
 * resistor (220Ω-1kΩ) between this pin and GND for visual feedback.
 */
#define STATUS_LED_PIN 13

/**
 * @brief Example input button pin
 *
 * EDUCATIONAL: Connect a pushbutton between this pin and GND.
 * The internal pull-up resistor will be enabled, so the pin reads HIGH
 * when not pressed, and LOW when pressed.
 */
#define BUTTON_INPUT_PIN 0  // BOOT button on most ESP32 boards

// ============================================================================
// SERVO CONTROL PINS
// ============================================================================

/**
 * @brief Primary servo arm control pin
 *
 * EDUCATIONAL: PWM-capable GPIO pins on ESP32 can control servo motors.
 * Servos expect a 50Hz signal (20ms period) with pulse widths:
 * - 1ms (5% duty) = 0 degrees
 * - 1.5ms (7.5% duty) = 90 degrees
 * - 2ms (10% duty) = 180 degrees
 */
#define SERVO_ARM_PIN 18

/**
 * @brief Secondary servo pin (for future expansion)
 */
#define SERVO_EXTRA_PIN 19

// ============================================================================
// I2C CONFIGURATION (for Phase 2)
// ============================================================================

/**
 * @brief I2C SDA (Data) pin
 *
 * EDUCATIONAL: I2C uses two wires - SDA for data and SCL for clock.
 * These pins require 4.7kΩ pull-up resistors to 3.3V for reliable operation.
 */
#define I2C_SDA_PIN 21  // Default I2C SDA on ESP32

/**
 * @brief I2C SCL (Clock) pin
 */
#define I2C_SCL_PIN 22  // Default I2C SCL on ESP32

/**
 * @brief I2C bus frequency in Hz
 *
 * EDUCATIONAL: Standard I2C speed is 100kHz. Fast mode is 400kHz.
 * Start with standard mode for reliability.
 */
#define I2C_FREQUENCY 100000  // 100kHz

/**
 * @brief BME280 sensor I2C address
 *
 * EDUCATIONAL: I2C devices have 7-bit addresses. The BME280 sensor
 * can be addressed at 0x76 or 0x77 depending on its SDO pin connection.
 */
#define BME280_I2C_ADDRESS 0x76

// ============================================================================
// SPI CONFIGURATION (for Phase 2)
// ============================================================================

/**
 * @brief SPI MOSI (Master Out Slave In) pin
 */
#define SPI_MOSI_PIN 23  // Default SPI MOSI on ESP32

/**
 * @brief SPI MISO (Master In Slave Out) pin
 */
#define SPI_MISO_PIN 19  // Default SPI MISO on ESP32

/**
 * @brief SPI SCK (Clock) pin
 */
#define SPI_SCK_PIN 18   // Default SPI SCK on ESP32

/**
 * @brief SPI CS (Chip Select) pin
 */
#define SPI_CS_PIN 5     // Configurable CS pin

// ============================================================================
// MOTOR CONTROL PINS (for Phase 2)
// ============================================================================

/**
 * @brief Motor A PWM control pin
 *
 * EDUCATIONAL: DC motors use PWM for speed control. Higher duty cycle = faster.
 * Direction is controlled by the logic levels on DIR pins.
 */
#define MOTOR_A_PWM_PIN 25

/**
 * @brief Motor A direction pin
 */
#define MOTOR_A_DIR_PIN 26

/**
 * @brief Motor B PWM control pin
 */
#define MOTOR_B_PWM_PIN 27

/**
 * @brief Motor B direction pin
 */
#define MOTOR_B_DIR_PIN 14

// ============================================================================
// TIMING CONSTANTS
// ============================================================================

/**
 * @brief LED blink interval in milliseconds
 *
 * EDUCATIONAL: Using constants instead of magic numbers makes code readable
 * and easy to tune. 500ms = 0.5 second on, 0.5 second off = 1Hz blink rate.
 */
#define LED_BLINK_INTERVAL 500

/**
 * @brief Main loop update interval in milliseconds
 *
 * EDUCATIONAL: This defines how often subsystems are updated.
 * 20ms = 50Hz update rate, which is good for servo and sensor control.
 */
#define MAIN_LOOP_INTERVAL 20

/**
 * @brief Sensor read interval in milliseconds
 *
 * EDUCATIONAL: Sensors don't need to be read as frequently as servos.
 * 1000ms = 1 second is appropriate for temperature/humidity monitoring.
 */
#define SENSOR_READ_INTERVAL 1000

/**
 * @brief Telemetry transmission interval in milliseconds
 *
 * EDUCATIONAL: How often to send telemetry data over Wi-Fi.
 * 100ms = 10Hz provides smooth real-time visualization without overwhelming the network.
 */
#define TELEMETRY_INTERVAL 100

// ============================================================================
// SERVO PARAMETERS
// ============================================================================

/**
 * @brief Minimum servo angle in degrees
 */
#define SERVO_MIN_ANGLE 0.0f

/**
 * @brief Maximum servo angle in degrees
 */
#define SERVO_MAX_ANGLE 180.0f

/**
 * @brief Default servo angle in degrees (centered position)
 */
#define SERVO_DEFAULT_ANGLE 90.0f

/**
 * @brief Servo transition speed in degrees per update
 *
 * EDUCATIONAL: This creates smooth servo movements instead of instant jumps.
 * At 50Hz update rate, 2.0 deg/update = 100 deg/second transition speed.
 */
#define SERVO_TRANSITION_SPEED 2.0f

// ============================================================================
// WIFI CONFIGURATION (for Phase 3)
// ============================================================================

/**
 * @brief Wi-Fi network SSID (network name)
 *
 * EDUCATIONAL: Change this to match your Wi-Fi network name.
 * IMPORTANT: ESP32 only supports 2.4GHz networks, not 5GHz!
 */
#define WIFI_SSID "YourWiFiSSID"

/**
 * @brief Wi-Fi network password
 *
 * EDUCATIONAL: Store your Wi-Fi password here. In production systems,
 * credentials should be stored securely or configured via a setup interface.
 */
#define WIFI_PASSWORD "YourWiFiPassword"

/**
 * @brief Wi-Fi connection timeout in milliseconds
 */
#define WIFI_TIMEOUT 10000  // 10 seconds

/**
 * @brief HTTP server port
 *
 * EDUCATIONAL: Port 80 is the standard HTTP port. The ESP32 will listen
 * on this port for telemetry requests and commands from the Streamlit UI.
 */
#define HTTP_SERVER_PORT 80

// ============================================================================
// DEBUG SETTINGS
// ============================================================================

/**
 * @brief Enable verbose debug output
 *
 * EDUCATIONAL: Set to true during development to see detailed Serial output.
 * Set to false in production to reduce clutter and improve performance.
 */
#define DEBUG_VERBOSE true

/**
 * @brief Enable subsystem status reporting
 */
#define DEBUG_SUBSYSTEM_STATUS true

/**
 * @brief Debug output interval in milliseconds
 */
#define DEBUG_PRINT_INTERVAL 2000  // Print debug info every 2 seconds

// ============================================================================
// SYSTEM LIMITS
// ============================================================================

/**
 * @brief Maximum number of registered subsystems
 */
#define MAX_SUBSYSTEMS 10

/**
 * @brief Watchdog timeout in milliseconds
 *
 * EDUCATIONAL: If the main loop takes longer than this, the ESP32 will reset.
 * This prevents the system from hanging indefinitely.
 */
#define WATCHDOG_TIMEOUT 5000

/**
 * @brief Low battery voltage threshold in millivolts
 */
#define LOW_BATTERY_THRESHOLD 6500  // 6.5V for 2S LiPo

#endif // CONFIG_H
