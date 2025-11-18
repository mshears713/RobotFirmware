/**
 * @file main.cpp
 * @brief Main firmware entry point for ESP32 Robotics Project
 *
 * ESP32 Robotics Firmware - WALL-E & EVE Apprenticeship
 * Phase 1: Foundations & Environment Setup
 *
 * EDUCATIONAL NOTE: This is the heart of the embedded firmware. The structure
 * follows the standard Arduino pattern:
 * 1. Global declarations and includes
 * 2. setup() - runs once at startup
 * 3. loop() - runs continuously forever
 *
 * Key concepts demonstrated:
 * - GPIO output control (LED blinking)
 * - GPIO input handling (button reading)
 * - Serial communication for debugging
 * - Non-blocking timing (millis() instead of delay())
 * - Object-oriented subsystem architecture
 * - Main loop with cooperative multitasking
 */

#include <Arduino.h>
#include "config.h"
#include "RobotSubsystem.h"
#include "ServoArm.h"

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

/**
 * EDUCATIONAL NOTE: Global variables in embedded systems
 *
 * In embedded firmware, global variables are common and acceptable because:
 * 1. There's only one "thread" of execution (single-core operation)
 * 2. Hardware resources are singular (only one servo on a given pin)
 * 3. Memory is limited - avoiding stack allocation overhead
 *
 * However, we should still minimize their use and prefer encapsulation
 * in classes when possible.
 */

// LED state tracking
bool ledState = false;                    ///< Current LED state (on/off)
unsigned long lastLedToggle = 0;          ///< Timestamp of last LED toggle

// Button input tracking
bool lastButtonState = HIGH;              ///< Previous button state
bool currentButtonState = HIGH;           ///< Current button state
unsigned long lastDebounceTime = 0;       ///< Last time button state changed
const unsigned long debounceDelay = 50;   ///< Debounce time in milliseconds

// Debug output timing
unsigned long lastDebugPrint = 0;         ///< Timestamp of last debug output

// Subsystem instances
/**
 * EDUCATIONAL: We use pointers to subsystems for several reasons:
 * 1. Allows polymorphism (RobotSubsystem* can point to any derived class)
 * 2. Enables dynamic creation in setup() after Serial is initialized
 * 3. Can be nullptr to indicate "not created yet"
 */
ServoArm* servoArm = nullptr;             ///< Pointer to servo arm subsystem

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

void setupSerial();
void setupGPIO();
void setupSubsystems();
void updateLED();
void updateButton();
void printDebugInfo();

// ============================================================================
// SETUP FUNCTION
// ============================================================================

/**
 * @brief One-time initialization function
 *
 * EDUCATIONAL: setup() runs once when the ESP32 powers on or resets.
 * This is where we configure hardware, initialize subsystems, and prepare
 * for the main loop. Order matters!
 *
 * Typical setup order:
 * 1. Serial communication (so we can see debug output)
 * 2. GPIO pins
 * 3. Subsystems (sensors, actuators)
 * 4. Network (Wi-Fi) - in later phases
 */
void setup() {
    // Step 1: Initialize serial communication
    // EDUCATIONAL: Do this FIRST so we can see debug output from other init steps
    setupSerial();

    Serial.println("\n\n");
    Serial.println("========================================");
    Serial.println("  ESP32 Robotics Firmware");
    Serial.println("  WALL-E & EVE Apprenticeship");
    Serial.println("  Phase 1: Foundations");
    Serial.println("========================================");
    Serial.println();

    // Step 2: Configure GPIO pins
    setupGPIO();

    // Step 3: Initialize subsystems
    setupSubsystems();

    // Initialization complete
    Serial.println();
    Serial.println("========================================");
    Serial.println("  Initialization Complete!");
    Serial.println("  Entering main loop...");
    Serial.println("========================================");
    Serial.println();
}

// ============================================================================
// MAIN LOOP FUNCTION
// ============================================================================

/**
 * @brief Main program loop - runs continuously
 *
 * EDUCATIONAL: The loop() function runs forever in a tight cycle.
 * On ESP32, it typically executes thousands of times per second.
 *
 * CRITICAL RULE: Never use delay() in loop()!
 * - delay() blocks everything
 * - Wi-Fi connection will drop
 * - Sensors won't update
 * - System appears frozen
 *
 * Instead, use millis() for non-blocking timing (see updateLED example).
 *
 * Main loop responsibilities:
 * 1. Update all subsystems (servos, sensors, motors)
 * 2. Handle user input (buttons, serial commands)
 * 3. Process network requests (in later phases)
 * 4. Perform periodic tasks (LED blinking, debug output)
 */
void loop() {
    // Update LED (non-blocking blink)
    updateLED();

    // Check button input
    updateButton();

    // Update all subsystems
    if (servoArm != nullptr && servoArm->isInitialized()) {
        servoArm->update();
    }

    // Periodic debug output
    if (DEBUG_VERBOSE && DEBUG_SUBSYSTEM_STATUS) {
        unsigned long now = millis();
        if (now - lastDebugPrint >= DEBUG_PRINT_INTERVAL) {
            printDebugInfo();
            lastDebugPrint = now;
        }
    }

    // EDUCATIONAL: Small delay to prevent watchdog timeout
    // This is one of the few acceptable uses of delay() - it gives the
    // ESP32's background tasks (Wi-Fi, Bluetooth) time to run.
    // 10ms delay still allows 100 loop iterations per second.
    delay(10);
}

// ============================================================================
// INITIALIZATION FUNCTIONS
// ============================================================================

/**
 * @brief Initialize serial communication for debugging
 *
 * EDUCATIONAL: Serial communication lets us send debug messages from the
 * ESP32 to a computer over USB. This is invaluable for development!
 *
 * The baud rate (115200) must match:
 * - Serial.begin() value here
 * - monitor_speed in platformio.ini
 * - Your serial monitor settings
 *
 * If you see garbage characters, baud rate mismatch is the likely cause.
 */
void setupSerial() {
    Serial.begin(SERIAL_BAUD_RATE);

    // EDUCATIONAL: Wait for serial port to connect
    // On native USB boards (ESP32), this ensures Serial is ready.
    // Timeout after 3 seconds so we don't block if no monitor is connected.
    unsigned long startTime = millis();
    while (!Serial && (millis() - startTime < 3000)) {
        ; // Wait
    }

    Serial.println();
    Serial.println("[INIT] Serial communication initialized");
    Serial.print("[INIT] Baud rate: ");
    Serial.println(SERIAL_BAUD_RATE);
}

/**
 * @brief Configure GPIO pins for LEDs and buttons
 *
 * EDUCATIONAL: GPIO (General Purpose Input/Output) pins can be configured as:
 * - OUTPUT: We control the pin (HIGH = 3.3V, LOW = 0V)
 * - INPUT: We read the pin state from external source
 * - INPUT_PULLUP: Input with internal pull-up resistor (pin reads HIGH by default)
 */
void setupGPIO() {
    Serial.println("[INIT] Configuring GPIO pins...");

    // Configure LED pins as outputs
    // EDUCATIONAL: pinMode() configures the pin direction
    pinMode(LED_BUILTIN_PIN, OUTPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);

    // Start with LEDs off
    // EDUCATIONAL: digitalWrite() sets output pin HIGH (3.3V) or LOW (0V)
    digitalWrite(LED_BUILTIN_PIN, LOW);
    digitalWrite(STATUS_LED_PIN, LOW);

    Serial.print("[INIT]   - LED pins configured (");
    Serial.print(LED_BUILTIN_PIN);
    Serial.print(", ");
    Serial.print(STATUS_LED_PIN);
    Serial.println(")");

    // Configure button pin as input with pull-up
    // EDUCATIONAL: INPUT_PULLUP enables an internal resistor that pulls
    // the pin to HIGH (3.3V) when not connected to anything.
    // When button is pressed (connecting pin to GND), pin reads LOW.
    pinMode(BUTTON_INPUT_PIN, INPUT_PULLUP);

    Serial.print("[INIT]   - Button pin configured (");
    Serial.print(BUTTON_INPUT_PIN);
    Serial.println(") with pull-up");

    Serial.println("[INIT] ✓ GPIO configuration complete");
}

/**
 * @brief Initialize robot subsystems
 *
 * EDUCATIONAL: We create and initialize subsystem objects here.
 * Each subsystem follows the same pattern:
 * 1. Create object (allocate memory)
 * 2. Call initialize() method
 * 3. Check return value for success/failure
 * 4. Log results
 */
void setupSubsystems() {
    Serial.println("[INIT] Initializing subsystems...");

    // Create servo arm subsystem
    // EDUCATIONAL: 'new' allocates memory on the heap and calls constructor
    servoArm = new ServoArm(SERVO_ARM_PIN);

    // Initialize the servo
    if (servoArm->initialize()) {
        Serial.println("[INIT] ✓ ServoArm subsystem ready");
    } else {
        Serial.println("[INIT] ✗ ServoArm initialization FAILED");
        // Continue anyway - other subsystems might still work
    }

    // Move servo to demonstrate it's working
    Serial.println("[INIT] Testing servo movement...");
    servoArm->setTargetAngle(45);   // Move to 45 degrees
    delay(500);                      // Wait for movement (OK in setup)
    servoArm->setTargetAngle(135);  // Move to 135 degrees
    delay(500);
    servoArm->setTargetAngle(90);   // Return to center
    delay(500);

    Serial.println("[INIT] ✓ Servo test complete");

    // Future subsystems will be added here:
    // - I2C sensors (Phase 2)
    // - SPI devices (Phase 2)
    // - Motor control (Phase 2)
    // - Wi-Fi (Phase 3)
}

// ============================================================================
// UPDATE FUNCTIONS
// ============================================================================

/**
 * @brief Update LED state (non-blocking blink)
 *
 * EDUCATIONAL: This demonstrates NON-BLOCKING timing using millis().
 * Instead of:
 *   digitalWrite(LED, HIGH);
 *   delay(500);              // BLOCKS for 500ms
 *   digitalWrite(LED, LOW);
 *   delay(500);              // BLOCKS for 500ms
 *
 * We use:
 *   Check if enough time has passed since last toggle
 *   If yes, toggle LED and record current time
 *   If no, do nothing and return immediately
 *
 * This allows other code to run while LED is blinking.
 */
void updateLED() {
    unsigned long currentTime = millis();

    // Check if blink interval has elapsed
    if (currentTime - lastLedToggle >= LED_BLINK_INTERVAL) {
        // Toggle LED state
        ledState = !ledState;

        // Update both LEDs
        digitalWrite(LED_BUILTIN_PIN, ledState ? HIGH : LOW);
        digitalWrite(STATUS_LED_PIN, ledState ? HIGH : LOW);

        // Record time of this toggle for next interval calculation
        lastLedToggle = currentTime;
    }
}

/**
 * @brief Read and debounce button input
 *
 * EDUCATIONAL: Button debouncing is crucial in embedded systems!
 * When you press a mechanical button, the contacts "bounce" - they make and
 * break connection rapidly for a few milliseconds. Without debouncing, one
 * button press might register as multiple presses.
 *
 * Software debouncing: Only register a state change if the button has been
 * in the new state for a minimum time (debounceDelay = 50ms).
 */
void updateButton() {
    // Read current button state
    // EDUCATIONAL: Remember INPUT_PULLUP means:
    // - HIGH when not pressed (pulled up to 3.3V)
    // - LOW when pressed (connected to GND)
    bool reading = digitalRead(BUTTON_INPUT_PIN);

    // Check if state has changed
    if (reading != lastButtonState) {
        // State changed - reset debounce timer
        lastDebounceTime = millis();
    }

    // If state has been stable for debounceDelay time
    if ((millis() - lastDebounceTime) > debounceDelay) {
        // State is stable - check if it's different from current state
        if (reading != currentButtonState) {
            currentButtonState = reading;

            // EDUCATIONAL: Button pressed event (HIGH→LOW transition with pullup)
            if (currentButtonState == LOW) {
                Serial.println();
                Serial.println("========================================");
                Serial.println("  🔘 BUTTON PRESSED!");
                Serial.println("========================================");

                // Demo: Move servo to random position when button pressed
                float randomAngle = random(0, 181);  // 0-180 degrees
                Serial.print("  Moving servo to ");
                Serial.print(randomAngle);
                Serial.println("°");

                if (servoArm != nullptr) {
                    servoArm->setTargetAngle(randomAngle);
                }
            }
        }
    }

    // Save reading for next iteration
    lastButtonState = reading;
}

/**
 * @brief Print debug information to serial
 *
 * EDUCATIONAL: Regular status output helps monitor system health and
 * understand what's happening inside the firmware. This is like a
 * "heartbeat" that proves the system is alive and working.
 */
void printDebugInfo() {
    Serial.println();
    Serial.println("========================================");
    Serial.println("  System Status");
    Serial.println("========================================");

    // Uptime
    unsigned long uptime = millis() / 1000;
    Serial.print("  Uptime: ");
    Serial.print(uptime);
    Serial.println(" seconds");

    // Free heap memory
    // EDUCATIONAL: ESP32 has limited RAM. Monitoring free heap helps detect
    // memory leaks (continuously decreasing) or fragmentation.
    Serial.print("  Free Heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");

    // Button state
    Serial.print("  Button: ");
    Serial.println(currentButtonState == LOW ? "PRESSED" : "Released");

    // LED state
    Serial.print("  LED: ");
    Serial.println(ledState ? "ON" : "OFF");

    // Subsystem status
    Serial.println();
    Serial.println("  Subsystems:");

    if (servoArm != nullptr) {
        Serial.print("    - ");
        Serial.println(servoArm->getStatus());
        Serial.print("      Ready: ");
        Serial.println(servoArm->isReady() ? "Yes" : "No");
        Serial.print("      At target: ");
        Serial.println(servoArm->hasReachedTarget() ? "Yes" : "No");
    }

    Serial.println("========================================");
    Serial.println();
}

// ============================================================================
// EDUCATIONAL NOTES - PHASE 1 COMPLETE
// ============================================================================

/**
 * CONGRATULATIONS! You've completed Phase 1 of the ESP32 Robotics Firmware.
 *
 * What you've learned:
 * ✓ PlatformIO project structure and configuration
 * ✓ GPIO output control (LED blinking)
 * ✓ GPIO input handling with debouncing
 * ✓ Serial communication for debugging
 * ✓ Non-blocking timing with millis()
 * ✓ Object-oriented subsystem architecture
 * ✓ PWM servo control basics
 * ✓ Main loop design pattern
 *
 * Key concepts mastered:
 * - setup() vs loop() execution model
 * - Why delay() is problematic in embedded systems
 * - Abstract base classes and inheritance
 * - Encapsulation and modularity
 * - Hardware initialization sequences
 * - Debugging with Serial output
 *
 * Next steps (Phase 2):
 * - I2C sensor integration (temperature, humidity, pressure)
 * - SPI communication for high-speed devices
 * - PWM motor control for locomotion
 * - Telemetry data structure
 *
 * Try these experiments:
 * 1. Change LED_BLINK_INTERVAL in config.h to speed up/slow down blinking
 * 2. Modify SERVO_TRANSITION_SPEED to make servo movements faster/slower
 * 3. Add a second ServoArm on a different pin
 * 4. Create your own RobotSubsystem subclass
 * 5. Add more button functions (double-click, long-press detection)
 *
 * Remember: The best way to learn embedded systems is to experiment!
 * Don't be afraid to break things - you can always reflash the firmware.
 */
