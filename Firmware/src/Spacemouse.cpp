/*****************************************************************************************
 *
 * Space Mushroom / Hall Effect 3D Mouse
 *
 * This code emulates a 3DConnexion SpaceMouse using an Arduino Pro Micro (atmega32u4),
 * Hall Effect Sensors, and a custom HID descriptor.
 *
 * Purpose: To provide a 6-Degrees-of-Freedom (6-DOF) input device for CAD and 3D software.
 *
 * Original works and contributions by:
 * - Shiura (Original Space Mushroom design)
 * - jfedor (3DConnexion emulation code)
 * - BennyBWalker (Remix of sketches)
 * - fdmakara (Four joystick logic)
 * - Teaching Tech (Consolidation and extensive debugging)
 * - John Crombie (*JC - Hall Effect sensor implementation and button state machine)
 *
 * Refactored by: Alexander Grauer with use of Google Gemini 
 *
 *****************************************************************************************/

#include "HID.h"

//========================================================================================
//                                  USER CONFIGURATION
//========================================================================================

// --- DEBUGGING ---
// Set via `build_flags` in platformio.ini, e.g., -D DEBUG_LEVEL=1
// If not set, it defaults to DEBUG_NONE (0).
#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL 0
#endif

// To provide context, the available levels are:
enum DebugLevel {
    DEBUG_NONE = 0,
    DEBUG_RAW_SENSORS = 1,      // Raw 10-bit ADC values (0-1023)
    DEBUG_CENTERED_SENSORS = 2, // Sensor values with center point subtracted
    DEBUG_DEADZONE_FILTERED = 3,// Centered values with deadzone applied
    DEBUG_AXIS_VALUES = 4,      // Final calculated translation/rotation values
    DEBUG_SIDE_BY_SIDE = 5,     // Filtered sensor values and final axis values
    DEBUG_BUTTON_STATE = 6      // Info for the button state machine
};

// --- SERIAL MONITOR SPEED ---
// Set via `build_flags` in platformio.ini, e.g., -D MONITOR_SPEED=115200
#ifndef MONITOR_SPEED
#define MONITOR_SPEED 115200
#endif

// --- MOVEMENT BEHAVIOR ---

// 3DCONNEXION vs. TEACHING TECH MOVEMENT
// true:  Push/pull for Z-axis pan (up/down), lift/press for Y-axis pan (zoom).
// false: Push/pull for Y-axis pan (up/down), lift/press for Z-axis pan (zoom). (Teaching Tech Default)
const bool use3DConnexionMovement = true;

// AXIS INVERSION
// Set to 'true' to invert the direction of an axis.
const bool invertX = false;     // Pan left/right
const bool invertY = false;     // Pan up/down (or Zoom if use3DConnexionMovement is false)
const bool invertZ = false;     // Zoom (or Pan up/down if use3DConnexionMovement is false)
const bool invertRX = false;    // Tilt front/back
const bool invertRY = false;    // Tilt left/right
const bool invertRZ = false;     // Twist clockwise/counter-clockwise (Original code reversed this by default)

// SENSITIVITY
// Adjusts the overall speed. 100 is default, 50 is half speed, etc.
const int16_t speedPercentage = 100;

// DEADZONE
// Ignores minor sensor drift around the center point. Increase if you see ghost movements.
const int DEADZONE = 30;


//========================================================================================
//                                   HARDWARE PINS
//========================================================================================

// Analog pins for the eight Hall Effect Sensors (HES)
const uint8_t HES_PINS[8] = {A0, A1, A2, A3, A6, A7, A8, A9};
enum SensorIndex {
    HES_6_LEFT   = 0, // A0
    HES_6_RIGHT  = 1, // A1
    HES_3_NEAR   = 2, // A2
    HES_3_FAR    = 3, // A3
    HES_12_RIGHT = 4, // A6
    HES_12_LEFT  = 5, // A7
    HES_9_FAR    = 6, // A8
    HES_9_NEAR   = 7  // A9
};

// Digital pins for the three buttons
const uint8_t BUTTON_PINS[3] = {0, 1, 2};
enum ButtonIndex {
    BUTTON_1 = 2, // Pin 2
    BUTTON_2 = 0, // Pin 0
    BUTTON_3 = 1  // Pin 1
};


//========================================================================================
//                               HID REPORT DESCRIPTOR
//========================================================================================
// This descriptor tells the host computer that this device is a 3DConnexion SpaceMouse.
// It defines three types of reports:
// Report ID 1: Translation data (X, Y, Z, RX, RY, RZ axes)
// Report ID 3: Button data

static const uint8_t _hidReportDescriptor[] PROGMEM = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x08,        // Usage (Multi-Axis)
    0xa1, 0x01,        // Collection (Application)
    // --- Translation Data Report ---
    0xa1, 0x00,        //   Collection (Physical)
    0x85, 0x01,        //     Report ID (1)
    0x16, 0x00, 0xFE,  //     Logical Minimum (-512) // instead of 500 =  0x10 oxFE
    0x26, 0xFF, 0x01,  //     Logical Maximum (511) // instead of 500 = 0xF4, 0x01
    0x75, 0x10,        //     Report Size (16 bits)
    0x95, 0x06,        //     Report Count (6)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x09, 0x32,        //     Usage (Z)
    0x09, 0x33,        //     Usage (RX)
    0x09, 0x34,        //     Usage (RY)
    0x09, 0x35,        //     Usage (RZ)
    0x81, 0x02,        //     Input (Data, Variable, Absolute)
    0xC0,              //   End Collection
   
    // --- Button Data Report ---
    0xa1, 0x00,        //   Collection (Physical)
    0x85, 0x03,        //     Report ID (3)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (Button 1)
    0x29, 0x20,        //     Usage Maximum (Button 32)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x75, 0x01,        //     Report Size (1)
    0x95, 0x20,        //     Report Count (32)
    0x81, 0x02,        //     Input (Data, Variable, Absolute)
    0xC0,              //   End Collection
    0xC0               // End Collection
};


//========================================================================================
//                                  GLOBAL VARIABLES
//========================================================================================

// Stores the sensor readings when the device is at rest.
int centerPoints[8];

// Stores the state of the 4 logical buttons (3 physical + 1 pseudo).
uint8_t buttonStates[4] = {0, 0, 0, 0};

// Variables for the button state machine.
enum ButtonStateMachine {
    STATE_IDLE,
    STATE_WAITING_FOR_SECOND_BUTTON,
    STATE_PSEUDO_BUTTON_CONFIRMED,
    STATE_SINGLE_BUTTON_CONFIRMED,
    STATE_WAITING_FOR_RELEASE
};
ButtonStateMachine keyState = STATE_IDLE;
unsigned long keyPressTimestamp = 0;
uint8_t confirmedKeyPressed = 0;
const unsigned long PSEUDO_BUTTON_WINDOW_MS = 25; // Time window to detect a second button press.


//========================================================================================
//                             FUNCTION DECLARATIONS
//========================================================================================

void readSensors(int* rawReadings);
void processSensors(int* centeredReadings, const int* rawReadings);
void calculateMovements(int16_t& tx, int16_t& ty, int16_t& tz, int16_t& rx, int16_t& ry, int16_t& rz, const int* centeredReadings);
void readButtons(uint8_t* currentButtonStates);
void sendHidReport(int16_t rx, int16_t ry, int16_t rz, int16_t x, int16_t y, int16_t z, const uint8_t* buttons);
#if DEBUG_LEVEL > 0
void printDebugInfo(const int* raw, const int* centered, int16_t tx, int16_t ty, int16_t tz, int16_t rx, int16_t ry, int16_t rz, const uint8_t* buttons);
#endif

//========================================================================================
//                                      SETUP
//========================================================================================

void setup() {
    // Start HID communication
    static HIDSubDescriptor node(_hidReportDescriptor, sizeof(_hidReportDescriptor));
    HID().AppendDescriptor(&node);

    // Start Serial for debugging if enabled
    #if DEBUG_LEVEL > 0
        Serial.begin(MONITOR_SPEED);
        while (!Serial && millis() < 2000); // Wait for serial connection
        
    #endif
        
    // Set up button pins with internal pull-up resistors
    for (int i = 0; i < 3; i++) {
        pinMode(BUTTON_PINS[i], INPUT_PULLUP);
    }

    // Set ADC reference voltage. INTERNAL (2.56V on Pro Micro) gives better resolution
    // for the sensors than the default 5V, unless debugging raw values.
    analogReference( (DEBUG_LEVEL == DEBUG_RAW_SENSORS) ? DEFAULT : INTERNAL );
    
    // Calibrate the center point of the sensors.
    // Reading twice helps stabilize initial values.
    readSensors(centerPoints);
    delay(10);
    readSensors(centerPoints);
}


//========================================================================================
//                                    MAIN LOOP
//========================================================================================

void loop() {
    // --- DATA AQUISITION ---
    int rawSensorReadings[8];
    readSensors(rawSensorReadings);

    // --- SENSOR PROCESSING ---
    int centeredSensorReadings[8];
    processSensors(centeredSensorReadings, rawSensorReadings);

    // --- MOVEMENT CALCULATION ---
    int16_t tx, ty, tz, rx, ry, rz;
    calculateMovements(tx, ty, tz, rx, ry, rz, centeredSensorReadings);
    
    // --- BUTTON HANDLING ---
    readButtons(buttonStates);
    
    // --- SEND DATA TO PC ---
    if (use3DConnexionMovement) {
        sendHidReport(rx, ry, rz, tx, ty, tz, buttonStates);
    } else {
        // Swap Y and Z translation for "Teaching Tech" style movement
        sendHidReport(rx, ry, rz, tx, tz, ty, buttonStates);
    }

    // --- DEBUG OUTPUT ---
    #if DEBUG_LEVEL > 0
        printDebugInfo(rawSensorReadings, centeredSensorReadings, tx, ty, tz, rx, ry, rz, buttonStates);
    #endif
}


//========================================================================================
//                                CORE LOGIC FUNCTIONS
//========================================================================================

/**
 * @brief Reads all 8 Hall Effect Sensor values.
 * @param rawReadings Pointer to an array to store the raw ADC values.
 */
void readSensors(int* rawReadings) {
    for (int i = 0; i < 8; i++) {
        rawReadings[i] = analogRead(HES_PINS[i]);
    }
}

/**
 * @brief Centers the sensor readings and applies the deadzone filter.
 * @param centeredReadings Pointer to an array to store the processed values.
 * @param rawReadings Pointer to the array of raw sensor values.
 */
void processSensors(int* centeredReadings, const int* rawReadings) {
    for (int i = 0; i < 8; i++) {
        // Subtract the calibrated center point to get directional movement
        int centeredValue = centerPoints[i] - rawReadings[i];

        // Apply deadzone to prevent jitter and unintended input
        if (abs(centeredValue) < DEADZONE) {
            centeredReadings[i] = 0;
        } else {
            // Subtract deadzone to create a smooth transition from zero
            centeredReadings[i] = centeredValue - (DEADZONE * (centeredValue > 0 ? 1 : -1));
        }
    }
}

/**
 * @brief Calculates the 6-DOF movement values from the processed sensor data.
 */
void calculateMovements(int16_t& tx, int16_t& ty, int16_t& tz, int16_t& rx, int16_t& ry, int16_t& rz, const int* centered) {
    // Calculate raw axis values based on sensor physics
    tx = (centered[HES_6_RIGHT] - centered[HES_6_LEFT] + centered[HES_12_RIGHT] - centered[HES_12_LEFT]) / 2;
    ty = (centered[HES_3_NEAR]  - centered[HES_3_FAR]  + centered[HES_9_NEAR]  - centered[HES_9_FAR])  / 2;
    tz = (centered[HES_6_LEFT]  + centered[HES_6_RIGHT] + centered[HES_3_NEAR] + centered[HES_3_FAR] +
          centered[HES_12_RIGHT]+ centered[HES_12_LEFT] + centered[HES_9_FAR]  + centered[HES_9_NEAR]) / 4;
    rx = (centered[HES_6_LEFT]  + centered[HES_6_RIGHT] - centered[HES_12_RIGHT]- centered[HES_12_LEFT])/ 2;
    ry = (centered[HES_9_FAR]   + centered[HES_9_NEAR]  - centered[HES_3_NEAR]  - centered[HES_3_FAR])  / 2;
    rz = (centered[HES_6_LEFT]  + centered[HES_3_NEAR]  + centered[HES_12_RIGHT]+ centered[HES_9_FAR] -
          centered[HES_6_RIGHT] - centered[HES_3_FAR]   - centered[HES_12_LEFT] - centered[HES_9_NEAR]) / 4;

    // Apply speed modifier. Multiply first to preserve integer precision.
    tx = (tx * speedPercentage) / 100;
    ty = (ty * speedPercentage) / 100;
    tz = (tz * speedPercentage) / 100;
    rx = (rx * speedPercentage) / 100;
    ry = (ry * speedPercentage) / 100;
    rz = (rz * speedPercentage) / 100;

    // Apply inversion settings
    if (invertX)  tx *= -1;
    if (invertY)  ty *= -1;
    if (invertZ)  tz *= -1;
    if (invertRX) rx *= -1;
    if (invertRY) ry *= -1;
    if (invertRZ) rz *= -1;
}

/**
 * @brief Reads physical buttons and implements a state machine to handle a "pseudo" button
 * press when two physical buttons are pressed simultaneously.
 * @param currentButtonStates Pointer to the array that holds the final logical button states.
 */
void readButtons(uint8_t* currentButtonStates) {
    // Read the physical state of the buttons (LOW is pressed due to INPUT_PULLUP)

    bool physical_B1_pressed = !digitalRead(BUTTON_PINS[BUTTON_1]);
    bool physical_B2_pressed = !digitalRead(BUTTON_PINS[BUTTON_2]);
    bool physical_B3_pressed = !digitalRead(BUTTON_PINS[BUTTON_3]);

    unsigned long currentTime = millis();

    // Reset logical button states before processing
    currentButtonStates[0] = 0; // Pseudo button
    currentButtonStates[1] = 0; // Physical Button 1 (maps to logical button 2)
    currentButtonStates[2] = 0; // Physical Button 2
    currentButtonStates[3] = 0; // Physical Button 3

    switch (keyState) {
        case STATE_IDLE:
            if (physical_B1_pressed || physical_B3_pressed) {
                keyState = STATE_WAITING_FOR_SECOND_BUTTON;
                keyPressTimestamp = currentTime;
            } else if (physical_B2_pressed) {
                // Handle button 2 directly as it's not part of the pseudo-button combo
                 currentButtonStates[2] = 1;
            }
            break;

        case STATE_WAITING_FOR_SECOND_BUTTON:
            if (physical_B1_pressed && physical_B3_pressed) {
                keyState = STATE_PSEUDO_BUTTON_CONFIRMED;
            } else if (currentTime - keyPressTimestamp > PSEUDO_BUTTON_WINDOW_MS) {
                keyState = STATE_SINGLE_BUTTON_CONFIRMED;
            }
            break;

        case STATE_PSEUDO_BUTTON_CONFIRMED:
            confirmedKeyPressed = 0; // 0 represents the pseudo button
            keyState = STATE_WAITING_FOR_RELEASE;
            break;

        case STATE_SINGLE_BUTTON_CONFIRMED:
            // Check which button was originally pressed
            if (physical_B1_pressed) confirmedKeyPressed = 1;
            else if (physical_B3_pressed) confirmedKeyPressed = 3;
            keyState = STATE_WAITING_FOR_RELEASE;
            break;

        case STATE_WAITING_FOR_RELEASE:
            // Keep the confirmed button active
            currentButtonStates[confirmedKeyPressed] = 1;

            // If the combo buttons are released, return to idle
            if (!physical_B1_pressed && !physical_B3_pressed) {
                keyState = STATE_IDLE;
                confirmedKeyPressed = 0; // Reset
            }
            break;
    }
    
    // Non-combo button state is passed through directly unless a combo is active
    if (keyState == STATE_IDLE) {
      currentButtonStates[2] = physical_B2_pressed;
    }
}


/**
 * @brief Sends the 6-DOF data and button states to the computer via HID reports.
 */
void sendHidReport(int16_t rx, int16_t ry, int16_t rz, int16_t x, int16_t y, int16_t z, const uint8_t* buttons) {
    // Send Translation Report (ID 0x1)
    uint8_t trans[] = { lowByte(x), highByte(x), 
                          lowByte(y), highByte(y), 
                          lowByte(z), highByte(z),
                          lowByte(rx), highByte(rx), 
                          lowByte(ry), highByte(ry), 
                          lowByte(rz), highByte(rz) };
    HID().SendReport(0x1, trans, sizeof(trans));

    // Send Button Report (ID 0x3) 
    static uint8_t lastButtonBits = 0;

    uint8_t currentButtonBits = (buttons[0] << 0) | (buttons[1] << 2) | (buttons[2] << 4) | (buttons[3] << 5);

    // Only send a button report if the button state has changed.
    // This prevents flooding the host and ensures proper press/release events.
    if (currentButtonBits != lastButtonBits) {
        uint8_t btnReport[] = {0x0, 0x0,  0x0, currentButtonBits};
        HID().SendReport(0x3, btnReport, sizeof(btnReport));
        lastButtonBits = currentButtonBits;
    }
}


/**
 * @brief Consolidates all serial printing for debugging into one function.
 */
#if DEBUG_LEVEL > 0
void printDebugInfo(const int* raw, const int* centered, int16_t tx, int16_t ty, int16_t tz, int16_t rx, int16_t ry, int16_t rz, const uint8_t* buttons) {
    
    switch(DEBUG_LEVEL) {
        case DEBUG_RAW_SENSORS:
            for(int i=0; i<8; i++) { Serial.print("HES"); Serial.print(i); Serial.print(":"); Serial.print(raw[i]); Serial.print(i==7 ? "" : ","); }
            Serial.println();
            break;
        case DEBUG_CENTERED_SENSORS:
            for(int i=0; i<8; i++) { Serial.print("HES"); Serial.print(i); Serial.print(":"); Serial.print(centered[i]); Serial.print(i==7 ? "" : ","); }
            Serial.println();
            break;
        case DEBUG_DEADZONE_FILTERED:
            for(int i=0; i<8; i++) { Serial.print("HES"); Serial.print(i); Serial.print(":"); Serial.print(centered[i]); Serial.print(","); }
            for(int i=0; i<4; i++) { Serial.print("But"); Serial.print(i); Serial.print(":"); Serial.print(buttons[i]); Serial.print(i==3 ? "" : ","); }
            Serial.println();
            break;
        case DEBUG_AXIS_VALUES:
            Serial.print("TX:"); Serial.print(tx); Serial.print(","); Serial.print("TY:"); Serial.print(ty); Serial.print(","); Serial.print("TZ:"); Serial.print(tz); Serial.print(",");
            Serial.print("RX:"); Serial.print(rx); Serial.print(","); Serial.print("RY:"); Serial.print(ry); Serial.print(","); Serial.print("RZ:"); Serial.println(rz);
            break;
        case DEBUG_SIDE_BY_SIDE:
            for(int i=0; i<8; i++) { Serial.print("HES"); Serial.print(i); Serial.print(":"); Serial.print(centered[i]); Serial.print(","); }
            Serial.print("||");
            Serial.print("TX:"); Serial.print(tx); Serial.print(","); Serial.print("TY:"); Serial.print(ty); Serial.print(","); Serial.print("TZ:"); Serial.print(tz); Serial.print(",");
            Serial.print("RX:"); Serial.print(rx); Serial.print(","); Serial.print("RY:"); Serial.print(ry); Serial.print(","); Serial.print("RZ:"); Serial.println(rz);
            break;
        case DEBUG_BUTTON_STATE:
             Serial.print("State: "); Serial.print(keyState);
             Serial.print(" | ConfirmedKey: "); Serial.print(confirmedKeyPressed);
             Serial.print(" | B0:"); Serial.print(buttons[0]);
             Serial.print(" | B1:"); Serial.print(buttons[1]);
             Serial.print(" | B2:"); Serial.print(buttons[2]);
             Serial.print(" | B3:"); Serial.println(buttons[3]);
            break;
        case DEBUG_NONE:
            break; 
    }  
}
#endif 