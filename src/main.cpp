/**
 * ============================================================================================
 * @file main.cpp
 * @brief ESP32-S3 Dual-Channel Capacitive Soil Moisture Monitoring System
 * @details CUSTOMIZED SPECIFICALLY FOR: Dracaena (Dragon Tree / Corn Plant)
 * 
 *          Features Exponential Moving Average (EMA) filtering, a 5-tier hysteresis state
 *          machine tailored specifically to Dracaena soil drying requirements, hardware fault
 *          detection (short/open circuit bounds checking), and 1 Hz structured JSON 
 *          telemetry output over Serial.
 * 
 * @target_plant Dracaena (Requires top 50%-75% of soil to dry out before re-watering)
 * @target ESP32-S3 (Arduino Framework / PlatformIO)
 * 
 * @hardware_topology
 *   - GPIO 1: Data pin connected to WS2812 RGB LED Strip (3 LED pixels total)
 *             - Pixel 0: System/Heartbeat (Breathing Magenta/Purple effect)
 *             - Pixel 1: Sensor 1 Status (Dracaena Moisture Tier or Flashing Red Fault)
 *             - Pixel 2: Sensor 2 Status (Dracaena Moisture Tier or Flashing Red Fault)
 *   - GPIO 2: Analog input for Capacitive Soil Moisture Sensor 1 (Top Soil)
 *   - GPIO 3: Analog input for Capacitive Soil Moisture Sensor 2 (Deep Soil)
 * ============================================================================================
 */

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// ============================================================================================
// PLANT IDENTIFICATION & CONFIGURATION
// ============================================================================================

const char* TARGET_PLANT_NAME = "Dracaena"; 

// ============================================================================================
// HARDWARE PIN & CONFIGURATION CONSTANTS
// ============================================================================================

#define LED_PIN      1    ///< GPIO pin driving WS2812 NeoPixel data input line
#define NUM_LEDS     3    ///< Total count of cascaded WS2812 RGB LEDs

#define SENSOR1_PIN  2    ///< ADC1 Channel assigned to Moisture Sensor 1 (Top Soil)
#define SENSOR2_PIN  3    ///< ADC1 Channel assigned to Moisture Sensor 2 (Deep Soil)

/**
 * @brief Independent Sensor Calibration Constants (12-bit ADC Range: 0 - 4095)
 */
const int S1_AIR_VALUE   = 2655; ///< Sensor 1 Dry Air Baseline (0%)
const int S1_WATER_VALUE = 934;  ///< Sensor 1 Submerged Water Baseline (100%)

const int S2_AIR_VALUE   = 2575; ///< Sensor 2 Dry Air Baseline (0%)
const int S2_WATER_VALUE = 643;  ///< Sensor 2 Submerged Water Baseline (100%)

// ============================================================================================
// FAULT DETECTION BOUNDARIES
// ============================================================================================

const int ADC_MIN_FAULT = 200;  ///< Short-circuit / grounded sensor boundary
const int ADC_MAX_FAULT = 3800; ///< Open-circuit / disconnected sensor boundary

const int STATE_FAULT = -1;     ///< State representing hardware fault condition

// ============================================================================================
// SIGNAL PROCESSING & ALGORITHM CONFIGURATION
// ============================================================================================

const float ALPHA = 0.15;       ///< Exponential Moving Average (EMA) Weight Coefficient

float filteredADC1 = 0.0;
float filteredADC2 = 0.0;

/**
 * @brief Discrete Moisture Tier States — Tailored Specifically for DRACAENA
 *       -1 = FAULT               (Hardware Error / Disconnected Pin)
 *        0 = RED                 (< 15%: Water Immediately - Root stress zone)
 *        1 = AMBER / YELLOW      (15% - 25%: Water Soon - Optimal dry target for Dracaena)
 *        2 = GREEN               (25% - 65%: Optimal Moisture Zone)
 *        3 = CYAN / BLUE         (65% - 75%: Recently Watered)
 *        4 = MAGENTA / DEEP BLUE (> 75%: Over-Saturated Warning - Risk of cane/root rot)
 */
int s1State = 0;
int s2State = 0;

/// @brief Hysteresis Threshold Band (+/- percentage points)
const int HYSTERESIS = 2;

// ============================================================================================
// NON-BLOCKING SCHEDULER & TIMING CONSTANTS
// ============================================================================================

unsigned long lastSampleTime = 0;   ///< Timestamp (ms) of the last ADC sample tick
unsigned long lastReportTime = 0;   ///< Timestamp (ms) of the last Telemetry update tick
unsigned long lastBlinkTime  = 0;   ///< Timestamp (ms) for hardware fault LED toggle

const unsigned long SAMPLE_INTERVAL = 100;  ///< Sensor sampling frequency: 10 Hz (100 ms)
const unsigned long REPORT_INTERVAL = 1000; ///< Reporting frequency: 1 Hz (1000 ms)
const unsigned long BLINK_INTERVAL  = 200;  ///< Fault LED strobe period: 5 Hz (200 ms)

const float FADE_PERIOD_MS = 5000.0;       ///< System heartbeat sine wave period (5 seconds)

bool faultBlinkState = false;              ///< Toggle flag for fault strobe pattern

// ============================================================================================
// GRAPHICS & COLOR DEFINITIONS (Adapted for Dracaena Moisture Profiles)
// ============================================================================================

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Color Mappings configured specifically for Dracaena Table Requirements
const uint32_t COLOR_RED         = strip.Color(255, 0, 0);     ///< Tier 0: <15% (Water Immediately - Stress)
const uint32_t COLOR_AMBER       = strip.Color(255, 90, 0);    ///< Tier 1: 15-25% (Water Soon - Dracaena Dry Target)
const uint32_t COLOR_GREEN       = strip.Color(0, 255, 0);     ///< Tier 2: 25-65% (Optimal Moisture Zone)
const uint32_t COLOR_CYAN        = strip.Color(0, 255, 255);   ///< Tier 3: 65-75% (Recently Watered)
const uint32_t COLOR_MAGENTA     = strip.Color(255, 0, 255);   ///< Tier 4: >75% (Over-Saturated Warning)
const uint32_t COLOR_OFF         = strip.Color(0, 0, 0);       ///< Off state for fault strobe

// ============================================================================================
// HELPER FUNCTIONS & LOGIC UNITS
// ============================================================================================

/**
 * @brief Validates if raw ADC counts fall within hardware operational tolerance.
 */
bool isSensorValid(int rawADC) {
    return (rawADC >= ADC_MIN_FAULT && rawADC <= ADC_MAX_FAULT);
}

/**
 * @brief Evaluates moisture state transition with hysteresis for DRACAENA plants.
 * @details DRACAENA THRESHOLD MATRIX:
 *          - Tier 0 (RED):     < 15%   -> Water Immediately (Soil completely dry down to roots)
 *          - Tier 1 (AMBER):   15-25%  -> Water Soon (Optimal dry target; top 50%-75% dried out)
 *          - Tier 2 (GREEN):   25-65%  -> Optimal Moisture Zone (Ideal root absorption)
 *          - Tier 3 (CYAN):    65-75%  -> Recently Watered (High moisture post-watering)
 *          - Tier 4 (MAGENTA): > 75%   -> Over-Saturated Warning (Risk of cane rot and root damage)
 * 
 * @param currentState Current discrete moisture tier (-1 to 4).
 * @param percentage   Calculated moisture percentage (0% to 100%).
 * @param isValid      Hardware validation flag.
 * @return int         Updated moisture tier state (-1 to 4).
 */
int updateMoistureState(int currentState, int percentage, bool isValid) {
    if (!isValid) {
        return STATE_FAULT;
    }

    // Recovery path from fault state back to valid monitoring
    if (currentState == STATE_FAULT) {
        if (percentage > 75) return 4;
        if (percentage > 65) return 3;
        if (percentage > 25) return 2;
        if (percentage > 15) return 1;
        return 0;
    }

    switch (currentState) {
        case 0: // DRACAENA: Currently RED (< 15%)
            if (percentage >= (15 + HYSTERESIS)) return 1;
            break;

        case 1: // DRACAENA: Currently AMBER/YELLOW (15% - 25% - Optimal dry target)
            if (percentage < (15 - HYSTERESIS))  return 0;
            if (percentage >= (25 + HYSTERESIS)) return 2;
            break;

        case 2: // DRACAENA: Currently GREEN (25% - 65% - Optimal moisture)
            if (percentage < (25 - HYSTERESIS))  return 1;
            if (percentage >= (65 + HYSTERESIS)) return 3;
            break;

        case 3: // DRACAENA: Currently CYAN/BLUE (65% - 75% - Recently watered)
            if (percentage < (65 - HYSTERESIS))  return 2;
            if (percentage >= (75 + HYSTERESIS)) return 4;
            break;

        case 4: // DRACAENA: Currently MAGENTA/DEEP BLUE (> 75% - Over-saturated warning)
            if (percentage < (75 - HYSTERESIS))  return 3;
            break;
    }
    return currentState; // Maintain tier within hysteresis band
}

/**
 * @brief Maps discrete state code to 24-bit NeoPixel color matching DRACAENA moisture table.
 * 
 * @param state Discrete moisture state (-1 to 4).
 * @return uint32_t Packed GRB/RGB color code.
 */
uint32_t getColorFromState(int state) {
    switch (state) {
        case 4:           return COLOR_MAGENTA;  // DRACAENA: Over-saturated (> 75%)
        case 3:           return COLOR_CYAN;     // DRACAENA: Recently Watered (65-75%)
        case 2:           return COLOR_GREEN;    // DRACAENA: Optimal Zone (25-65%)
        case 1:           return COLOR_AMBER;    // DRACAENA: Water Soon Target (15-25%)
        case 0:           return COLOR_RED;      // DRACAENA: Water Immediately (< 15%)
        case STATE_FAULT: return faultBlinkState ? COLOR_RED : COLOR_OFF; // Fault Strobe
        default:          return COLOR_OFF;
    }
}

/**
 * @brief Maps raw/filtered analog ADC counts into a bounded 0-100% moisture scale.
 */
int calculatePercentage(float adcVal, int airVal, int waterVal) {
    int percent = map((int)adcVal, airVal, waterVal, 0, 100);
    return constrain(percent, 0, 100);
}

// ============================================================================================
// SYSTEM SETUP & INITIALIZATION
// ============================================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // Announce system initialization with target plant parameter
    Serial.printf("{\"system\":\"ESP32-S3 Dual Moisture System\",\"target_plant\":\"%s\",\"status\":\"initializing\"}\n", TARGET_PLANT_NAME);

    analogReadResolution(12);

    int raw1 = analogRead(SENSOR1_PIN);
    int raw2 = analogRead(SENSOR2_PIN);

    filteredADC1 = raw1;
    filteredADC2 = raw2;

    int p1 = calculatePercentage(filteredADC1, S1_AIR_VALUE, S1_WATER_VALUE);
    int p2 = calculatePercentage(filteredADC2, S2_AIR_VALUE, S2_WATER_VALUE);

    s1State = updateMoistureState(s1State, p1, isSensorValid(raw1));
    s2State = updateMoistureState(s2State, p2, isSensorValid(raw2));

    strip.begin();
    strip.setBrightness(8); // Limit max brightness to ~3%
    strip.clear();
    strip.show();
}

// ============================================================================================
// MAIN EXECUTION LOOP
// ============================================================================================

void loop() {
    unsigned long currentMillis = millis();

    // ----------------------------------------------------------------------------------------
    // TASK 1: Sensor Sampling, Fault Verification & EMA Filtering (10 Hz)
    // ----------------------------------------------------------------------------------------
    if (currentMillis - lastSampleTime >= SAMPLE_INTERVAL) {
        lastSampleTime = currentMillis;

        int raw1 = analogRead(SENSOR1_PIN);
        int raw2 = analogRead(SENSOR2_PIN);

        filteredADC1 = (ALPHA * raw1) + ((1.0 - ALPHA) * filteredADC1);
        filteredADC2 = (ALPHA * raw2) + ((1.0 - ALPHA) * filteredADC2);

        bool v1 = isSensorValid(raw1);
        bool v2 = isSensorValid(raw2);

        int p1 = calculatePercentage(filteredADC1, S1_AIR_VALUE, S1_WATER_VALUE);
        int p2 = calculatePercentage(filteredADC2, S2_AIR_VALUE, S2_WATER_VALUE);

        s1State = updateMoistureState(s1State, p1, v1);
        s2State = updateMoistureState(s2State, p2, v2);
    }

    // ----------------------------------------------------------------------------------------
    // TASK 2: Hardware Fault Strobe Timer Toggle (5 Hz / 200 ms)
    // ----------------------------------------------------------------------------------------
    if (currentMillis - lastBlinkTime >= BLINK_INTERVAL) {
        lastBlinkTime = currentMillis;
        faultBlinkState = !faultBlinkState;
    }

    // ----------------------------------------------------------------------------------------
    // TASK 3: Heartbeat Animation for Pixel 0 & Status Outputs for Pixels 1 & 2
    // ----------------------------------------------------------------------------------------
    float radians = ((currentMillis % (unsigned long)FADE_PERIOD_MS) / FADE_PERIOD_MS) * (2.0 * PI);
    float brightnessFactor = (sin(radians - (PI / 2.0)) + 1.0) / 2.0;

    uint8_t r = (uint8_t)(255 * brightnessFactor);
    uint8_t b = (uint8_t)(255 * brightnessFactor);
    
    strip.setPixelColor(0, strip.Color(r, 0, b));

    strip.setPixelColor(1, getColorFromState(s1State));
    strip.setPixelColor(2, getColorFromState(s2State));

    // ----------------------------------------------------------------------------------------
    // TASK 4: Structured JSON Telemetry Output over Serial (1 Hz)
    // ----------------------------------------------------------------------------------------
    if (currentMillis - lastReportTime >= REPORT_INTERVAL) {
        lastReportTime = currentMillis;

        int percent1 = calculatePercentage(filteredADC1, S1_AIR_VALUE, S1_WATER_VALUE);
        int percent2 = calculatePercentage(filteredADC2, S2_AIR_VALUE, S2_WATER_VALUE);

        Serial.printf(
            "{\"uptime_ms\":%lu,\"target_plant\":\"%s\",\"s1\":{\"gpio\":2,\"raw\":%.2f,\"pct\":%d,\"tier\":%d,\"fault\":%s},\"s2\":{\"gpio\":3,\"raw\":%.2f,\"pct\":%d,\"tier\":%d,\"fault\":%s}}\n",
            currentMillis,
            TARGET_PLANT_NAME,
            filteredADC1,
            percent1,
            s1State,
            (s1State == STATE_FAULT) ? "true" : "false",
            filteredADC2,
            percent2,
            s2State,
            (s2State == STATE_FAULT) ? "true" : "false"
        );
    }

    strip.show();
    delay(10);
}