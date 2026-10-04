/**
 * ============================================================================================
 * @file main.cpp
 * @brief ESP32-S3 Dual-Channel Capacitive Soil Moisture System (Low-Power Deep-Sleep Edition)
 * @details CUSTOMIZED SPECIFICALLY FOR: Dracaena (Dragon Tree / Corn Plant)
 * 
 *          System Architecture & Operational Summary:
 *          1. Boot Initialization:
 *             - Configures GPIO 0 as an active-LOW input with an internal pull-up resistor.
 *             - Reads parameter overrides (Device ID, MQTT host/port, sleep duration) from NVS Flash.
 *             - Checks for physical button press or missing Wi-Fi credentials to conditionally 
 *               spin up an Access Point (AP) configuration portal via WiFiManager.
 * 
 *          2. Operational Measurement Cycle (Wake Phase):
 *             - Activates 12-bit ADC resolution on channels GPIO 2 (Sensor 1) and GPIO 3 (Sensor 2).
 *             - Samples sensors over a 1-second window (10 periodic reads spaced 100 ms apart).
 *             - Applies an Exponential Moving Average (EMA) filter ($\alpha = 0.15$) to smooth raw readings.
 *             - Evaluates moisture percentage against calibrated dry/submerged baselines.
 *             - Applies a 5-tier state machine with a $\pm 2\%$ hysteresis band tailored for Dracaena.
 * 
 *          3. Data Transmission & Visual Feedback:
 *             - Connects to Wi-Fi, serializes structured JSON telemetry, and publishes via MQTT (QoS 0/1).
 *             - Gracefully terminates MQTT and Wi-Fi connections to ensure zero radio overhead.
 *             - Triggers a 3-cycle sinusoidal "breathing" animation on WS2812 NeoPixels to present status.
 * 
 *          4. Ultra-Low-Power Shutdown:
 *             - Transmits all-zero RGB data to the WS2812 strip to clear internal registers and stop current draw.
 *             - Configures the ESP32 RTC timer wakeup source for the user-specified interval.
 *             - Enters Deep Sleep mode (~10–20 µA total current draw).
 * 
 * @target_plant Dracaena (Requires top 50%-75% of soil to dry out before re-watering)
 * @target ESP32-S3 (Arduino Framework / PlatformIO)
 * 
 * @hardware_topology
 *   - GPIO 0: Physical Config Button (BOOT button, Active LOW with internal pull-up)
 *   - GPIO 1: High-speed NRZ Data line connected to WS2812 RGB LED Strip (3 LED pixels total)
 *   - GPIO 2: Analog input for Capacitive Soil Moisture Sensor 1 (Top Soil Layer)
 *   - GPIO 3: Analog input for Capacitive Soil Moisture Sensor 2 (Deep Root Layer)
 * ============================================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>

// ============================================================================================
// HARDWARE PIN DEFINITIONS & SYSTEM CONSTANTS
// ============================================================================================

/** @brief GPIO pin assigned to the active-LOW user configuration button (ESP32 BOOT button). */
#define CONFIG_BUTTON_PIN 0

/** @brief GPIO output pin driving the WS2812 NeoPixel data input signal line. */
#define LED_PIN           1

/** @brief Total quantity of cascaded WS2812 RGB LED pixels on the data bus. */
#define NUM_LEDS          3

/** @brief Analog input pin (ADC1_CH1) connected to Moisture Sensor 1 (Top Soil Layer). */
#define SENSOR1_PIN       2

/** @brief Analog input pin (ADC1_CH2) connected to Moisture Sensor 2 (Deep Root Layer). */
#define SENSOR2_PIN       3

/** @brief String constant identifying target plant species for MQTT JSON payload metadata. */
const char* TARGET_PLANT_NAME = "Dracaena";

/**
 * @struct SensorCalibration
 * @brief Container for independent hardware sensor analog-to-digital converter (ADC) boundaries.
 */
const int S1_AIR_VALUE   = 2655; ///< Sensor 1 Raw ADC value in dry air (0% moisture baseline).
const int S1_WATER_VALUE = 934;  ///< Sensor 1 Raw ADC value fully submerged in water (100% moisture baseline).

const int S2_AIR_VALUE   = 2575; ///< Sensor 2 Raw ADC value in dry air (0% moisture baseline).
const int S2_WATER_VALUE = 643;  ///< Sensor 2 Raw ADC value fully submerged in water (100% moisture baseline).

// ============================================================================================
// FAULT DETECTION & STATE MACHINE CONSTANTS
// ============================================================================================

/** @brief Minimum raw ADC boundary below which a sensor line is flagged as shorted/grounded. */
const int ADC_MIN_FAULT = 200;

/** @brief Maximum raw ADC boundary above which a sensor line is flagged as open-circuit/disconnected. */
const int ADC_MAX_FAULT = 3800;

/** @brief Numeric error indicator code assigned to a channel when hardware fault is detected. */
const int STATE_FAULT   = -1;

/** @brief Exponential Moving Average (EMA) smoothing weight coefficient ($0 < \alpha \le 1.0$). */
const float ALPHA       = 0.15;

/** @brief Deadband percentage range applied to state boundaries to suppress rapidly oscillating state toggles. */
const int HYSTERESIS    = 2;

// ============================================================================================
// COLOR DEFINITIONS (RGB Triplet Arrays for Sinusoidal Brightness Scaling)
// ============================================================================================

/** @brief Initialize the NeoPixel control class targeting 800 kHz bitstream transmission. */
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

/** @brief Base RGB triplet for Tier 0 (<15% moisture): Critical Under-watering Warning (Red). */
const uint8_t COLOR_RED_RGB[3]     = {255, 0, 0};

/** @brief Base RGB triplet for Tier 1 (15–25% moisture): Approaching Water Target (Amber). */
const uint8_t COLOR_AMBER_RGB[3]   = {255, 90, 0};

/** @brief Base RGB triplet for Tier 2 (25–65% moisture): Optimal Dracaena Root Zone (Green). */
const uint8_t COLOR_GREEN_RGB[3]   = {0, 255, 0};

/** @brief Base RGB triplet for Tier 3 (65–75% moisture): Soil Recently Watered (Cyan). */
const uint8_t COLOR_CYAN_RGB[3]    = {0, 255, 255};

/** @brief Base RGB triplet for Tier 4 (>75% moisture) / System Status: Over-saturation Warning (Magenta). */
const uint8_t COLOR_MAGENTA_RGB[3] = {255, 0, 255};

// ============================================================================================
// NVS STORAGE, NETWORK CLIENTS & GLOBAL STATE VARIABLES
// ============================================================================================

/** @brief Non-Volatile Storage (NVS) handle used for persisting configuration settings across reboots. */
Preferences preferences;

/** @brief Underlying TCP network client instance for MQTT communications. */
WiFiClient espClient;

/** @brief PubSubClient MQTT client wrapper using the TCP network connection. */
PubSubClient mqttClient(espClient);

// Storage buffer strings for NVS key-value management
char device_id[32]     = "Dracaena_Monitor_01"; ///< Unique hardware identifier string.
char mqtt_server[64]   = "192.168.1.100";       ///< MQTT broker IP address or hostname.
char mqtt_port[6]      = "1883";                ///< MQTT broker TCP port.
char sleep_time_s[8]   = "300";                 ///< Deep sleep duration expressed in seconds (Default: 5 mins).

bool shouldSaveConfig  = false;                 ///< Flag set by WiFiManager when user submits new portal configuration.
bool isConfigured      = false;                 ///< System flag indicating valid parameters have been provisioned in NVS.

// Runtime sensor measurement and evaluation variables
float filteredADC1     = 0.0;                   ///< Smoothed EMA ADC value for Sensor 1.
float filteredADC2     = 0.0;                   ///< Smoothed EMA ADC value for Sensor 2.
int s1State            = 0;                     ///< Evaluated moisture tier (0–4 or -1 for fault) for Sensor 1.
int s2State            = 0;                     ///< Evaluated moisture tier (0–4 or -1 for fault) for Sensor 2.

// ============================================================================================
// FUNCTION IMPLEMENTATIONS
// ============================================================================================

/**
 * @brief Validates whether a raw ADC reading falls within valid operational boundaries.
 * @param rawADC Unfiltered 12-bit analog read value from the ADC hardware (0 to 4095).
 * @return True if the signal is within safe operating limits; false if shorted or disconnected.
 */
bool isSensorValid(int rawADC) {
    return (rawADC >= ADC_MIN_FAULT && rawADC <= ADC_MAX_FAULT);
}

/**
 * @brief Converts a filtered analog ADC reading into a relative moisture percentage.
 * @details Reverses the relationship because capacitive sensor voltage drops as moisture rises.
 * @param adcVal The smoothed exponential moving average ADC value.
 * @param airVal Calibrated ADC baseline when sensor is in dry air (0%).
 * @param waterVal Calibrated ADC baseline when sensor is fully submerged (100%).
 * @return Constrained integer percentage value bounded strictly between 0% and 100%.
 */
int calculatePercentage(float adcVal, int airVal, int waterVal) {
    int percent = map((int)adcVal, airVal, waterVal, 0, 100);
    return constrain(percent, 0, 100);
}

/**
 * @brief Evaluates moisture state transition with hysteresis tailored for Dracaena species.
 * @details Dracaena plants thrive when topsoil dries out significantly between waterings.
 *          Implements a 5-tier state machine (0 to 4) with a deadband zone (`HYSTERESIS`) to prevent state chattering.
 * 
 * @param currentState Current evaluated state code of the sensor channel.
 * @param percentage Calculated moisture percentage (0–100%).
 * @param isValid Boolean flag representing signal integrity.
 * @return Updated state code integer (0 to 4), or STATE_FAULT (-1) on invalid signal.
 */
int updateMoistureState(int currentState, int percentage, bool isValid) {
    // Immediately flag hardware fault if raw signal is out of bounds
    if (!isValid) return STATE_FAULT;

    // Reset state from fault directly to matching percentage tier upon recovery
    if (currentState == STATE_FAULT) {
        if (percentage > 75) return 4;
        if (percentage > 65) return 3;
        if (percentage > 25) return 2;
        if (percentage > 15) return 1;
        return 0;
    }

    // Apply hysteresis thresholding on state boundary transitions
    switch (currentState) {
        case 0: 
            if (percentage >= (15 + HYSTERESIS)) return 1; 
            break;
        case 1: 
            if (percentage < (15 - HYSTERESIS))  return 0; 
            if (percentage >= (25 + HYSTERESIS)) return 2; 
            break;
        case 2: 
            if (percentage < (25 - HYSTERESIS))  return 1; 
            if (percentage >= (65 + HYSTERESIS)) return 3; 
            break;
        case 3: 
            if (percentage < (65 - HYSTERESIS))  return 2; 
            if (percentage >= (75 + HYSTERESIS)) return 4; 
            break;
        case 4: 
            if (percentage < (75 - HYSTERESIS))  return 3; 
            break;
    }
    return currentState;
}

/**
 * @brief Maps a numerical state code to its corresponding base RGB color array.
 * @param state Moisture tier code (0 to 4) or error code (-1).
 * @return Pointer to a 3-element uint8_t array containing {Red, Green, Blue} intensity channels.
 */
const uint8_t* getRGBFromState(int state) {
    switch (state) {
        case 4:           return COLOR_MAGENTA_RGB;
        case 3:           return COLOR_CYAN_RGB;
        case 2:           return COLOR_GREEN_RGB;
        case 1:           return COLOR_AMBER_RGB;
        case 0:           return COLOR_RED_RGB;
        case STATE_FAULT: return COLOR_RED_RGB;
        default:          return COLOR_RED_RGB;
    }
}

/**
 * @brief Callback handler triggered by WiFiManager when new user settings are saved in AP portal.
 */
void saveConfigCallback() {
    shouldSaveConfig = true;
}

/**
 * @brief Reads persisted configuration strings from ESP32 Non-Volatile Storage (NVS Flash).
 */
void loadStoredParameters() {
    preferences.begin("plant_config", true); // Open NVS namespace in read-only mode
    if (preferences.isKey("device_id"))   String(preferences.getString("device_id")).toCharArray(device_id, 32);
    if (preferences.isKey("mqtt_server")) String(preferences.getString("mqtt_server")).toCharArray(mqtt_server, 64);
    if (preferences.isKey("mqtt_port"))   String(preferences.getString("mqtt_port")).toCharArray(mqtt_port, 6);
    if (preferences.isKey("sleep_time"))  String(preferences.getString("sleep_time")).toCharArray(sleep_time_s, 8);
    isConfigured = preferences.getBool("configured", false);
    preferences.end();
}

/**
 * @brief Writes updated configuration parameters into ESP32 Non-Volatile Storage (NVS Flash).
 */
void saveStoredParameters() {
    preferences.begin("plant_config", false); // Open NVS namespace in read-write mode
    preferences.putString("device_id", device_id);
    preferences.putString("mqtt_server", mqtt_server);
    preferences.putString("mqtt_port", mqtt_port);
    preferences.putString("sleep_time", sleep_time_s);
    preferences.putBool("configured", true);
    preferences.end();
}

/**
 * @brief Spawns an Access Point (AP) web configuration portal using WiFiManager.
 * @details Keeps the MCU awake indefinitely so users can connect to `ESP32-Moisture-AP`
 *          at IP `192.168.4.1` to provision network credentials and system settings.
 */
void startConfigurationPortal() {
    Serial.println("[AP Mode] Starting Access Point Configuration Portal...");

    // Set distinctive static LED color profile indicating configuration AP mode is active
    strip.setPixelColor(0, strip.Color(255, 0, 255)); // Pixel 0: Magenta
    strip.setPixelColor(1, strip.Color(255, 90, 0));  // Pixel 1: Amber
    strip.setPixelColor(2, strip.Color(255, 90, 0));  // Pixel 2: Amber
    strip.show();

    WiFiManager wm;
    wm.setSaveConfigCallback(saveConfigCallback);

    // Bind custom HTML input fields to the WiFiManager web page
    WiFiManagerParameter custom_device_id("device_id", "Device ID", device_id, 32);
    WiFiManagerParameter custom_mqtt_server("server", "MQTT Server IP", mqtt_server, 64);
    WiFiManagerParameter custom_mqtt_port("port", "MQTT Port", mqtt_port, 6);
    WiFiManagerParameter custom_sleep_time("sleep", "Deep Sleep (Seconds)", sleep_time_s, 8);

    wm.addParameter(&custom_device_id);
    wm.addParameter(&custom_mqtt_server);
    wm.addParameter(&custom_mqtt_port);
    wm.addParameter(&custom_sleep_time);

    wm.setConfigPortalTimeout(180); // 3-minute AP portal timeout before retrying loop

    if (!wm.startConfigPortal("ESP32-Moisture-AP")) {
        Serial.println("[AP Mode] Configuration portal timed out or failed to connect.");
    } else {
        Serial.println("[AP Mode] Connected to Wi-Fi successfully via Portal!");
    }

    // Persist new parameters to NVS flash memory if portal form was submitted
    if (shouldSaveConfig) {
        strcpy(device_id, custom_device_id.getValue());
        strcpy(mqtt_server, custom_mqtt_server.getValue());
        strcpy(mqtt_port, custom_mqtt_port.getValue());
        strcpy(sleep_time_s, custom_sleep_time.getValue());
        saveStoredParameters();
        Serial.println("[AP Mode] New parameters saved to NVS!");
    }
}

/**
 * @brief Samples both analog moisture channels over a 1-second interval using an EMA filter.
 * @details Executes 10 discrete discrete sampling iterations spaced 100 ms apart.
 *          Applies the formula: $\text{EMA}_t = (\alpha \times \text{Raw}) + ((1 - \alpha) \times \text{EMA}_{t-1})$
 */
void sampleSensorsOneSecond() {
    Serial.println("[Sensors] Sampling for 1 second (10 samples @ 100ms)...");

    int raw1 = analogRead(SENSOR1_PIN);
    int raw2 = analogRead(SENSOR2_PIN);

    // Initialize EMA filter seeds with first raw read
    filteredADC1 = raw1;
    filteredADC2 = raw2;

    // Perform 10 sampling loops over a 1000 ms duration
    for (int i = 0; i < 10; i++) {
        delay(100);
        raw1 = analogRead(SENSOR1_PIN);
        raw2 = analogRead(SENSOR2_PIN);

        // Apply Exponential Moving Average equations
        filteredADC1 = (ALPHA * raw1) + ((1.0 - ALPHA) * filteredADC1);
        filteredADC2 = (ALPHA * raw2) + ((1.0 - ALPHA) * filteredADC2);
    }

    bool v1 = isSensorValid(raw1);
    bool v2 = isSensorValid(raw2);

    int percent1 = calculatePercentage(filteredADC1, S1_AIR_VALUE, S1_WATER_VALUE);
    int percent2 = calculatePercentage(filteredADC2, S2_AIR_VALUE, S2_WATER_VALUE);

    // Update state machine variables
    s1State = updateMoistureState(s1State, percent1, v1);
    s2State = updateMoistureState(s2State, percent2, v2);
}

/**
 * @brief Smoothly pulses NeoPixel LEDs 3 times using a sinusoidal brightness scaling algorithm.
 * @details Calculates color intensity per channel based on $\sin(\theta)$ where $0 \le \theta \le \pi$.
 *          Fades smoothy from 0% brightness $\rightarrow$ 100% $\rightarrow$ 0% over 3 cycles.
 */
void breatheStatusLeds3Times() {
    const uint8_t* rgb0 = COLOR_MAGENTA_RGB;         // Pixel 0: System status indicator
    const uint8_t* rgb1 = getRGBFromState(s1State); // Pixel 1: Sensor 1 status color
    const uint8_t* rgb2 = getRGBFromState(s2State); // Pixel 2: Sensor 2 status color

    const int totalBreaths   = 3;
    const int stepsPerBreath = 60; // Smoothness step resolution per cycle
    const int stepDelayMs    = 25; // ~1.5 seconds per complete breath wave

    for (int b = 0; b < totalBreaths; b++) {
        for (int step = 0; step < stepsPerBreath; step++) {
            // Calculate sine factor ranging smoothly from 0.0 to 1.0 back to 0.0
            float angle = ((float)step / (float)stepsPerBreath) * PI;
            float factor = sin(angle);

            // Dynamically scale RGB channels for all cascaded pixels
            strip.setPixelColor(0, strip.Color((uint8_t)(rgb0[0] * factor), (uint8_t)(rgb0[1] * factor), (uint8_t)(rgb0[2] * factor)));
            strip.setPixelColor(1, strip.Color((uint8_t)(rgb1[0] * factor), (uint8_t)(rgb1[1] * factor), (uint8_t)(rgb1[2] * factor)));
            strip.setPixelColor(2, strip.Color((uint8_t)(rgb2[0] * factor), (uint8_t)(rgb2[1] * factor), (uint8_t)(rgb2[2] * factor)));
            
            strip.show();
            delay(stepDelayMs);
        }
    }

    // Ensure all pixels are fully cleared at the conclusion of the animation sequence
    strip.clear();
    strip.show();
}

/**
 * @brief Establishes network links, transmits telemetry JSON payload over MQTT, and tears down sockets.
 * @return True on successful MQTT payload delivery; false on network timeout or socket error.
 */
bool connectAndPublishMQTT() {
    // Attempt connection to Wi-Fi access point if disconnected
    if (WiFi.status() != WL_CONNECTED) {
        Serial.print("[WiFi] Connecting to AP...");
        WiFi.begin();
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            Serial.print(".");
            attempts++;
        }
        Serial.println();
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] Connection failed!");
        return false;
    }

    Serial.printf("[WiFi] Connected! Assigned IP: %s\n", WiFi.localIP().toString().c_str());

    uint16_t port = atoi(mqtt_port);
    mqttClient.setServer(mqtt_server, port);

    Serial.printf("[MQTT] Connecting to broker %s:%d...\n", mqtt_server, port);

    // Generate unique MQTT client ID string using pseudo-random hex suffix
    String clientId = String(device_id) + "-" + String(random(0xffff), HEX);
    if (!mqttClient.connect(clientId.c_str())) {
        Serial.printf("[MQTT] Connection failed! Error code: %d\n", mqttClient.state());
        return false;
    }

    Serial.println("[MQTT] Connected successfully!");

    int percent1 = calculatePercentage(filteredADC1, S1_AIR_VALUE, S1_WATER_VALUE);
    int percent2 = calculatePercentage(filteredADC2, S2_AIR_VALUE, S2_WATER_VALUE);

    // Construct target MQTT topic path: plant_monitor/<device_id>/telemetry
    char topic[128];
    snprintf(topic, sizeof(topic), "plant_monitor/%s/telemetry", device_id);

    // Format telemetry measurements into structured JSON payload string
    char payload[384];
    snprintf(payload, sizeof(payload),
        "{\"device_id\":\"%s\",\"target_plant\":\"%s\",\"s1\":{\"gpio\":2,\"raw\":%.2f,\"pct\":%d,\"tier\":%d,\"fault\":%s},\"s2\":{\"gpio\":3,\"raw\":%.2f,\"pct\":%d,\"tier\":%d,\"fault\":%s}}",
        device_id,
        TARGET_PLANT_NAME,
        filteredADC1, percent1, s1State, (s1State == STATE_FAULT) ? "true" : "false",
        filteredADC2, percent2, s2State, (s2State == STATE_FAULT) ? "true" : "false"
    );

    Serial.printf("[MQTT] Publishing message to %s:\n%s\n", topic, payload);
    bool success = mqttClient.publish(topic, payload, true);

    // Gracefully shutdown network radio to optimize power consumption prior to deep sleep
    mqttClient.disconnect();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);

    return success;
}

/**
 * @brief Triggers LED breathing visualization, clears display shift registers, and enters deep sleep.
 */
void goToDeepSleep() {
    uint64_t sleepSeconds = atoi(sleep_time_s);
    if (sleepSeconds < 10) sleepSeconds = 10; // Enforce 10-second lower safety bound

    Serial.println("[UI] Executing 3-cycle status LED breathing animation...");
    breatheStatusLeds3Times();

    Serial.printf("[Power] Clearing LEDs and entering deep sleep for %llu seconds...\n", sleepSeconds);

    // Push zero values to WS2812 strip so LEDs draw zero current in sleep
    strip.clear();
    strip.show();

    // Enable internal Real-Time Clock (RTC) timer wakeup source
    esp_sleep_enable_timer_wakeup(sleepSeconds * 1000000ULL);
    esp_deep_sleep_start();
}

// ============================================================================================
// MAIN SYSTEM ENTRY POINTS
// ============================================================================================

/**
 * @brief System initialization function executed on boot/reset.
 */
void setup() {
    Serial.begin(115200);
    delay(500);

    // Configure pin directions and ADC resolution
    pinMode(CONFIG_BUTTON_PIN, INPUT_PULLUP);
    analogReadResolution(12);

    // Initialize NeoPixel strip registers
    strip.begin();
    strip.setBrightness(16); // Set peak brightness ceiling
    strip.clear();
    strip.show();

    // Load persisted configurations from flash NVS memory
    loadStoredParameters();

    // Sample physical BOOT button state on startup
    bool buttonPressed = (digitalRead(CONFIG_BUTTON_PIN) == LOW);

    if (buttonPressed) {
        Serial.println("[System] Configuration button override detected at boot!");
    }

    // Force Access Point configuration portal if button is pressed or settings are missing
    if (buttonPressed || !isConfigured) {
        startConfigurationPortal();
        loadStoredParameters(); // Re-read updated configuration from NVS
    }

    // Fallback safeguard: If still unconfigured, bypass deep sleep to allow continuous AP access
    if (!isConfigured || WiFi.SSID() == "") {
        Serial.println("[System] Wi-Fi not configured. Staying awake without entering deep sleep...");
        return; // Exits setup directly into loop() fallback handler
    }

    // ----------------------------------------------------------------------------------------
    // STANDARD LOW-POWER WORKFLOW
    // ----------------------------------------------------------------------------------------
    // 1. Measure and filter analog moisture channels for 1 second
    sampleSensorsOneSecond();

    // 2. Connect to network and publish JSON telemetry payload over MQTT
    connectAndPublishMQTT();

    // 3. Pulse status LEDs 3 times, shut off peripheral power, and enter deep sleep
    goToDeepSleep();
}

/**
 * @brief Continuous background execution loop.
 * @note Under normal operation, deep sleep is triggered at the end of setup(), meaning loop()
 *       is never reached. This loop serves exclusively as a fallback mode when Wi-Fi is unconfigured.
 */
void loop() {
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck > 2000) {
        lastCheck = millis();
        Serial.println("[System] Unconfigured fallback mode active. Hold GPIO 0 at boot to configure.");

        // Blink LED 0 Amber to signal unconfigured fallback state
        strip.setPixelColor(0, strip.Color(255, 90, 0));
        strip.show();
        delay(100);
        strip.setPixelColor(0, strip.Color(0, 0, 0));
        strip.show();
    }
}