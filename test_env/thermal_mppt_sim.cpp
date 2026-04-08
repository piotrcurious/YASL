
#include "Arduino.h"
#include "avr/interrupt.h"

void checkAndInitiateSleep();
void processCommand(const char* line);
void validateConfig();
void loadConfig();
void saveConfig();
void readSensors();
float getSmoothedADC(uint8_t pin);
void performCalibration();
void updateMPPT();
void runSMCMPPT();
void updateLight();
void sleepSystem();
void configureWDT();
void disableWDT();
void restoreHardware();
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);


// Content of .ino file

/*
 * YASL (Yet Another Solar Lamp) - Consolidated Version
 *
 * This version combines features from all project iterations:
 * - Simple P&O MPPT with CC/CV (Constant Current/Constant Voltage) safety.
 * - Robust ADC sensor reading with averaging.
 * - PIR motion detection with smooth dimming.
 * - Advanced Sleep Management (WDT for timed wake + INT0 for motion wake).
 * - JSON-formatted Serial output for remote data logging.
 *
 * Hardware: ATmega328P (Uno/Nano), INA219 (Solar Input), PIR Sensor,
 *           Battery Voltage Divider, MOSFET for LED, MOSFET for MPPT Buck.
 */

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <EEPROM.h>
#include <string.h>

// --- Hardware Pins ---
const uint8_t PIN_SOLAR_ADC = A0;   // Raw solar ADC (backup/redundant)
const uint8_t PIN_BAT_ADC   = A1;   // Main Battery Voltage Divider
const uint8_t PIN_LED_PWM   = 3;    // LED Mosfet (Timer2)
const uint8_t PIN_PIR       = 2;    // Motion Sensor (D2 is INT0)
const uint8_t PIN_MPPT_PWM  = 9;    // Solar MPPT Mosfet (Timer1)

// --- Tuning & Defaults ---

// System Reference
#define REF_VOLTAGE             5.0f
#define BAT_DIVIDER_RATIO       3.0f
#define SOLAR_DIVIDER_RATIO     4.0f
#define ADC_SMOOTHING_SAMPLES   8

// Battery Defaults (Li-Ion/Li-Po typical)
#define DEF_BAT_MAX_V           4.15f  // CV Absorption Voltage
#define DEF_BAT_MIN_V           3.00f  // Low Voltage Disconnect
#define DEF_BAT_FLOAT_V         3.45f  // Float Voltage
#define BAT_LOW_SLEEP_PCNT      40.0f  // Sleep if dark and below this %

// Solar & MPPT
#define SOLAR_START_V           5.0f   // Min voltage to start switching
#define SOLAR_KICK_PWM          50     // Initial PWM pulse to start induction
#define SOLAR_DARK_V            2.0f   // Bus voltage below which it's night
#define SOLAR_HYST_V            0.5f   // Hysteresis for day/night transition
#define MPPT_INTERVAL_MS        100
#define MPPT_PWM_MAX_RES        1023   // 10-bit range (Full ICR1)
#define MPPT_PWM_MIN_RES        0

// Sliding Mode Control (SMC) Tuning
#define SMC_BASE_GAIN           2.0f
#define SMC_DV_THRESHOLD        0.05f  // Noise filter for dV
#define SMC_S_HYSTERESIS        0.10f  // Error margin for sliding surface
#define SMC_SENSED_GAIN_MULT    4.0f   // Faster tracking with INA219
#define SMC_SENSORLESS_BIAS     -10.0f // Right-side bias for stability

// Charge Stage Constants
#define TAIL_CURRENT_MA         50.0f  // Absorption -> Float transition
#define ABSORPTION_TIMEOUT_MS   7200000UL // 2 hours max
#define REBULK_FLOAT_DELTA      0.3f   // Restart bulk if V drops this much below Float
#define REBULK_ABS_DELTA        0.2f   // Restart bulk if V drops this much below Abs
#define BAT_OVERVOLT_MARGIN     0.20f  // Shutdown MPPT if Vbat > Max + this
#define CV_REG_MARGIN           0.02f  // Precision for absorption regulation
#define FLOAT_REG_MARGIN        0.05f  // Precision for float regulation

// Sensorless Model Heuristics (Tuned for typical Buck converter)
#define DEF_MODEL_R_CONV        0.20f  // Initial R estimation (Ohms)
#define DEF_MODEL_V_DIODE       0.40f  // Schottky/Body Diode drop
#define CALIB_INTERVAL_MS       600000UL // 10 minutes
#define CALIB_DUTY_RAW          800    // ~78% duty for sampling
#define CALIB_ISC_EST           3.0f   // Panel short-circuit current heuristic
#define CALIB_VT_EST            2.0f   // Thermal voltage scale heuristic
#define CALIB_V_DROP_MIN        1.0f   // Min voltage drop for valid R-calc
#define INFERENCE_MIN_DUTY      0.05f  // Min duty cycle to attempt inference

// Light & Sleep Behavior
#define PWM_LED_OFF             0
#define DEF_LED_MAX_PWM         255
#define DEF_LED_DIM_PWM         15
#define LED_FADE_INTERVAL_MS    10
#define MOTION_CHECK_INTERVAL_MS 1000
#define DEF_MOTION_TIMEOUT_MS   15000UL
#define OVERRIDE_TIMEOUT_MS     300000UL // 5 minutes manual light
#define JSON_INTERVAL_MS        10000UL
#define SLEEP_IDLE_TIMEOUT_MS   300000UL // 5 mins idle before sleep
#define LOOP_STABILITY_DELAY_MS 50

// --- System Structures ---

struct SystemState {
    float solarV;       // Panel Voltage (V)
    float solarI;       // Panel current (mA, Sensed or Inferred)
    float solarP_mW;    // Power (mW, Sensed or Inferred)
    float batV;         // ADC averaged (V)
    float batPcnt;      // 0-100%
    float batMaxToday;
    float batMinToday;
    bool  isDark;
    bool  isMotion;
    int   ledPWM;
    int   mpptPWM;
    char  chargeMode;   // 'N'ight, 'L'ow solar, 'B'ulk (MPPT), 'A'bsorption (CV), 'F'loat, 'X' Overvoltage
    unsigned long absorptionStart;
};

// Permanent Configuration Struct (EEPROM)
struct Config {
    uint32_t magic;         // Validation token
    float    batMaxV;       // Absorption voltage (CV)
    float    batMinV;       // Low Voltage Disconnect
    float    batFloatV;     // Float voltage
    int      ledMaxPWM;
    int      ledDimPWM;
    long     motionTimeout;
};

const uint32_t MAGIC_TOKEN = 0x5941534C; // "YASL"

// Global State
SystemState sys = {0,0,0,0,0,0,0,true,false,0,0,'N', 0};
Config config;
float current_led_val = 0;
bool ina219_present = false;
bool last_dark_state = true;
int motion_intensity = 0; // Cumulative motion tracker for adaptive brightness
bool manual_override = false;
unsigned long manual_override_start = 0;

// MPPT Trackers (Sliding Mode Control)
float prevSolarV = -1.0f; // Flag for initialization
float prevSolarP = -1.0f;
char current_charge_stage = 'B';

// Non-linear Inference Parameters
float model_R_conv = DEF_MODEL_R_CONV;
float model_V_diode = DEF_MODEL_V_DIODE;
float model_duty_prev = 0.0f;

// Timers
unsigned long lastLog = 0;
unsigned long motionStart = 0;
unsigned long lastMppt = 0;
unsigned long lastCalib = 0;

// Wake Flags
volatile bool wakePIR = false;
volatile bool wakeWDT = false;

// --- Prototypes ---
void loadConfig();
void validateConfig();
void saveConfig();
void readSensors();
void updateMPPT();
void runSMCMPPT();
void performCalibration();
void updateLight();
void processCommand(const char* line);
void sleepSystem();
void configureWDT();
void disableWDT();
void restoreHardware();
float getSmoothedADC(uint8_t pin);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

// --- INA219 ---
Adafruit_INA219 ina219;

// --- ISRs ---

ISR(INT0_vect) {
    wakePIR = true;
    // If in LOW-level mode (for sleep wake), disable to prevent wake-loops.
    // In RISING-edge mode (for awake operation), keep enabled for re-triggering.
    if ((EICRA & 0b00000011) == 0) {
        EIMSK &= ~(1 << INT0);
    }
}

ISR(WDT_vect) {
    wakeWDT = true;
}

// --- Arduino Core ---

void setup() {
    // Immediate hardware config
    restoreHardware();

    Serial.begin(115200);
    while (!Serial && millis() < 2000);
    Serial.println(F("YASL CONSOLIDATED v2.0 INIT"));

    // 0. Load Configuration (Now that Serial is ready)
    loadConfig();

    if (!ina219.begin()) {
        Serial.println(F("ERR: INA219 FAILED - Using fallback ADC"));
        ina219_present = false;
    } else {
        Serial.println(F("INA219 OK"));
        ina219_present = true;
    }

    disableWDT();
    readSensors();
}

void checkAndInitiateSleep() {
    bool shouldSleep = false;

    // Do not sleep if user is actively using the lamp via override or motion
    if (manual_override || sys.isMotion) return;

    if (sys.batPcnt < BAT_LOW_SLEEP_PCNT && sys.isDark) {
        shouldSleep = true;
        Serial.println(F("Low battery, initiating timed sleep."));
    } else if (sys.isDark && sys.ledPWM <= config.ledDimPWM &&
               (millis() - motionStart > SLEEP_IDLE_TIMEOUT_MS)) {
        shouldSleep = true;
        Serial.println(F("Dark and idle, initiating timed sleep."));
    }

    if (shouldSleep) {
        sleepSystem();
    }
}

void loop() {
    unsigned long now = millis();

    // 1. Process Wakeup
    if (wakePIR) {
        wakePIR = false;
        sys.isMotion = true;
        motionStart = now;
        motion_intensity++; // Increment on each trigger
        Serial.println(F("WAKE: PIR"));
    }
    if (wakeWDT) {
        wakeWDT = false;
        Serial.println(F("WAKE: WDT"));
    }

    // 2. Refresh Data
    readSensors();

    // 3. Logic: Daytime vs Nighttime with Hysteresis
    if (sys.isDark) {
        if (sys.solarV > SOLAR_DARK_V + SOLAR_HYST_V) sys.isDark = false;
    } else {
        if (sys.solarV < SOLAR_DARK_V) sys.isDark = true;
    }

    // Dawn Detection (Transition from Dark to Light)
    if (last_dark_state && !sys.isDark) {
        Serial.println(F("DAWN: Resetting Stats"));
        sys.batMaxToday = sys.batV;
        sys.batMinToday = sys.batV;
    }
    last_dark_state = sys.isDark;

    if (sys.isDark) {
        // --- NIGHT ---
        sys.mpptPWM = 0;
        OCR1A = 0;
        sys.chargeMode = 'N';

        // Check Manual Override timeout
        if (manual_override && (now - manual_override_start > OVERRIDE_TIMEOUT_MS)) {
            manual_override = false;
            sys.isMotion = false;
            Serial.println(F("Override: Timeout"));
        }

        updateLight();

        // Deep sleep if idle
        if (!sys.isMotion && sys.ledPWM <= config.ledDimPWM && !manual_override) {
            if (now - motionStart > config.motionTimeout + 5000) {
                // Ensure PIR line is quiet before sleeping (prevents immediate wake)
                if (digitalRead(PIN_PIR) == LOW) {
                    sleepSystem();
                } else {
                    motionStart = now; // Stay awake a bit longer
                }
            }
        }
    }
    else {
        // --- DAY ---
        // Light stays off or very dim
        sys.ledPWM = 0;
        analogWrite(PIN_LED_PWM, 0);
        sys.isMotion = false;

        if (now - lastCalib > CALIB_INTERVAL_MS && current_charge_stage == 'B') {
            performCalibration();
        }

        updateMPPT();
    }

    // 4. Logging
    if (now - lastLog > JSON_INTERVAL_MS) {
        Serial.print(F("{\"sV\":")); Serial.print(sys.solarV, 2);
        Serial.print(F(",\"sI_mA\":")); Serial.print(sys.solarI, 1);
        Serial.print(F(",\"sP_mW\":")); Serial.print(sys.solarP_mW, 0);
        Serial.print(F(",\"bV\":")); Serial.print(sys.batV, 2);
        Serial.print(F(",\"bP\":")); Serial.print(sys.batPcnt, 0);
        Serial.print(F(",\"bMax\":")); Serial.print(sys.batMaxToday, 2);
        Serial.print(F(",\"bMin\":")); Serial.print(sys.batMinToday, 2);
        Serial.print(F(",\"dk\":")); Serial.print(sys.isDark ? "true" : "false");
        Serial.print(F(",\"mot\":")); Serial.print(sys.isMotion ? "true" : "false");
        Serial.print(F(",\"mP\":")); Serial.print(sys.mpptPWM);
        Serial.print(F(",\"lP\":")); Serial.print(sys.ledPWM);
        Serial.print(F(",\"ch\":")); Serial.print("\""); Serial.print(sys.chargeMode); Serial.print("\"");
        Serial.println(F("}"));
        lastLog = now;
    }

    // --- Command Processing (Non-Blocking) ---
    static char rxBuffer[32];
    static uint8_t rxIndex = 0;
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            rxBuffer[rxIndex] = '\0';
            if (rxIndex > 0) processCommand(rxBuffer);
            rxIndex = 0;
        } else if (rxIndex < sizeof(rxBuffer) - 1) {
            rxBuffer[rxIndex++] = c;
        }
    }

    checkAndInitiateSleep();
    delay(LOOP_STABILITY_DELAY_MS);
}

void processCommand(const char* line) {
    if (strlen(line) == 0) return;
    unsigned long now = millis();
    char cmd = line[0];

    if (cmd == 'd') { // Diagnostics
        Serial.println(F("--- DIAGNOSTICS ---"));
        Serial.print(F("INA219: ")); Serial.println(ina219_present ? "OK" : "MISSING");
        Serial.print(F("Uptime: ")); Serial.print(now / 1000); Serial.println(F("s"));
        Serial.print(F("Mode: ")); Serial.println(sys.chargeMode);
        Serial.print(F("Intensity: ")); Serial.println(motion_intensity);
#ifdef SIMULATION
        Serial.print(F("Harvested: ")); Serial.print(sim.harvestedMAH); Serial.println(F("mAh"));
        Serial.print(F("Consumed: ")); Serial.print(sim.consumedMAH); Serial.println(F("mAh"));
#endif
    }
    else if (cmd == 'c') { // View Config
        Serial.println(F("--- CONFIG ---"));
        Serial.print(F("BatMax: ")); Serial.println(config.batMaxV);
        Serial.print(F("BatMin: ")); Serial.println(config.batMinV);
        Serial.print(F("LEDMax: ")); Serial.println(config.ledMaxPWM);
        Serial.print(F("Timeout: ")); Serial.println(config.motionTimeout);
    }
    else if (cmd == 'm') { // Manual Light Toggle
        manual_override = !manual_override;
        sys.isMotion = manual_override;
        motionStart = now;
        manual_override_start = now;
        Serial.print(F("Manual Override: ")); Serial.println(manual_override);
    }
    else if (cmd == 's' && strlen(line) > 2) { // Set Param (e.g. sM 4.2)
        char param = line[1];
        float val = atof(&line[2]);
        bool changed = false;
        if (param == 'M' && config.batMaxV != val) { config.batMaxV = val; changed = true; }
        else if (param == 'm' && config.batMinV != val) { config.batMinV = val; changed = true; }
        else if (param == 'F' && config.batFloatV != val) { config.batFloatV = val; changed = true; }
        else if (param == 'T' && (long)config.motionTimeout != (long)val) { config.motionTimeout = (long)val; changed = true; }
        else if (param == 'X' && config.ledMaxPWM != (int)val) { config.ledMaxPWM = (int)val; changed = true; }
        else if (param == 'D' && config.ledDimPWM != (int)val) { config.ledDimPWM = (int)val; changed = true; }

        if (changed) {
            validateConfig();
            saveConfig();
            Serial.print(F("Param ")); Serial.print(param); Serial.println(F(" Saved"));
        }
    }
    else if (cmd == 'v') { // View Stats (Sim only)
#ifdef SIMULATION
        Serial.print(F("Harvested: ")); Serial.print(sim.harvestedMAH); Serial.println(F("mAh"));
        Serial.print(F("Consumed: ")); Serial.print(sim.consumedMAH); Serial.println(F("mAh"));
#endif
    }
    else if (cmd == 'r') { // Reset Defaults
        config.magic = 0;
        loadConfig();
        Serial.println(F("Config Reset"));
    }
    else if (cmd == 'h' || cmd == '?') {
        Serial.println(F("--- HELP ---"));
        Serial.println(F("d: Diag, c: Config, m: Toggle Override, v: Stats"));
        Serial.println(F("sM: BatMax, sm: BatMin, sF: Float, sT: Timeout"));
        Serial.println(F("sX: LEDMax, sD: LEDDim, r: Reset"));
    }
    else if (cmd == 'S' && strlen(line) > 1) { // Manual Stage Force (for testing)
        char stage = line[1];
        if (stage == 'B' || stage == 'A' || stage == 'F') {
            current_charge_stage = stage;
            Serial.print(F("Stage forced to: ")); Serial.println(stage);
        } else {
            Serial.println(F("Invalid stage. Use B, A, or F."));
        }
    }
    else if (cmd == 'k') { // Force Calibration
        performCalibration();
    }
}

// --- Support Functions ---

void validateConfig() {
    // Hard limits for Li-ion/LiFePO4 safety
    if (config.batMinV < 2.50f) config.batMinV = 2.50f;
    if (config.batMinV > 3.50f) config.batMinV = 3.50f;
    if (config.batMaxV > 4.50f) config.batMaxV = 4.50f;
    if (config.batMaxV < 3.00f) config.batMaxV = 3.00f;

    // Ensure Vmin < Vfloat < Vmax with mandatory separation
    if (config.batMaxV < config.batMinV + 0.5f) {
        config.batMaxV = config.batMinV + 0.5f;
    }

    // Float must be between Min and Max
    if (config.batFloatV > config.batMaxV - 0.15f) {
        config.batFloatV = config.batMaxV - 0.15f;
    }
    if (config.batFloatV < config.batMinV + 0.15f) {
        config.batFloatV = config.batMinV + 0.15f;
    }

    // PWM sanity
    config.ledMaxPWM = constrain(config.ledMaxPWM, 10, 255);
    config.ledDimPWM = constrain(config.ledDimPWM, 0, config.ledMaxPWM - 5);

    // Timeout sanity
    if (config.motionTimeout < 1000) config.motionTimeout = 1000;
    if (config.motionTimeout > 3600000) config.motionTimeout = 3600000;
}

void loadConfig() {
    EEPROM.get(0, config);
    if (config.magic != MAGIC_TOKEN) {
        // Factory Defaults
        config.magic = MAGIC_TOKEN;
        config.batMaxV = DEF_BAT_MAX_V;
        config.batMinV = DEF_BAT_MIN_V;
        config.batFloatV = DEF_BAT_FLOAT_V;
        config.ledMaxPWM = DEF_LED_MAX_PWM;
        config.ledDimPWM = DEF_LED_DIM_PWM;
        config.motionTimeout = DEF_MOTION_TIMEOUT_MS;
        validateConfig();
        saveConfig();
        Serial.println(F("EEPROM: Initialized Defaults"));
    } else {
        validateConfig(); // Ensure persistent values are still sane
        Serial.println(F("EEPROM: Loaded Config"));
    }
}

void saveConfig() {
    // EEPROM.put internally uses update() logic on modern Arduino cores,
    // only writing bytes that have changed.
    EEPROM.put(0, config);
}

void readSensors() {
    // 1. Raw Voltages
    if (ina219_present) {
        float bus = ina219.getBusVoltage_V();
        float shunt = ina219.getShuntVoltage_mV();
        sys.solarV = bus + (shunt / 1000.0f);
    } else {
        float rawSolar = getSmoothedADC(PIN_SOLAR_ADC);
        sys.solarV = rawSolar * (REF_VOLTAGE / 1023.0f) * SOLAR_DIVIDER_RATIO;
    }

    float rawBat = getSmoothedADC(PIN_BAT_ADC);
    sys.batV = rawBat * (REF_VOLTAGE / 1023.0f) * BAT_DIVIDER_RATIO;

    // 2. Current Inference (Non-linear Model)
    // Non-ideal Buck: Vbat = (Vsolar * Duty) - (Iout * R_conv) - Vdiode * (1 - Duty)
    // Solving for Iout:
    // Iout = (Vsolar * Duty - Vbat - Vdiode * (1 - Duty)) / R_conv
    // Ipanel = Iout * Duty

    float duty = (float)OCR1A / 1023.0f;

    if (ina219_present) {
        sys.solarI = ina219.getCurrent_mA();
        if (sys.solarI < 0) sys.solarI = 0; // Harvest only
        sys.solarP_mW = sys.solarV * sys.solarI;
    } else if (duty > INFERENCE_MIN_DUTY) {
        // Inference logic (Physically derived)
        float V_comp = sys.batV + model_V_diode * (1.0f - duty);
        float numerator = (sys.solarV * duty) - V_comp;
        if (numerator < 0) numerator = 0;

        float inferred_Iout = numerator / model_R_conv; // Amps
        if (inferred_Iout > 10.0f) inferred_Iout = 10.0f; // Logic clamp
        sys.solarI = inferred_Iout * duty * 1000.0f;    // mA
        sys.solarP_mW = sys.solarV * sys.solarI;
    } else {
        sys.solarI = 0.0f;
        sys.solarP_mW = 0.0f;
    }

    // 3. Update Stats
    sys.batPcnt = constrain(mapFloat(sys.batV, config.batMinV, config.batMaxV, 0, 100), 0, 100);

    // Update Daily Stats
    if (sys.batV > sys.batMaxToday) sys.batMaxToday = sys.batV;
    if (sys.batV < sys.batMinToday || sys.batMinToday < 0.01f) sys.batMinToday = sys.batV;
}

float getSmoothedADC(uint8_t pin) {
    long sum = 0;
    for(int i=0; i < ADC_SMOOTHING_SAMPLES; i++) {
        sum += analogRead(pin);
    }
    return (float)sum / ADC_SMOOTHING_SAMPLES;
}

void performCalibration() {
    lastCalib = millis();
    if (ina219_present) return;

    Serial.println(F("CALIB: Sampling..."));

    // 1. Measure Voc (Averaged)
    int oldPWM = sys.mpptPWM;
    OCR1A = 0;
    delay(250);
    float sumVoc = 0, voc_sq_sum = 0;
    for(int i=0; i<5; i++) {
        readSensors();
        sumVoc += sys.solarV;
        voc_sq_sum += (sys.solarV * sys.solarV);
        delay(30);
    }
    float Voc = sumVoc / 5.0f;
    float Voc_var = (voc_sq_sum / 5.0f) - (Voc * Voc);

    // 2. Measure Under Load (Averaged)
    OCR1A = CALIB_DUTY_RAW;
    delay(250);
    float sumVp = 0, sumVb = 0;
    for(int i=0; i<5; i++) {
        readSensors();
        sumVp += sys.solarV;
        sumVb += sys.batV;
        delay(30);
    }
    float Vpanel = sumVp / 5.0f;
    float Vbat = sumVb / 5.0f;
    float D = (float)CALIB_DUTY_RAW / 1023.0f;

    // 3. Heuristic R Estimation with stability check
    if (Voc_var < 0.05f && Vpanel < Voc - CALIB_V_DROP_MIN) {
        float Ipanel_est = CALIB_ISC_EST * (1.0f - exp((Vpanel - Voc) / CALIB_VT_EST));

        if (Ipanel_est > 0.1f) {
            // Formula: Vbat = Vsolar*D - Iout*R - Vdiode*(1-D)
            // Iout = Ipanel / D
            // R = (Vsolar*D - Vbat - Vdiode*(1-D)) / (Ipanel/D)
            float V_diode_comp = model_V_diode * (1.0f - D);
            float new_R = (Vpanel * D - Vbat - V_diode_comp) / (Ipanel_est / D);

            // Reject outliers and large jumps
            if (new_R > 0.05f && new_R < 1.5f && fabsf(new_R - model_R_conv) < 0.5f) {
                model_R_conv = (model_R_conv * 0.8f) + (new_R * 0.2f); // Heavier EMA for stability
                Serial.print(F("CALIB: R_conv=")); Serial.println(model_R_conv, 3);
            } else {
                Serial.println(F("CALIB: Rejected Sample"));
            }
        }
    }

    OCR1A = oldPWM;
    prevSolarV = -1.0f;
}

void updateMPPT() {
    // 1. Mode Recovery & State Synchronization (Priority 1)
    // Clear fault/low states if conditions are now good, before any early returns.
    if (sys.chargeMode == 'X' && sys.batV < config.batMaxV) {
        sys.chargeMode = current_charge_stage;
    }
    if ((sys.chargeMode == 'N' || sys.chargeMode == 'L') && !sys.isDark && sys.solarV >= SOLAR_START_V) {
        sys.chargeMode = current_charge_stage;
    }

    // 2. Overvoltage Protection (Emergency Cutoff)
    if (sys.batV > config.batMaxV + BAT_OVERVOLT_MARGIN) {
        sys.mpptPWM = 0;
        sys.chargeMode = 'X';
        OCR1A = 0;
        return;
    }

    // 3. Safety & Night check
    if (sys.solarV < SOLAR_START_V || sys.isDark) {
        sys.mpptPWM = 0;
        if (sys.isDark) sys.chargeMode = 'N';
        else sys.chargeMode = 'L'; // Low solar

        // Note: charge stage is preserved through brief solar dips
        OCR1A = 0;
        return;
    }

    // 4. MPPT Startup Kick: If idle but solar is present, give it a nudge
    if (sys.mpptPWM == 0 && sys.solarV > SOLAR_START_V) {
        sys.mpptPWM = SOLAR_KICK_PWM;
        OCR1A = sys.mpptPWM;
        prevSolarV = -1.0f; // Ensure trackers reset after kick
        return;
    }

    // 3-Stage Logic
    if (current_charge_stage == 'B') { // Bulk (MPPT)
        sys.chargeMode = 'B';
        if (sys.batV >= config.batMaxV) {
            current_charge_stage = 'A'; // To Absorption
            sys.absorptionStart = millis();
            prevSolarV = -1.0f; // Reset trackers on stage change
        } else {
            runSMCMPPT();
        }
    }
    else if (current_charge_stage == 'A') { // Absorption (CV)
        sys.chargeMode = 'A';
        // Maintain config.batMaxV
        if (sys.batV > config.batMaxV) {
            if (sys.mpptPWM > 0) sys.mpptPWM--;
        } else if (sys.batV < config.batMaxV - CV_REG_MARGIN) {
            if (sys.mpptPWM < MPPT_PWM_MAX_RES) sys.mpptPWM++;
        }

        // Transition to Float after current drops (Tail Current) or timeout.
        // Battery current is inferred from the converter model: Ibat = Isolar / Duty.
        // NOTE: In sensorless mode, this is a HEURISTIC based on inferred power.
        // It helps prevent overcharging if Absorption is sustained too long.
        float duty = (float)OCR1A / 1023.0f;
        float batCurrentMA = (duty > 0.1f) ? (sys.solarI / duty) : 0;

        bool is_tail_current_heuristic = (batCurrentMA < TAIL_CURRENT_MA && batCurrentMA > 0);
        bool is_abs_timeout = (millis() - sys.absorptionStart > ABSORPTION_TIMEOUT_MS);

        if (is_tail_current_heuristic || is_abs_timeout) {
            current_charge_stage = 'F';
            prevSolarV = -1.0f;
        }
        if (sys.batV < config.batMinV + REBULK_ABS_DELTA) {
            current_charge_stage = 'B'; // Deep discharge
            prevSolarV = -1.0f;
        }
    }
    else if (current_charge_stage == 'F') { // Float
        sys.chargeMode = 'F';
        if (sys.batV > config.batFloatV) {
            if (sys.mpptPWM > 0) sys.mpptPWM--;
        } else if (sys.batV < config.batFloatV - FLOAT_REG_MARGIN) {
            if (sys.mpptPWM < MPPT_PWM_MAX_RES) sys.mpptPWM++;
        }
        if (sys.batV < config.batFloatV - REBULK_FLOAT_DELTA) {
            current_charge_stage = 'B'; // Re-bulk
            prevSolarV = -1.0f;
        }
    }

    // MPPT PWM is now 10-bit (0-1023)
    sys.mpptPWM = constrain(sys.mpptPWM, MPPT_PWM_MIN_RES, MPPT_PWM_MAX_RES);
    OCR1A = sys.mpptPWM;
}

void runSMCMPPT() {
    // Initializing trackers on first run
    if (prevSolarV < 0) {
        prevSolarV = sys.solarV;
        prevSolarP = sys.solarP_mW;
        return;
    }

    unsigned long now = millis();
    if (now - lastMppt > MPPT_INTERVAL_MS) {
        float dv = sys.solarV - prevSolarV;
        float dp = sys.solarP_mW - prevSolarP;

        // Sliding Mode Surface S = dP/dV
        // Goal: S = 0 at MPP.

        if (fabsf(dv) > SMC_DV_THRESHOLD) {
            float S = dp / dv;

            // Sliding Mode Gain - Adaptive based on confidence
            float current_gain = (ina219_present) ? (SMC_BASE_GAIN * SMC_SENSED_GAIN_MULT) : SMC_BASE_GAIN;

            // Bias: In sensorless mode, we bias towards the right side (Higher Vsolar)
            // to avoid voltage collapse when inference error is high.
            float target_S = (ina219_present) ? 0.0f : SMC_SENSORLESS_BIAS;

            if (S > target_S + SMC_S_HYSTERESIS) {
                if (sys.mpptPWM > (int)current_gain) sys.mpptPWM -= (int)current_gain;
            } else if (S < target_S - SMC_S_HYSTERESIS) {
                if (sys.mpptPWM < MPPT_PWM_MAX_RES) sys.mpptPWM += (int)current_gain;
            }

            prevSolarV = sys.solarV;
            prevSolarP = sys.solarP_mW;
        } else {
            // Small perturbation if dv is zero
            sys.mpptPWM += (int)SMC_BASE_GAIN;
            // Update trackers even during perturbation to keep gradients fresh
            prevSolarV = sys.solarV;
            prevSolarP = sys.solarP_mW;
        }
        lastMppt = now;
    }
}

void updateLight() {
    static unsigned long last_fade = 0;
    static unsigned long last_motion_check = 0;
    static bool lvd_active = false;
    unsigned long now = millis();

    // Low Voltage Disconnect with Hysteresis
    if (!lvd_active && sys.batV < config.batMinV) {
        lvd_active = true;
        sys.ledPWM = PWM_LED_OFF;
        motion_intensity = 0;
        Serial.println(F("LVD: Active"));
    } else if (lvd_active && sys.batV > config.batMinV + 0.25f) {
        lvd_active = false;
        Serial.println(F("LVD: Recovered"));
    }

    if (lvd_active) {
        sys.ledPWM = PWM_LED_OFF;
        current_led_val = 0;
        analogWrite(PIN_LED_PWM, 0); // Forced immediate cut
        motion_intensity = 0;
    }
    else if (sys.isMotion) {
        if (now - motionStart < config.motionTimeout) {
            // Adaptive Brightness (Float-based for smoothness)
            float intensity_scale = mapFloat(constrain(motion_intensity, 1, 5), 1, 5, 0.5f, 1.0f);
            float adaptive_max = config.ledMaxPWM * intensity_scale;

            float bat_limit = mapFloat(sys.batV, config.batMinV, config.batMaxV, config.ledDimPWM, adaptive_max);
            sys.ledPWM = (int)constrain(bat_limit, config.ledDimPWM, config.ledMaxPWM);

            if (digitalRead(PIN_PIR) == HIGH && (now - last_motion_check > MOTION_CHECK_INTERVAL_MS)) {
                motion_intensity = constrain(motion_intensity + 1, 0, 10);
                last_motion_check = now;
            }
        } else {
            sys.isMotion = false;
            motion_intensity = 0;
            sys.ledPWM = config.ledDimPWM;
        }
    } else {
        sys.ledPWM = config.ledDimPWM;
        motion_intensity = 0;
    }

    // Smooth time-based PWM transition (independent of loop rate)
    if (now - last_fade > LED_FADE_INTERVAL_MS) {
        if (current_led_val < sys.ledPWM) current_led_val += 1.0;
        else if (current_led_val > sys.ledPWM) current_led_val -= 1.0;
        analogWrite(PIN_LED_PWM, (int)current_led_val);
        last_fade = now;
    }
}

void sleepSystem() {
    Serial.println(F("SLEEP: Start"));
    Serial.flush();

    // Kill outputs
    analogWrite(PIN_LED_PWM, 0);
    current_led_val = 0; // Reset software fade state
    OCR1A = 0; // MPPT 10-bit PWM off

    // Peripheral Shutdown
    power_all_disable();

    // WDT for periodic health check (8s)
    configureWDT();

    // INT0 for PIR wake
    // Use LOW level for reliable PWR_DOWN wake on older AVRs.
    EICRA &= ~((1 << ISC01) | (1 << ISC00));
    EIFR = (1 << INTF0);
    EIMSK |= (1 << INT0);

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sei();

    sleep_cpu(); // HALT

    // --- WAKE UP ---
    sleep_disable();
    disableWDT();
    power_all_enable();

    // Restore Peripherals
    restoreHardware(); // Ensure timers/pinmodes are back
    Serial.begin(115200);
    Wire.begin();
    if (ina219_present) {
        if (!ina219.begin()) {
            Serial.println(F("INA219: Recovery Failed"));
            ina219_present = false;
        }
    }

    // Reconfigure INT0 for RISING edge for normal awake operation
    EICRA = (1 << ISC01) | (1 << ISC00);
    EIFR = (1 << INTF0);
    EIMSK |= (1 << INT0);

    prevSolarV = -1.0f; // Force MPPT reset after sleep gap
    Serial.println(F("SLEEP: Woke up"));
}

void configureWDT() {
    cli();
    wdt_reset();
    // Clear WDRF in MCUSR
    MCUSR &= ~(1 << WDRF);
    // Write 1 to WDCE and WDE to allow changes
    WDTCSR = (1 << WDCE) | (1 << WDE);
    // Set new watchdog timeout value and enable interrupt mode (not reset)
    WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);
    sei();
}

void disableWDT() {
    cli();
    wdt_reset();
    MCUSR &= ~(1 << WDRF);
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    WDTCSR = 0x00;
    sei();
}

void restoreHardware() {
    pinMode(PIN_MPPT_PWM, OUTPUT);
    digitalWrite(PIN_MPPT_PWM, LOW);
    pinMode(PIN_LED_PWM, OUTPUT);
    digitalWrite(PIN_LED_PWM, LOW);
    pinMode(PIN_PIR, INPUT_PULLUP);

    // --- High Frequency 10-bit PWM for MPPT (Timer1) ---
    // Pin 9 (OC1A)
    TCCR1A = _BV(COM1A1) | _BV(WGM11); // Fast PWM, non-inverting, mode 14
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // mode 14, no prescaler
    ICR1 = 1023; // 10-bit resolution -> 16MHz / 1024 = 15.6 kHz
    OCR1A = sys.mpptPWM;

    // Set Timer2 (pins 3, 11) for ~976Hz PWM
    TCCR2B = (TCCR2B & 0b11111000) | 0x04;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



int main() {
    sim.ina219_ok = false; // Force sensorless mode
    sim.R_conv_base = 0.2f;
    sim.tempC = 25.0f;
    sim.batteryV = 3.5f;
    sim.solarOCV = 18.0f;

    setup();

    std::cout << "\n[SCENARIO] Sensorless MPPT start" << std::endl;
    for(int i = 0; i < 50; ++i) {
        loop();
        update_sim();
    }

    std::cout << "\n[SCENARIO] Thermal Drift (Heating up to 75C)" << std::endl;
    sim.tempC = 75.0f;
    for(int i = 0; i < 100; ++i) {
        loop();
        update_sim();
    }

    std::cout << "\n[SCENARIO] Triggering Calibration" << std::endl;
    Serial.sim_input("k\n");
    for(int i = 0; i < 20; ++i) {
        loop();
        update_sim();
    }

    std::cout << "\n[SCENARIO] Re-evaluating MPPT performance" << std::endl;
    for(int i = 0; i < 50; ++i) {
        loop();
        update_sim();
    }

    return 0;
}
