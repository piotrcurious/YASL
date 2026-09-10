/*
 * YASL (Yet Another Solar Lamp) - Consolidated Version
 * Refactored for Safety, Non-Blocking execution, Safe Deadtime, and PCINT PIR Motion Sensing.
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
const uint8_t PIN_SOLAR_ADC = A0;   // Raw solar ADC (backup)
const uint8_t PIN_BAT_ADC   = A1;   // Main Battery Voltage Divider
const uint8_t PIN_LED_PWM   = 3;    // LED Mosfet (Timer2)
const uint8_t PIN_PIR       = 2;    // Motion Sensor (D2 is INT0 / PCINT18)
const uint8_t PIN_MPPT_PWM  = 9;    // Solar MPPT (Main FET, Timer1 OC1A)
const uint8_t PIN_MPPT_SYNC = 10;   // Solar MPPT (Sync FET, Timer1 OC1B)

// --- Tuning & Defaults ---
#define USE_INTERNAL_1V1_REF    false

#define BAT_DIVIDER_RATIO_DEF   3.0f
#define SOLAR_DIVIDER_RATIO_DEF 4.0f
#define BAT_DIVIDER_RATIO_INT   5.54f
#define SOLAR_DIVIDER_RATIO_INT 31.3f

#define REF_VOLTAGE             5.0f
#define BAT_DIVIDER_RATIO       (USE_INTERNAL_1V1_REF ? BAT_DIVIDER_RATIO_INT : BAT_DIVIDER_RATIO_DEF)
#define SOLAR_DIVIDER_RATIO     (USE_INTERNAL_1V1_REF ? SOLAR_DIVIDER_RATIO_INT : SOLAR_DIVIDER_RATIO_DEF)
#define ADC_SMOOTHING_SAMPLES   8

#define DEF_BAT_MAX_V           4.15f
#define DEF_BAT_MIN_V           3.00f
#define DEF_BAT_FLOAT_V         3.45f
#define BAT_LOW_SLEEP_PCNT      40.0f

#define SOLAR_START_V_MIN       4.5f
#define SOLAR_DARK_V            2.0f
#define SOLAR_HYST_V            0.5f
#define MPPT_INTERVAL_MS        100
#define MPPT_PWM_MAX_RES        1023
#define MPPT_PWM_MIN_RES        0
#define SYNC_FET_DEADTIME       12     // ~0.75us deadtime

#define SMC_BASE_GAIN           2.0f
#define SMC_MIN_GAIN            0.5f
#define SMC_DV_THRESHOLD        0.05f
#define SMC_S_HYSTERESIS        0.10f
#define SMC_SENSED_GAIN_MULT    4.0f
#define SMC_SENSORLESS_BIAS     -10.0f

#define TAIL_CURRENT_MA         50.0f
#define ABSORPTION_TIMEOUT_MS   7200000UL
#define REBULK_FLOAT_DELTA      0.3f
#define REBULK_ABS_DELTA        0.2f
#define BAT_OVERVOLT_MARGIN     0.20f
#define CV_REG_MARGIN           0.02f
#define FLOAT_REG_MARGIN        0.05f

#define DEF_MODEL_R_CONV        0.20f
#define DEF_MODEL_V_DIODE       0.40f
#define CALIB_INTERVAL_MS       600000UL
#define CALIB_DUTY_RAW          800
#define CALIB_ISC_EST           3.0f
#define CALIB_VT_EST            2.0f
#define CALIB_V_DROP_MIN        1.0f
#define INFERENCE_MIN_DUTY      0.03f

#define PWM_LED_OFF             0
#define DEF_LED_MAX_PWM         255
#define DEF_LED_DIM_PWM         15
#define LED_FADE_INTERVAL_MS    10
#define MOTION_CHECK_INTERVAL_MS 1000
#define DEF_MOTION_TIMEOUT_MS   15000UL
#define OVERRIDE_TIMEOUT_MS     300000UL
#define JSON_INTERVAL_MS        10000UL
#define SLEEP_IDLE_TIMEOUT_MS   300000UL
#define LOOP_TICK_RATE_MS       50
#define TRANSITION_DEBOUNCE_MS  60000UL

// --- System Structures ---
struct SystemState {
    float solarV;
    float solarI;
    float solarP_mW;
    float batV;
    float batPcnt;
    float batMaxToday;
    float batMinToday;
    bool  isDark;
    bool  isMotion;
    int   ledPWM;
    int   mpptPWM;
    char  chargeMode;
    unsigned long absorptionStart;
};

struct Config {
    uint32_t magic;
    float    batMaxV;
    float    batMinV;
    float    batFloatV;
    uint32_t motionTimeout;
    uint16_t ledMaxPWM;
    uint16_t ledDimPWM;
};

const uint32_t MAGIC_TOKEN = 0x5941534D; // Refresh token for layout update

// Global State
SystemState sys = {0,0,0,0,0,0,0,true,false,0,0,'N', 0};
Config config;
float current_adc_scale = REF_VOLTAGE;
float current_led_val = 0;
bool ina219_present = false;
bool last_dark_state = true;
int motion_intensity = 0;
bool manual_override = false;
unsigned long manual_override_start = 0;

float prevSolarV = -1.0f;
float prevSolarP = -1.0f;
char current_charge_stage = 'B';
float model_R_conv = DEF_MODEL_R_CONV;
float model_V_diode = DEF_MODEL_V_DIODE;

// Timers
unsigned long lastLog = 0;
unsigned long motionStart = 0;
unsigned long lastMppt = 0;
unsigned long lastCalib = 0;
unsigned long lastInaRetry = 0;
unsigned long lastDarkTransition = 0;
unsigned long lastTick = 0;

volatile bool wakePIR = false;
volatile bool wakeWDT = false;

Adafruit_INA219 ina219;

// --- Prototypes ---
void loadConfig();
void validateConfig();
void saveConfig();
void readSensors();
void updateMPPT();
void runSMCMPPT();
void performCalibration();
void updateLight();
void updateLightFade();
void processCommand(const char* line);
void handleSerial();
void checkAndInitiateSleep();
void sleepSystem();
void enableActiveWDT();
void configureSleepWDT();
void restoreHardware();
float getSmoothedADC(uint8_t pin);
float readVcc();

// --- ISRs ---
// Pin Change Interrupt for Port D (Pin 2 / PCINT18)
// Handles HIGH signal from typical PIR motion sensors
ISR(PCINT2_vect) {
    if (digitalRead(PIN_PIR) == HIGH) {
        wakePIR = true;
    }
}

ISR(WDT_vect) {
    wakeWDT = true;
}

// --- Setup ---
void setup() {
    restoreHardware();

#if USE_INTERNAL_1V1_REF
    analogReference(INTERNAL);
#endif

    Wire.begin();
    Serial.begin(115200);

    // Clear watchdog reset flags to prevent boot loops
    MCUSR &= ~(1 << WDRF);
    enableActiveWDT(); // 4-second safety watchdog reset mode

    while (!Serial && millis() < 2000) wdt_reset();
    Serial.println(F("YASL CONSOLIDATED v2.5 INIT"));

    loadConfig();

    if (!ina219.begin()) {
        Serial.println(F("ERR: INA219 FAILED - Using fallback ADC"));
        ina219_present = false;
    } else {
        Serial.println(F("INA219 OK"));
        ina219_present = true;
    }

    lastDarkTransition = 0;
    readSensors();
}

// --- Loop ---
void loop() {
    unsigned long now = millis();
    wdt_reset(); // Feed active watchdog timer

    handleSerial(); // Instant non-blocking serial reading

    // 1. Process Wakeup Flags
    if (wakePIR) {
        wakePIR = false;
        sys.isMotion = true;
        motionStart = now;
        motion_intensity++;
        Serial.println(F("WAKE: PIR"));
    }
    if (wakeWDT) {
        wakeWDT = false;
    }

    // 2. Non-blocking Tick Loop (50ms)
    if (now - lastTick >= LOOP_TICK_RATE_MS) {
        lastTick = now;

        readSensors();

        // Daytime vs Nighttime with Hysteresis and Debounce
        bool potential_dark = (sys.solarV < SOLAR_DARK_V);
        if (sys.isDark && sys.solarV > SOLAR_DARK_V + SOLAR_HYST_V) potential_dark = false;
        else if (!sys.isDark && sys.solarV < SOLAR_DARK_V) potential_dark = true;
        else potential_dark = sys.isDark;

        if (potential_dark != sys.isDark) {
            if (lastDarkTransition == 0) lastDarkTransition = now;
            if (now - lastDarkTransition > TRANSITION_DEBOUNCE_MS) {
                sys.isDark = potential_dark;
                lastDarkTransition = 0;
                Serial.print(F("State Change: ")); Serial.println(sys.isDark ? F("NIGHT") : F("DAY"));
            }
        } else {
            lastDarkTransition = 0;
        }

        // Dawn Detection
        if (last_dark_state && !sys.isDark) {
            Serial.println(F("DAWN: Resetting Stats"));
            sys.batMaxToday = sys.batV;
            sys.batMinToday = sys.batV;
        }
        last_dark_state = sys.isDark;

        if (sys.isDark) {
            sys.mpptPWM = 0;
            OCR1A = 0;
            OCR1B = MPPT_PWM_MAX_RES;
            sys.chargeMode = 'N';

            if (manual_override && (now - manual_override_start > OVERRIDE_TIMEOUT_MS)) {
                manual_override = false;
                sys.isMotion = false;
                Serial.println(F("Override: Timeout"));
            }

            updateLight();

            if (!sys.isMotion && sys.ledPWM <= config.ledDimPWM && !manual_override) {
                if (now - motionStart > config.motionTimeout + 5000) {
                    if (digitalRead(PIN_PIR) == LOW) {
                        checkAndInitiateSleep();
                    } else {
                        motionStart = now;
                    }
                }
            }
        }
        else {
            sys.ledPWM = 0;
            analogWrite(PIN_LED_PWM, 0);
            sys.isMotion = false;

            if (now - lastCalib > CALIB_INTERVAL_MS && current_charge_stage == 'B') {
                performCalibration();
            }

            updateMPPT();
        }
    }

    // 3. LED Smooth Fade Execution (Independent of 50ms tick loop)
    updateLightFade();

    // 4. Telemetry Logging (10s)
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
}

void handleSerial() {
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
}

void checkAndInitiateSleep() {
    if (manual_override || sys.isMotion) return;
    if ((sys.batPcnt < BAT_LOW_SLEEP_PCNT && sys.isDark) ||
        (sys.isDark && sys.ledPWM <= config.ledDimPWM && (millis() - motionStart > SLEEP_IDLE_TIMEOUT_MS))) {
        sleepSystem();
    }
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
    }
    else if (cmd == 'c') { // View Config
        Serial.println(F("--- CONFIG ---"));
        Serial.print(F("BatMax: ")); Serial.println(config.batMaxV);
        Serial.print(F("BatMin: ")); Serial.println(config.batMinV);
        Serial.print(F("LEDMax: ")); Serial.println(config.ledMaxPWM);
        Serial.print(F("Timeout: ")); Serial.println((unsigned long)config.motionTimeout);
    }
    else if (cmd == 'm') { // Manual Light Toggle
        manual_override = !manual_override;
        sys.isMotion = manual_override;
        motionStart = now;
        manual_override_start = now;
        Serial.print(F("Manual Override: ")); Serial.println(manual_override);
    }
    else if (cmd == 's' && strlen(line) > 2) { // Set Param
        char param = line[1];
        float val = atof(&line[2]);
        bool changed = false;
        if (param == 'M' && config.batMaxV != val) { config.batMaxV = val; changed = true; }
        else if (param == 'm' && config.batMinV != val) { config.batMinV = val; changed = true; }
        else if (param == 'F' && config.batFloatV != val) { config.batFloatV = val; changed = true; }
        else if (param == 'T' && config.motionTimeout != (uint32_t)val) { config.motionTimeout = (uint32_t)val; changed = true; }
        else if (param == 'X' && config.ledMaxPWM != (uint16_t)val) { config.ledMaxPWM = (uint16_t)val; changed = true; }
        else if (param == 'D' && config.ledDimPWM != (uint16_t)val) { config.ledDimPWM = (uint16_t)val; changed = true; }

        if (changed) {
            validateConfig();
            saveConfig();
            Serial.print(F("Param ")); Serial.print(param); Serial.println(F(" Saved"));
        }
    }
    else if (cmd == 'r') { // Reset Defaults
        config.magic = 0;
        saveConfig();
        loadConfig();
        Serial.println(F("Config Reset"));
    }
    else if (cmd == 'h' || cmd == '?') {
        Serial.println(F("--- HELP ---"));
        Serial.println(F("d: Diag, c: Config, m: Toggle Override"));
        Serial.println(F("sM: BatMax, sm: BatMin, sF: Float, sT: Timeout"));
        Serial.println(F("sX: LEDMax, sD: LEDDim, r: Reset"));
    }
    else if (cmd == 'S' && strlen(line) > 1) { // Manual Stage Force
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

void validateConfig() {
    if (config.batMinV < 2.50f) config.batMinV = 2.50f;
    if (config.batMinV > 3.50f) config.batMinV = 3.50f;
    if (config.batMaxV > 4.50f) config.batMaxV = 4.50f;
    if (config.batMaxV < 3.00f) config.batMaxV = 3.00f;

    if (config.batMaxV < config.batMinV + 0.5f) {
        config.batMaxV = config.batMinV + 0.5f;
    }

    if (config.batFloatV > config.batMaxV - 0.15f) {
        config.batFloatV = config.batMaxV - 0.15f;
    }
    if (config.batFloatV < config.batMinV + 0.15f) {
        config.batFloatV = config.batMinV + 0.15f;
    }

    config.ledMaxPWM = (uint16_t)constrain(config.ledMaxPWM, 10, 255);
    config.ledDimPWM = (uint16_t)constrain(config.ledDimPWM, 0, config.ledMaxPWM - 5);

    if (config.motionTimeout < 1000) config.motionTimeout = 1000;
    if (config.motionTimeout > 3600000) config.motionTimeout = 3600000;
}

void loadConfig() {
    EEPROM.get(0, config);
    if (config.magic != MAGIC_TOKEN) {
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
        validateConfig();
        Serial.println(F("EEPROM: Loaded Config"));
    }
}

void saveConfig() {
    EEPROM.put(0, config);
}

void readSensors() {
    unsigned long now = millis();

    // Optimize Vcc check to run every 5s instead of every cycle
    static unsigned long lastVccCheck = 0;
    if (now - lastVccCheck > 5000 || lastVccCheck == 0) {
#if USE_INTERNAL_1V1_REF
        current_adc_scale = 1.1f;
#else
        current_adc_scale = readVcc();
#endif
        lastVccCheck = now;
    }

    if (ina219_present) {
        Wire.beginTransmission(0x40);
        if (Wire.endTransmission() != 0) {
            ina219_present = false;
            lastInaRetry = now;
        }
    } else if (now - lastInaRetry > 30000UL) {
        if (ina219.begin()) {
            ina219_present = true;
            prevSolarV = -1.0f;
            lastMppt = now;
        }
        lastInaRetry = now;
    }

    if (ina219_present) {
        sys.solarV = ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV() / 1000.0f);
        sys.solarI = max(0.0f, ina219.getCurrent_mA());
        sys.solarP_mW = sys.solarV * sys.solarI;
    } else {
        sys.solarV = getSmoothedADC(PIN_SOLAR_ADC) * (current_adc_scale / 1023.0f) * SOLAR_DIVIDER_RATIO;
        float duty = (float)OCR1A / 1023.0f;
        if (duty > INFERENCE_MIN_DUTY) {
            float d_clamped = max(duty, 0.01f);
            float eff_vdiode = (duty > 0.01f) ? 0.02f : model_V_diode;
            float V_comp = sys.batV + eff_vdiode * (1.0f - d_clamped);
            float numerator = max(0.0f, (sys.solarV * d_clamped) - V_comp);

            float inferred_Iout = min(numerator / model_R_conv, 10.0f);
            sys.solarI = inferred_Iout * duty * 1000.0f;
            sys.solarP_mW = sys.solarV * sys.solarI;
        } else {
            sys.solarI = 0.0f;
            sys.solarP_mW = 0.0f;
        }
    }

    sys.batV = getSmoothedADC(PIN_BAT_ADC) * (current_adc_scale / 1023.0f) * BAT_DIVIDER_RATIO;

    float pcnt_calc = (sys.batV - config.batMinV) * 100.0f / (config.batMaxV - config.batMinV);
    sys.batPcnt = constrain(pcnt_calc, 0.0f, 100.0f);

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

float readVcc() {
#ifdef SIMULATION
    int result = analogRead(14);
    return 1.1f * 1023.0f / (float)result;
#else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(2);
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA, ADSC));
    uint8_t low  = ADCL;
    uint8_t high = ADCH;
    long result = (high << 8) | low;
    return 1125.3f / (float)result;
#endif
}

void performCalibration() {
    lastCalib = millis();
    if (ina219_present) return;

    Serial.println(F("CALIB: Sampling..."));

    int oldPWM = sys.mpptPWM;
    OCR1A = 0;
    OCR1B = MPPT_PWM_MAX_RES;
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

    prevSolarV = -1.0f;
    lastMppt = millis();

    if (Voc_var < 0.05f && Vpanel < Voc - CALIB_V_DROP_MIN) {
        float Ipanel_est = CALIB_ISC_EST * (1.0f - exp((Vpanel - Voc) / CALIB_VT_EST));

        if (Ipanel_est > 0.1f) {
            float V_diode_comp = model_V_diode * (1.0f - D);
            float new_R = (Vpanel * D - Vbat - V_diode_comp) / (Ipanel_est / D);

            if (new_R > 0.05f && new_R < 1.5f && fabsf(new_R - model_R_conv) < 0.5f) {
                model_R_conv = (model_R_conv * 0.8f) + (new_R * 0.2f);
                Serial.print(F("CALIB: R_conv=")); Serial.println(model_R_conv, 3);
            } else {
                Serial.println(F("CALIB: Rejected Sample"));
            }
        }
    }

    OCR1A = oldPWM;
    if (oldPWM > 50) OCR1B = constrain(oldPWM + SYNC_FET_DEADTIME, 0, MPPT_PWM_MAX_RES);
    else OCR1B = MPPT_PWM_MAX_RES;
    prevSolarV = -1.0f;
}

void updateMPPT() {
    unsigned long now = millis();
    float dynamic_start_v = max(SOLAR_START_V_MIN, sys.batV + 0.5f);

    if (sys.chargeMode == 'X' && sys.batV < config.batMaxV) {
        sys.chargeMode = current_charge_stage;
    }
    if ((sys.chargeMode == 'N' || sys.chargeMode == 'L') && !sys.isDark && sys.solarV >= dynamic_start_v) {
        sys.chargeMode = current_charge_stage;
    }

    if (sys.batV > config.batMaxV + BAT_OVERVOLT_MARGIN) {
        sys.mpptPWM = 0;
        sys.chargeMode = 'X';
        OCR1A = 0;
        OCR1B = MPPT_PWM_MAX_RES;
        return;
    }

    if (sys.solarV < dynamic_start_v || sys.isDark) {
        sys.mpptPWM = 0;
        sys.chargeMode = sys.isDark ? 'N' : 'L';
        OCR1A = 0;
        OCR1B = MPPT_PWM_MAX_RES;
        return;
    }

    if (current_charge_stage == 'B') {
        if (sys.mpptPWM == 0 && sys.solarV > dynamic_start_v) {
            float predictive_duty = (sys.batV + 0.5f) / sys.solarV;
            sys.mpptPWM = constrain((int)(predictive_duty * 1024.0f), 100, 800);
            prevSolarV = -1.0f;
            lastMppt = now;
            sys.chargeMode = current_charge_stage;
            Serial.print(F("MPPT Kick: ")); Serial.println(sys.mpptPWM);
        }

        if (sys.batV >= config.batMaxV) {
            current_charge_stage = 'A';
            sys.absorptionStart = now;
            prevSolarV = -1.0f;
            lastMppt = now;
        } else {
            runSMCMPPT();
        }
    }
    else if (current_charge_stage == 'A') {
        if (sys.batV > config.batMaxV) {
            if (sys.mpptPWM > 0) sys.mpptPWM--;
        } else if (sys.batV < config.batMaxV - CV_REG_MARGIN) {
            if (sys.mpptPWM < MPPT_PWM_MAX_RES) sys.mpptPWM++;
        }

        float duty = (float)OCR1A / 1023.0f;
        float batCurrentMA = (duty > 0.1f) ? (sys.solarI / duty) : 0;

        bool is_tail_current = (batCurrentMA < TAIL_CURRENT_MA && batCurrentMA > 0);
        bool is_abs_timeout = (millis() - sys.absorptionStart > ABSORPTION_TIMEOUT_MS);

        if (is_tail_current || is_abs_timeout) {
            current_charge_stage = 'F';
            prevSolarV = -1.0f;
            lastMppt = now;
        }
        if (sys.batV < config.batMinV + REBULK_ABS_DELTA) {
            current_charge_stage = 'B';
            prevSolarV = -1.0f;
            lastMppt = now;
        }
    }
    else if (current_charge_stage == 'F') {
        if (sys.batV > config.batFloatV) {
            if (sys.mpptPWM > 0) sys.mpptPWM--;
        } else if (sys.batV < config.batFloatV - FLOAT_REG_MARGIN) {
            if (sys.mpptPWM < MPPT_PWM_MAX_RES) sys.mpptPWM++;
        }
        if (sys.batV < config.batFloatV - REBULK_FLOAT_DELTA) {
            current_charge_stage = 'B';
            prevSolarV = -1.0f;
            lastMppt = now;
        }
    }

    sys.chargeMode = current_charge_stage;

    // CRITICAL FIX: Clamp max PWM to leave room for deadtime to prevent shoot-through / overflow
    int safe_max_pwm = MPPT_PWM_MAX_RES - SYNC_FET_DEADTIME;
    sys.mpptPWM = constrain(sys.mpptPWM, MPPT_PWM_MIN_RES, safe_max_pwm);

    OCR1A = sys.mpptPWM;
    if (sys.mpptPWM > 50) {
        OCR1B = sys.mpptPWM + SYNC_FET_DEADTIME;
    } else {
        OCR1B = MPPT_PWM_MAX_RES;
    }
}

void runSMCMPPT() {
    if (prevSolarV < 0) {
        prevSolarV = sys.solarV;
        prevSolarP = sys.solarP_mW;
        return;
    }

    unsigned long now = millis();
    if (now - lastMppt > MPPT_INTERVAL_MS) {
        float dv = sys.solarV - prevSolarV;
        float dp = sys.solarP_mW - prevSolarP;

        if (fabsf(dv) > SMC_DV_THRESHOLD) {
            float S = dp / dv;
            float current_gain = (ina219_present) ? (SMC_BASE_GAIN * SMC_SENSED_GAIN_MULT) : SMC_BASE_GAIN;

            if (sys.solarP_mW < 5000.0f) {
                float power_scale = (sys.solarP_mW - 0.0f) * (1.0f - 0.2f) / (5000.0f - 0.0f) + 0.2f;
                current_gain *= power_scale;
                if (current_gain < SMC_MIN_GAIN) current_gain = SMC_MIN_GAIN;
            }

            float target_S = (ina219_present) ? 0.0f : SMC_SENSORLESS_BIAS;

            if (S > target_S + SMC_S_HYSTERESIS) {
                if (sys.mpptPWM > 0) sys.mpptPWM -= (int)ceil(current_gain);
            } else if (S < target_S - SMC_S_HYSTERESIS) {
                if (sys.mpptPWM < MPPT_PWM_MAX_RES) sys.mpptPWM += (int)ceil(current_gain);
            }

            prevSolarV = sys.solarV;
            prevSolarP = sys.solarP_mW;
        } else {
            sys.mpptPWM += (int)SMC_BASE_GAIN;
            prevSolarV = sys.solarV;
            prevSolarP = sys.solarP_mW;
        }
        lastMppt = now;
    }
}

void updateLight() {
    static bool lvd_active = false;
    unsigned long now = millis();

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
        analogWrite(PIN_LED_PWM, 0);
        motion_intensity = 0;
    }
    else if (sys.isMotion) {
        if (now - motionStart < config.motionTimeout) {
            int constrained_intensity = constrain(motion_intensity, 1, 5);
            float intensity_scale = (constrained_intensity - 1) * (1.0f - 0.5f) / 4.0f + 0.5f;
            float adaptive_max = config.ledMaxPWM * intensity_scale;

            float bat_scale = (sys.batV - config.batMinV) / (config.batMaxV - config.batMinV);
            float bat_limit = config.ledDimPWM + bat_scale * (adaptive_max - config.ledDimPWM);

            sys.ledPWM = (int)constrain(bat_limit, config.ledDimPWM, config.ledMaxPWM);
        } else {
            sys.isMotion = false;
            motion_intensity = 0;
            sys.ledPWM = config.ledDimPWM;
        }
    } else {
        sys.ledPWM = config.ledDimPWM;
        motion_intensity = 0;
    }
}

void updateLightFade() {
    static unsigned long last_fade = 0;
    unsigned long now = millis();
    if (now - last_fade > LED_FADE_INTERVAL_MS) {
        if (current_led_val < sys.ledPWM) current_led_val += 1.0f;
        else if (current_led_val > sys.ledPWM) current_led_val -= 1.0f;

        analogWrite(PIN_LED_PWM, (int)current_led_val);
        last_fade = now;
    }
}

void sleepSystem() {
    Serial.println(F("SLEEP: Start"));
    Serial.flush();

    analogWrite(PIN_LED_PWM, 0);
    current_led_val = 0;
    OCR1A = 0;
    OCR1B = MPPT_PWM_MAX_RES;

    power_all_disable();
    configureSleepWDT(); // 8s interrupt sleep watchdog

    // Disable INT0 to prevent wake-loops on active-HIGH PIR sensors (which are normally LOW)
    EIMSK &= ~(1 << INT0);

    // Configure PCINT2 on Pin 2 (PD2 / PCINT18) for PIR HIGH motion detection
    PCIFR |= (1 << PCIF2);     // Clear pending interrupt flag
    PCICR |= (1 << PCIE2);     // Enable PCINT bank 2
    PCMSK2 |= (1 << PCINT18);  // Enable PCINT18 (Pin 2)

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sei();
    sleep_cpu();

    // --- WAKE UP ---
    sleep_disable();
    power_all_enable();
    enableActiveWDT(); // Re-enable active 4-second watchdog reset mode
    restoreHardware();

    Serial.begin(115200);
    Wire.begin();
    if (ina219.begin()) {
        ina219_present = true;
    } else {
        Serial.println(F("INA219: Absent after wake"));
        ina219_present = false;
        lastInaRetry = millis();
    }

    prevSolarV = -1.0f;
    lastMppt = millis();
    lastDarkTransition = 0;
    Serial.println(F("SLEEP: Woke up"));
}

void enableActiveWDT() {
    cli();
    wdt_reset();
    wdt_enable(WDTO_4S); // Standard 4-second System Reset Mode
    sei();
}

void configureSleepWDT() {
    cli();
    wdt_reset();
    MCUSR &= ~(1 << WDRF);
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    // 8-second Interrupt Mode (No System Reset)
    WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);
    sei();
}

void restoreHardware() {
    pinMode(PIN_MPPT_PWM, OUTPUT);
    digitalWrite(PIN_MPPT_PWM, LOW);
    pinMode(PIN_MPPT_SYNC, OUTPUT);
    digitalWrite(PIN_MPPT_SYNC, LOW);
    pinMode(PIN_LED_PWM, OUTPUT);
    digitalWrite(PIN_LED_PWM, LOW);
    pinMode(PIN_PIR, INPUT_PULLUP);

    // Disable INT0
    EIMSK &= ~(1 << INT0);

    // Enable PCINT2 for Pin 2 (PD2 / PCINT18)
    PCIFR |= (1 << PCIF2);     // Clear pending interrupt flag
    PCICR |= (1 << PCIE2);     // Enable PCINT bank 2
    PCMSK2 |= (1 << PCINT18);  // Enable PCINT18 (Pin 2)

    // --- High Frequency 10-bit PWM for MPPT (Timer1) ---
    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(COM1B0) | _BV(WGM11);
    TCCR1B = _BV(WGM13) | _BV(CS10);
    ICR1 = 1023;
    OCR1A = sys.mpptPWM;
    if (sys.mpptPWM > 50) {
        OCR1B = constrain(sys.mpptPWM + SYNC_FET_DEADTIME, 0, MPPT_PWM_MAX_RES);
    } else {
        OCR1B = MPPT_PWM_MAX_RES;
    }

    // --- Restore Timer2 (Pins 3, 11) ---
    TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS22);
}
