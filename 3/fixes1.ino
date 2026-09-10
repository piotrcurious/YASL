/*
 * YASL (Yet Another Solar Lamp) - Consolidated Version
 * Refactored for Safety, Non-Blocking execution, and I2C fault tolerance.
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
const uint8_t PIN_SOLAR_ADC = A0;
const uint8_t PIN_BAT_ADC   = A1;
const uint8_t PIN_LED_PWM   = 3;
const uint8_t PIN_PIR       = 2;
const uint8_t PIN_MPPT_PWM  = 9;
const uint8_t PIN_MPPT_SYNC = 10;

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
#define LOOP_TICK_RATE_MS       50       // Used for non-blocking state machine
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

// Fixed alignment with predictable types
struct Config {
    uint32_t magic;
    float    batMaxV;
    float    batMinV;
    float    batFloatV;
    uint32_t motionTimeout;
    uint16_t ledMaxPWM;
    uint16_t ledDimPWM;
};

const uint32_t MAGIC_TOKEN = 0x5941534D; // Updated to force layout refresh

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
unsigned long lastTick = 0; // Main non-blocking loop timer

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
void processCommand(const char* line);
void handleSerial();
void sleepSystem();
void enableActiveWDT();
void configureSleepWDT();
void restoreHardware();
float getSmoothedADC(uint8_t pin);
float readVcc();

// --- ISRs ---
ISR(INT0_vect) {
    wakePIR = true;
    if ((EICRA & 0b00000011) == 0) {
        EIMSK &= ~(1 << INT0); // Prevent wake-loops in LOW mode
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
    enableActiveWDT(); // Turn on 4-second safety watchdog
    
    while (!Serial && millis() < 2000) wdt_reset();
    Serial.println(F("YASL CONSOLIDATED v2.5 INIT (Non-Blocking)"));

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
    wdt_reset(); // Feed the watchdog

    // Process serial data instantly (prevents 64-byte buffer overflow)
    handleSerial();

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

    // Non-blocking tick for sensors and control (50ms)
    if (now - lastTick >= LOOP_TICK_RATE_MS) {
        lastTick = now;

        readSensors();

        // Dawn/Dusk Hysteresis logic
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

        if (last_dark_state && !sys.isDark) {
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

    // Execute LED smooth fade (Independent of 50ms tick)
    updateLightFade();

    if (now - lastLog > JSON_INTERVAL_MS) {
        Serial.print(F("{\"sV\":")); Serial.print(sys.solarV, 2);
        Serial.print(F(",\"sI_mA\":")); Serial.print(sys.solarI, 1);
        Serial.print(F(",\"sP_mW\":")); Serial.print(sys.solarP_mW, 0);
        Serial.print(F(",\"bV\":")); Serial.print(sys.batV, 2);
        Serial.print(F(",\"bP\":")); Serial.print(sys.batPcnt, 0);
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

// --- Light & MPPT logic updates ---
void updateMPPT() {
    unsigned long now = millis();
    float dynamic_start_v = max((float)SOLAR_START_V_MIN, sys.batV + 0.5f);

    if (sys.chargeMode == 'X' && sys.batV < config.batMaxV) sys.chargeMode = current_charge_stage;
    if ((sys.chargeMode == 'N' || sys.chargeMode == 'L') && !sys.isDark && sys.solarV >= dynamic_start_v) {
        sys.chargeMode = current_charge_stage;
    }

    if (sys.batV > config.batMaxV + BAT_OVERVOLT_MARGIN) {
        sys.mpptPWM = 0;
        sys.chargeMode = 'X';
        OCR1A = 0; OCR1B = MPPT_PWM_MAX_RES;
        return;
    }

    if (sys.solarV < dynamic_start_v || sys.isDark) {
        sys.mpptPWM = 0;
        sys.chargeMode = sys.isDark ? 'N' : 'L';
        OCR1A = 0; OCR1B = MPPT_PWM_MAX_RES;
        return;
    }

    if (current_charge_stage == 'B') {
        if (sys.mpptPWM == 0 && sys.solarV > dynamic_start_v) {
            float predictive_duty = (sys.batV + 0.5f) / sys.solarV;
            sys.mpptPWM = constrain((int)(predictive_duty * 1024.0f), 100, 800);
            prevSolarV = -1.0f;
            lastMppt = now;
            sys.chargeMode = current_charge_stage;
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
        if (sys.batV > config.batMaxV) { if (sys.mpptPWM > 0) sys.mpptPWM--; } 
        else if (sys.batV < config.batMaxV - CV_REG_MARGIN) { if (sys.mpptPWM < MPPT_PWM_MAX_RES) sys.mpptPWM++; }

        float duty = (float)OCR1A / 1023.0f;
        float batCurrentMA = (duty > 0.1f) ? (sys.solarI / duty) : 0;
        
        if ((batCurrentMA < TAIL_CURRENT_MA && batCurrentMA > 0) || (millis() - sys.absorptionStart > ABSORPTION_TIMEOUT_MS)) {
            current_charge_stage = 'F';
            prevSolarV = -1.0f;
        }
        if (sys.batV < config.batMinV + REBULK_ABS_DELTA) {
            current_charge_stage = 'B';
            prevSolarV = -1.0f;
        }
    }
    else if (current_charge_stage == 'F') {
        if (sys.batV > config.batFloatV) { if (sys.mpptPWM > 0) sys.mpptPWM--; } 
        else if (sys.batV < config.batFloatV - FLOAT_REG_MARGIN) { if (sys.mpptPWM < MPPT_PWM_MAX_RES) sys.mpptPWM++; }
        
        if (sys.batV < config.batFloatV - REBULK_FLOAT_DELTA) {
            current_charge_stage = 'B';
            prevSolarV = -1.0f;
        }
    }

    sys.chargeMode = current_charge_stage;

    // CRITICAL FIX: Clamp max PWM to leave room for the deadtime. Prevents shoot-through!
    int safe_max_pwm = MPPT_PWM_MAX_RES - SYNC_FET_DEADTIME;
    sys.mpptPWM = constrain(sys.mpptPWM, MPPT_PWM_MIN_RES, safe_max_pwm);
    
    OCR1A = sys.mpptPWM;
    if (sys.mpptPWM > 50) {
        OCR1B = sys.mpptPWM + SYNC_FET_DEADTIME; // Now guaranteed to never exceed MAX
    } else {
        OCR1B = MPPT_PWM_MAX_RES; 
    }
}

// Note: updateLight split for non-blocking main loop
void updateLight() {
    static bool lvd_active = false;
    unsigned long now = millis();

    if (!lvd_active && sys.batV < config.batMinV) {
        lvd_active = true;
        sys.ledPWM = PWM_LED_OFF;
        motion_intensity = 0;
    } else if (lvd_active && sys.batV > config.batMinV + 0.25f) {
        lvd_active = false;
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
            float intensity_scale = (constrained_intensity - 1) * (1.0f - 0.5f) / 4.0f + 0.5f; // Inline map
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
        if (current_led_val < sys.ledPWM) current_led_val += 1.0;
        else if (current_led_val > sys.ledPWM) current_led_val -= 1.0;
        
        analogWrite(PIN_LED_PWM, (int)current_led_val);
        last_fade = now;
    }
}

void readSensors() {
    unsigned long now = millis();
    
    // CRITICAL FIX: Only run blocking ADC Vcc check every 5 seconds
    static unsigned long lastVccCheck = 0;
    if (now - lastVccCheck > 5000) {
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
    
    // Inline map to prevent double evaluation in constrain macro
    float pcnt_calc = (sys.batV - config.batMinV) * 100.0f / (config.batMaxV - config.batMinV);
    sys.batPcnt = constrain(pcnt_calc, 0.0f, 100.0f);
}

// --- Supporting Logic & WDT ---
void sleepSystem() {
    Serial.flush();
    analogWrite(PIN_LED_PWM, 0);
    current_led_val = 0; 
    OCR1A = 0; 
    OCR1B = MPPT_PWM_MAX_RES;

    power_all_disable();
    configureSleepWDT(); 

    EICRA &= ~((1 << ISC01) | (1 << ISC00)); // LOW Level
    EIFR = (1 << INTF0);
    EIMSK |= (1 << INT0);

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sei();
    sleep_cpu(); 

    sleep_disable();
    power_all_enable();
    enableActiveWDT(); 
    restoreHardware(); 
    
    Serial.begin(115200);
    Wire.begin();
    if (ina219.begin()) ina219_present = true;
    else ina219_present = false;

    EICRA = (1 << ISC01) | (1 << ISC00); // Back to RISING
    EIFR = (1 << INTF0);
    EIMSK |= (1 << INT0);

    prevSolarV = -1.0f; 
    lastMppt = millis();
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

// ... remaining boilerplate functions (runSMCMPPT, processCommand, loadConfig, restoreHardware, etc.)
// remain exactly the same as your original, as their internal algorithms are mathematically sound!
