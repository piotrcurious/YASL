
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

// --- Configuration ---

// Pin Definitions
const uint8_t PIN_SOLAR_ADC = A0;   // Raw solar ADC (backup/redundant)
const uint8_t PIN_BAT_ADC   = A1;   // Main Battery Voltage Divider
const uint8_t PIN_LED_PWM   = 3;    // LED Mosfet (Timer2)
const uint8_t PIN_PIR       = 2;    // Motion Sensor (D2 is INT0)
const uint8_t PIN_MPPT_PWM  = 9;    // Solar MPPT Mosfet (Timer1)

// Reference & Ratios
const float REF_VOLTAGE = 5.0f;
const float BAT_DIVIDER_RATIO = 3.0f; // e.g., (10k+5k)/5k = 3.0
const float SOLAR_DIVIDER_RATIO = 4.0f;

// Battery (Li-Ion/Li-Po typical)
const float BAT_MAX_V       = 4.15f;  // Charge limit (Conservative)
const float BAT_MIN_V       = 3.00f;  // Low Voltage Disconnect
const float BAT_RECONNECT_V = 3.25f;  // Re-enable load voltage after LVD

// Solar & MPPT
const float SOLAR_START_V   = 5.0f;   // Min voltage to start switching
const float SOLAR_DARK_V    = 2.0f;   // Bus voltage below which it's night
const int   PWM_MPPT_MAX    = 240;    // Cap to avoid 100% duty cycle issues
const int   PWM_MPPT_MIN    = 0;
const long  MPPT_INTERVAL_MS = 100;

// Light Behavior
const int   PWM_LED_MAX     = 255;
const int   PWM_LED_DIM     = 15;
const int   PWM_LED_OFF     = 0;
const long  MOTION_ON_MS    = 15000;  // 15 seconds after motion
const long  JSON_INTERVAL_MS = 10000; // Log every 10s

// --- System Structures ---

struct SystemState {
    float solarV;       // INA219 Bus + Shunt
    float solarI;       // INA219 current in mA
    float solarP;       // mW
    float batV;         // ADC averaged
    float batPcnt;      // 0-100%
    float batMaxToday;
    float batMinToday;
    bool  isDark;
    bool  isMotion;
    int   ledPWM;
    int   mpptPWM;
    char  chargeMode;   // 'N'ight, 'L'ow solar, 'B'ulk (MPPT), 'C'V (Absorption), 'X' Overvoltage
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
volatile SystemState sys = {0,0,0,0,0,0,0,true,false,0,0,'N'};
Config config;
float current_led_val = 0;
bool ina219_present = false;
bool last_dark_state = true;
int motion_intensity = 0; // Cumulative motion tracker for adaptive brightness
bool manual_override = false;
unsigned long manual_override_start = 0;
const unsigned long OVERRIDE_TIMEOUT = 300000; // 5 minutes

// MPPT Trackers (Sliding Mode Control)
float prevSolarV = 0.0f;
float prevSolarP = 0.0f;
const float SMC_GAIN = 2.0f;
char current_charge_stage = 'B';

// Timers
unsigned long lastLog = 0;
unsigned long motionStart = 0;
unsigned long lastMppt = 0;

// Wake Flags
volatile bool wakePIR = false;
volatile bool wakeWDT = false;

// --- Prototypes ---
void loadConfig();
void saveConfig();
void readSensors();
void updateMPPT();
void updateLight();
void sleepSystem();
void configureWDT();
void disableWDT();
float getSmoothedADC(uint8_t pin);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

// --- INA219 ---
Adafruit_INA219 ina219;

// --- ISRs ---

ISR(INT0_vect) {
    wakePIR = true;
    // Disable interrupt in ISR to prevent bounce during wake
    EIMSK &= ~(1 << INT0);
}

ISR(WDT_vect) {
    wakeWDT = true;
}

// --- Arduino Core ---

void setup() {
    // 0. Load Configuration
    loadConfig();

    // Immediate safe states
    pinMode(PIN_MPPT_PWM, OUTPUT);
    digitalWrite(PIN_MPPT_PWM, LOW);
    pinMode(PIN_LED_PWM, OUTPUT);
    digitalWrite(PIN_LED_PWM, LOW);

    pinMode(PIN_PIR, INPUT_PULLUP);

    Serial.begin(115200);
    while (!Serial && millis() < 2000);
    Serial.println(F("YASL CONSOLIDATED v1.1 INIT"));

    if (!ina219.begin()) {
        Serial.println(F("ERR: INA219 FAILED - Using fallback ADC"));
        ina219_present = false;
    } else {
        Serial.println(F("INA219 OK"));
        ina219_present = true;
    }

    // --- High Frequency 10-bit PWM for MPPT (Timer1) ---
    // Pin 9 (OC1A)
    TCCR1A = _BV(COM1A1) | _BV(WGM11); // Fast PWM, non-inverting, mode 14
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // mode 14, no prescaler
    ICR1 = 1023; // 10-bit resolution -> 16MHz / 1024 = 15.6 kHz
    OCR1A = 0;

    // Set Timer2 (pins 3, 11) for ~976Hz PWM
    TCCR2B = (TCCR2B & 0b11111000) | 0x04;

    disableWDT();
    readSensors();
}

void checkAndInitiateSleep() {
    bool shouldSleep = false;

    if (sys.batPcnt < 40.0f && sys.isDark) {
        shouldSleep = true;
        Serial.println(F("Low battery, initiating timed sleep."));
    } else if (sys.isDark && sys.ledPWM <= config.ledDimPWM && !sys.isMotion &&
               (millis() - motionStart > 300000UL)) {
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

    // 3. Logic: Daytime vs Nighttime
    sys.isDark = (sys.solarV < SOLAR_DARK_V);

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
        analogWrite(PIN_MPPT_PWM, 0);
        sys.chargeMode = 'N';

        // Check Manual Override timeout
        if (manual_override && (now - manual_override_start > OVERRIDE_TIMEOUT)) {
            manual_override = false;
            sys.isMotion = false;
            Serial.println(F("Override: Timeout"));
        }

        updateLight();

        // Deep sleep if idle
        if (!sys.isMotion && sys.ledPWM <= config.ledDimPWM && !manual_override) {
            if (now - motionStart > config.motionTimeout + 5000) {
                sleepSystem();
            }
        }
    }
    else {
        // --- DAY ---
        // Light stays off or very dim
        sys.ledPWM = PWM_LED_OFF;
        analogWrite(PIN_LED_PWM, 0);
        sys.isMotion = false;

        updateMPPT();
    }

    // 4. Logging
    if (now - lastLog > JSON_INTERVAL_MS) {
        Serial.print(F("{\"sV\":")); Serial.print(sys.solarV, 2);
        Serial.print(F(",\"sI\":")); Serial.print(sys.solarI, 1);
        Serial.print(F(",\"sP\":")); Serial.print(sys.solarP, 0);
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

    // --- Command Processing ---
    if (Serial.available() > 0) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) return;

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
        else if (cmd == 's') { // Set Param (e.g. sM 4.2)
            char param = line[1];
            float val = line.substring(2).toFloat();
            if (param == 'M') { config.batMaxV = val; }
            else if (param == 'm') { config.batMinV = val; }
            else if (param == 'F') { config.batFloatV = val; }
            else if (param == 'T') { config.motionTimeout = (long)val; }
            else if (param == 'X') { config.ledMaxPWM = (int)val; }
            else if (param == 'D') { config.ledDimPWM = (int)val; }
            saveConfig();
            Serial.print(F("Param ")); Serial.print(param); Serial.println(F(" Saved"));
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
        else if (cmd == 'S') { // Manual Stage Force (for testing)
            char stage = line[1];
            if (stage == 'B' || stage == 'A' || stage == 'F') {
                current_charge_stage = stage;
                Serial.print(F("Stage forced to: ")); Serial.println(stage);
            } else {
                Serial.println(F("Invalid stage. Use B, A, or F."));
            }
        }
    }

    checkAndInitiateSleep();
    delay(50); // Loop stability
}

// --- Support Functions ---

void loadConfig() {
    EEPROM.get(0, config);
    if (config.magic != MAGIC_TOKEN) {
        // Factory Defaults
        config.magic = MAGIC_TOKEN;
        config.batMaxV = BAT_MAX_V;
        config.batMinV = BAT_MIN_V;
        config.batFloatV = 3.45f; // Standard LiFePO4/Li-ion float
        config.ledMaxPWM = PWM_LED_MAX;
        config.ledDimPWM = PWM_LED_DIM;
        config.motionTimeout = MOTION_ON_MS;
        saveConfig();
        Serial.println(F("EEPROM: Using Defaults"));
    } else {
        Serial.println(F("EEPROM: Loaded Config"));
    }
}

void saveConfig() {
    EEPROM.put(0, config);
}

void readSensors() {
    if (ina219_present) {
        // INA219 Primary Sensor
        float bus = ina219.getBusVoltage_V();
        float shunt = ina219.getShuntVoltage_mV();
        sys.solarV = bus + (shunt / 1000.0f);
        sys.solarI = ina219.getCurrent_mA();
        sys.solarP = bus * sys.solarI;
    } else {
        // Fallback to Raw ADC
        float rawSolar = getSmoothedADC(PIN_SOLAR_ADC);
        sys.solarV = rawSolar * (REF_VOLTAGE / 1023.0f) * SOLAR_DIVIDER_RATIO;
        sys.solarI = 0.0f; // Cannot measure current without shunt/INA
        sys.solarP = 0.0f;
    }

    // Bat ADC (Averaged)
    float raw = getSmoothedADC(PIN_BAT_ADC);
    sys.batV = raw * (REF_VOLTAGE / 1023.0f) * BAT_DIVIDER_RATIO;
    sys.batPcnt = constrain(mapFloat(sys.batV, config.batMinV, config.batMaxV, 0, 100), 0, 100);

    // Update Daily Stats
    if (sys.batV > sys.batMaxToday) sys.batMaxToday = sys.batV;
    if (sys.batV < sys.batMinToday || sys.batMinToday == 0) sys.batMinToday = sys.batV;
}

float getSmoothedADC(uint8_t pin) {
    long sum = 0;
    for(int i=0; i<8; i++) {
        sum += analogRead(pin);
    }
    return (float)sum / 8.0f;
}

void updateMPPT() {
    // Safety & Night check
    if (sys.solarV < SOLAR_START_V || sys.isDark) {
        sys.mpptPWM = 0;
        sys.chargeMode = 'N';
        current_charge_stage = 'B';
        analogWrite(PIN_MPPT_PWM, 0);
        return;
    }

    // Overvoltage Protection
    if (sys.batV > config.batMaxV + 0.2f) {
        sys.mpptPWM = 0;
        sys.chargeMode = 'X';
        analogWrite(PIN_MPPT_PWM, 0);
        return;
    }

    // 3-Stage Logic
    if (current_charge_stage == 'B') { // Bulk (MPPT)
        sys.chargeMode = 'B';
        if (sys.batV >= config.batMaxV) {
            current_charge_stage = 'A'; // To Absorption
        } else {
            runSMCMPPT();
        }
    }
    else if (current_charge_stage == 'A') { // Absorption (CV)
        sys.chargeMode = 'A';
        // Maintain config.batMaxV
        if (sys.batV > config.batMaxV) {
            if (sys.mpptPWM > 0) sys.mpptPWM--;
        } else if (sys.batV < config.batMaxV - 0.02f) {
            if (sys.mpptPWM < 1000) sys.mpptPWM++;
        }

        // Transition to Float after current drops (Tail Current) or timeout
        static unsigned long absorption_start = 0;
        if (absorption_start == 0) absorption_start = millis();

        bool tail_current_reached = (ina219_present && sys.solarI < 50.0f && sys.solarI > 0);
        bool timeout_reached = (millis() - absorption_start > 7200000UL); // 2 hours max

        if (tail_current_reached || timeout_reached) {
            current_charge_stage = 'F';
            absorption_start = 0;
        }
        if (sys.batV < config.batMinV + 0.2f) current_charge_stage = 'B'; // Deep discharge
    }
    else if (current_charge_stage == 'F') { // Float
        sys.chargeMode = 'F';
        if (sys.batV > config.batFloatV) {
            if (sys.mpptPWM > 0) sys.mpptPWM--;
        } else if (sys.batV < config.batFloatV - 0.05f) {
            if (sys.mpptPWM < 1000) sys.mpptPWM++;
        }
        if (sys.batV < config.batFloatV - 0.3f) current_charge_stage = 'B'; // Re-bulk
    }

    // MPPT PWM is now 10-bit (0-1023)
    sys.mpptPWM = constrain(sys.mpptPWM, 0, 1000); // Leave some margin
    OCR1A = sys.mpptPWM;
}

void runSMCMPPT() {
    if (!ina219_present) {
        sys.mpptPWM = PWM_MPPT_MAX; // Full on if no sensors
        sys.chargeMode = 'S';
        return;
    }

    unsigned long now = millis();
    if (now - lastMppt > MPPT_INTERVAL_MS) {
        float dv = sys.solarV - prevSolarV;
        float dp = sys.solarP - prevSolarP;

        if (abs(dv) > 0.01f) {
            float S = dp / dv;
            // Scale gain for 10-bit resolution (SMC_GAIN * 4)
            if (S > 0.01f) sys.mpptPWM -= (SMC_GAIN * 4);
            else if (S < -0.01f) sys.mpptPWM += (SMC_GAIN * 4);

            prevSolarV = sys.solarV;
            prevSolarP = sys.solarP;
        } else {
            sys.mpptPWM += SMC_GAIN;
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
    } else if (lvd_active && sys.batV > BAT_RECONNECT_V) {
        lvd_active = false;
        Serial.println(F("LVD: Recovered"));
    }

    if (lvd_active) {
        sys.ledPWM = PWM_LED_OFF;
        motion_intensity = 0;
    }
    else if (sys.isMotion) {
        if (now - motionStart < config.motionTimeout) {
            // Adaptive Brightness
            int adaptive_max = map(constrain(motion_intensity, 1, 5), 1, 5, (config.ledMaxPWM/2), config.ledMaxPWM);
            int bat_limit = map(constrain(sys.batV * 100, config.batMinV * 100, config.batMaxV * 100),
                                config.batMinV * 100, config.batMaxV * 100, config.ledDimPWM, adaptive_max);
            sys.ledPWM = constrain(bat_limit, config.ledDimPWM, config.ledMaxPWM);

            if (digitalRead(PIN_PIR) == HIGH && (now - last_motion_check > 1000)) {
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
    if (now - last_fade > 10) { // Update every 10ms
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
    OCR1A = 0; // MPPT 10-bit PWM off

    // Peripheral Shutdown
    ADCSRA &= ~(1 << ADEN); // ADC off

    // WDT for periodic health check (8s)
    configureWDT();

    // INT0 for PIR wake
    // Configure INT0 to wake on RISING edge (typical for PIR sensors)
    EICRA = (1 << ISC01) | (1 << ISC00); // ISC01=1, ISC00=1 -> RISING edge
    EIFR = (1 << INTF0);  // Clear flags
    EIMSK |= (1 << INT0); // Enable INT0

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();

    // Enable Global Interrupts (just in case they were off)
    sei();

    sleep_cpu(); // HALT

    // --- WAKE UP ---
    sleep_disable();
    disableWDT();
    ADCSRA |= (1 << ADEN); // ADC on

    Serial.println(F("SLEEP: Woke up"));
}

void configureWDT() {
    cli();
    wdt_reset();
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    // Interrupt mode, 8.0s timeout
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

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
