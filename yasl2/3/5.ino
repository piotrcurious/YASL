// Arduino Solar Motion Lamp - Enhanced Version
// Features: Safety-First MPPT (CC/CV), Averaged Readings, Robust Sleep
// Hardware: ATMega328P, INA219, PIR Sensor, MOSFET for LED & Solar

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <Arduino.h>

// --- Configuration ---
// Pinout
const uint8_t PIN_SOLAR_ADC = A0;   // Optional redundant voltage check
const uint8_t PIN_BAT_ADC   = A1;   // Main Battery Voltage Divider
const uint8_t PIN_LED_PWM   = 3;    // LED Mosfet
const uint8_t PIN_PIR       = 2;    // Motion Sensor (INT0)
const uint8_t PIN_MPPT_PWM  = 9;    // Solar Buck Converter Mosfet

// Hardware Constants
const float REF_VOLTAGE = 5.0f;
const float BAT_DIVIDER_RATIO = 3.0f; // Check your resistors!
const float INA_SHUNT_OHMS = 0.1f;    // Default Adafruit shunt

// Battery Safety (Li-Ion/Li-Po)
const float BAT_MAX_V     = 4.15f;    // Target CV voltage (Conservative 4.15V for longevity)
const float BAT_MIN_V     = 3.00f;    // LVD (Low Voltage Disconnect)
const float BAT_RECONNECT = 3.20f;    // Re-enable load voltage

// MPPT Settings
const float MIN_SOLAR_V_START = 5.0f; // Min voltage to start switching
const int   PWM_MPPT_MAX = 240;       // Cap to prevent 100% duty cycle issues
const int   PWM_MPPT_MIN = 0;
const long  MPPT_INTERVAL_MS = 100;   // Update freq (faster than original)

// Light Settings
const int   PWM_LED_MAX = 255;
const int   PWM_LED_DIM = 10;
const long  LIGHT_DURATION_MS = 15000;

// --- Objects & Globals ---
Adafruit_INA219 ina219(0x40);

// Data Struct for system state
struct SystemState {
    float solarV;       // From INA219 (Bus)
    float solarI;       // From INA219
    float solarP;       // Calculated
    float batV;         // From ADC
    float batPcnt;      // Calculated
    bool  isDark;
    bool  motionDetected;
    int   ledPWM;
    int   mpptPWM;
    bool  charging;
    char  chargeMode;   // 'O'ff, 'B'ulk (MPPT), 'C'V (Absorption)
};

volatile SystemState sysState = {0,0,0,0,0,true,false,0,0,false,'O'};

// MPPT Tracking Variables
float prevSolarPower = 0.0f;
bool  mpptDirectionUp = true; 

// Timing
unsigned long lastLoopTime = 0;
unsigned long motionStartTime = 0;
unsigned long lastLogTime = 0;

// Interrupt Flags
volatile bool wakeFromPIR = false;
volatile bool wakeFromWDT = false;

// --- Forward Declarations ---
void readSensors();
void updateMPPT();
void updateLight();
void sleepSystem();
void configureWDT();
float getSmoothedADC(uint8_t pin);

// --- ISRs ---
ISR(INT0_vect) {
    wakeFromPIR = true;
    // Don't detach interrupts here, handle in loop
}

ISR(WDT_vect) {
    wakeFromWDT = true;
}

void setup() {
    // 1. Safety First: Ensure PWMs are low immediately
    pinMode(PIN_MPPT_PWM, OUTPUT);
    digitalWrite(PIN_MPPT_PWM, LOW);
    pinMode(PIN_LED_PWM, OUTPUT);
    digitalWrite(PIN_LED_PWM, LOW);
    
    // 2. Pins
    pinMode(PIN_PIR, INPUT_PULLUP);
    
    // 3. Comms
    Serial.begin(115200);
    Serial.println(F("BOOT: Solar Controller Init"));

    // 4. Sensors
    if (!ina219.begin()) {
        Serial.println(F("ERR: INA219 not found!"));
        while(1);
    }
    // Calibrate INA219 if needed, defaults are usually fine for 32V/2A
    
    // 5. Initial Sensor Read
    readSensors();
}

void loop() {
    unsigned long currentMillis = millis();

    // 1. Handle Wake Flags
    if (wakeFromPIR) {
        wakeFromPIR = false;
        sysState.motionDetected = true;
        motionStartTime = currentMillis;
        Serial.println(F("EVT: Motion Wake"));
    }

    // 2. Read Sensors (Averaged)
    readSensors();

    // 3. State Determination
    sysState.isDark = (sysState.solarV < 3.0f); // Solar panel drops voltage at night

    // 4. Sub-systems
    if (sysState.isDark) {
        // NIGHT MODE
        // Disable charging
        sysState.mpptPWM = 0;
        analogWrite(PIN_MPPT_PWM, 0);
        sysState.chargeMode = 'N'; // Night
        
        // Handle Light
        updateLight();
        
        // Check for Deep Sleep opportunity
        // If light is off/dim and no recent motion, sleep
        if (!sysState.motionDetected && sysState.ledPWM <= PWM_LED_DIM) {
             // Only sleep if enough time passed since last motion
             if (currentMillis - motionStartTime > LIGHT_DURATION_MS) {
                 sleepSystem();
             }
        }
    } 
    else {
        // DAY MODE
        // Force Light Off
        analogWrite(PIN_LED_PWM, 0);
        sysState.ledPWM = 0;
        sysState.motionDetected = false;
        
        // Handle Charging
        updateMPPT();
    }

    // 5. Logging (Every 5 seconds)
    if (currentMillis - lastLogTime > 5000) {
        Serial.print(F("PV:")); Serial.print(sysState.solarV); Serial.print(F("V "));
        Serial.print(sysState.solarI); Serial.print(F("mA | "));
        Serial.print(F("BAT:")); Serial.print(sysState.batV); Serial.print(F("V | "));
        Serial.print(F("PWM:")); Serial.print(sysState.mpptPWM); 
        Serial.print(F(" | Mode:")); Serial.println(sysState.chargeMode);
        lastLogTime = currentMillis;
    }
    
    delay(50); // Small loop stabilization
}

// --- Sensor Logic ---
void readSensors() {
    // Read INA219
    float busV = ina219.getBusVoltage_V();
    float shuntV = ina219.getShuntVoltage_mV();
    sysState.solarV = busV + (shuntV / 1000.0f); // Load voltage
    sysState.solarI = ina219.getCurrent_mA();
    sysState.solarP = busV * sysState.solarI; // Power in mW
    
    // Read Battery ADC (Smoothed)
    float rawBat = getSmoothedADC(PIN_BAT_ADC);
    sysState.batV = rawBat * (REF_VOLTAGE / 1023.0f) * BAT_DIVIDER_RATIO;
    
    // Simple Percentage (Linear approx for Li-Ion 3.0-4.2)
    sysState.batPcnt = constrain(map(sysState.batV * 100, 300, 420, 0, 100), 0, 100);
}

float getSmoothedADC(uint8_t pin) {
    long sum = 0;
    for(int i=0; i<10; i++) {
        sum += analogRead(pin);
        delay(1);
    }
    return (float)sum / 10.0f;
}

// --- Improved MPPT Algorithm ---
void updateMPPT() {
    // Safety check
    if (sysState.batV >= BAT_MAX_V + 0.05) {
        // EMERGENCY OVERVOLTAGE
        sysState.mpptPWM = 0;
        analogWrite(PIN_MPPT_PWM, 0);
        sysState.chargeMode = 'X'; // Overvoltage Protection
        return;
    }

    // Check if sufficient input
    if (sysState.solarV < MIN_SOLAR_V_START) {
        sysState.mpptPWM = 0;
        sysState.chargeMode = 'L'; // Low Input
    }
    else if (sysState.batV >= BAT_MAX_V) {
        // CV MODE (Constant Voltage)
        // Battery is hitting limit. Reduce PWM to hold voltage steady.
        // We act like a negative feedback loop.
        sysState.chargeMode = 'C';
        if (sysState.mpptPWM > 0) sysState.mpptPWM--; 
    }
    else {
        // BULK MPPT MODE (P&O)
        sysState.chargeMode = 'B';
        
        // Perturb & Observe Logic
        // We only update every MPPT_INTERVAL_MS to let inductor settle
        static unsigned long lastMpptTime = 0;
        if (millis() - lastMpptTime > MPPT_INTERVAL_MS) {
            
            float powerDelta = sysState.solarP - prevSolarPower;
            
            if (powerDelta > 0) {
                // Power increased, keep going in same direction
                if (mpptDirectionUp) sysState.mpptPWM++; else sysState.mpptPWM--;
            } else {
                // Power decreased, reverse direction
                mpptDirectionUp = !mpptDirectionUp;
                if (mpptDirectionUp) sysState.mpptPWM++; else sysState.mpptPWM--;
            }
            
            prevSolarPower = sysState.solarP;
            lastMpptTime = millis();
        }
    }

    // Apply Constraints
    sysState.mpptPWM = constrain(sysState.mpptPWM, PWM_MPPT_MIN, PWM_MPPT_MAX);
    analogWrite(PIN_MPPT_PWM, sysState.mpptPWM);
}

// --- Light Logic ---
void updateLight() {
    unsigned long now = millis();
    bool batCritical = (sysState.batV < BAT_MIN_V);
    
    if (batCritical) {
        sysState.ledPWM = 0;
    } 
    else if (sysState.motionDetected) {
        // Check timeout
        if (now - motionStartTime < LIGHT_DURATION_MS) {
            // Motion Active
            sysState.ledPWM = PWM_LED_MAX;
        } else {
            // Timeout expired
            sysState.motionDetected = false;
            sysState.ledPWM = PWM_LED_DIM;
        }
    } else {
        sysState.ledPWM = PWM_LED_DIM;
    }
    
    // Write value
    analogWrite(PIN_LED_PWM, sysState.ledPWM);
}

// --- Sleep Management ---
void configureWDT() {
    cli();
    wdt_reset();
    // Enter Watchdog Config mode
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    // Set Interrupt Enable, no Reset, 4 Seconds timeout
    // WDP3=1, WDP0=0 (1000) = 4s. (Adjust as needed)
    WDTCSR = (1<<WDIE) | (1<<WDP3); 
    sei();
}

void sleepSystem() {
    // 1. Prepare Hardware
    Serial.println(F("SLEEP: Entering Deep Sleep..."));
    Serial.flush(); // Ensure text is sent
    
    analogWrite(PIN_LED_PWM, 0); // Ensure LED is off during sleep to save massive power
    analogWrite(PIN_MPPT_PWM, 0);
    
    // 2. Setup Interrupts
    // WDT is used to wake up periodically to check Battery/Solar status
    configureWDT(); 
    
    // Ensure INT0 (PIR) is ready
    EIFR = (1 << INTF0); // Clear flags
    EIMSK |= (1 << INT0); // Enable INT0
    
    // 3. Sleep Configuration
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    
    // 4. Sleep
    // Brown-out disable can save power, but requires specific fuse settings or specific macro
    sleep_cpu(); // HALT HERE
    
    // -- WAKE UP HAPPENS HERE --
    
    sleep_disable();
    wdt_disable(); // Turn off WDT immediately
    
    Serial.println(F("WAKE: System Active"));
}
