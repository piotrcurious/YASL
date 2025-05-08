// Arduino code for solar powered motion sensor lamp
// With INA219, basic MPPT, and ADVANCED SLEEP MODES
// Current Date: Thursday, May 8, 2025 (for context, not functional)

#include <Wire.h>              // For I2C communication
#include <Adafruit_INA219.h>   // For INA219 Current Sensor
#include <avr/sleep.h>         // For sleep modes
#include <avr/wdt.h>           // For Watchdog Timer
#include <avr/power.h>         // For power reduction register
#include <avr/interrupt.h>     // For ISR macro and interrupt functions

// --- Pin Definitions ---
const int solarPin = A0;       // Analog pin for a separate solar panel voltage sensor
const int batteryPin = A1;     // Analog pin for battery voltage sensor
const int ledPwmPin = 3;       // PWM pin for LED light (Timer2)
const int motionPin = 2;       // Digital pin for PIR motion sensor (MUST BE D2 for INT0)
const int mpptPwmPin = 9;      // PWM pin for MPPT control (Timer1)

// --- System Parameters (Mostly unchanged, review if needed) ---
const float ADC_REFERENCE_VOLTAGE = 5.0;
const float ADC_MAX_VALUE = 1023.0;
const float SOLAR_VOLTAGE_DIVIDER_RATIO = 3.0 + 1.0;
const float BATTERY_VOLTAGE_DIVIDER_RATIO = 2.0 + 1.0;
const float SOLAR_THRESHOLD_VOLTAGE_AT_PIN = 0.8; // Actual solar voltage from INA219 used now. This is for raw ADC pin.
const float SOLAR_DARK_THRESHOLD_ACTUAL_VOLTAGE = 2.0; // Actual solar panel voltage (from INA219) to consider it "dark"
const float BATTERY_MAX_VOLTAGE = 4.2;
const float BATTERY_MIN_VOLTAGE = 3.0;
const float BATTERY_PROTECTION_VOLTAGE = 3.2;
const float BATTERY_CHARGE_TARGET_VOLTAGE = 4.1;
const float MIN_SOLAR_VOLTAGE_FOR_MPPT_ACTUAL = 6.0;
const int PWM_MAX = 255;
const int PWM_MIN = 0;
const int PWM_DIM = 20; // Slightly dimmer for power saving
const int PWM_STEP = 10; // Faster dimming/brightening steps
const long JSON_EMIT_INTERVAL_MS = 30000;
const long MOTION_DEBOUNCE_MS = 200; // PIR sensors often have internal debounce too
const long LIGHT_ON_DURATION_AFTER_MOTION_MS = 10000; // 10 seconds
const long MPPT_UPDATE_INTERVAL_MS = 1000;
const uint16_t DEEP_SLEEP_TARGET_DURATION_S = 60; // Target deep sleep duration in seconds

// --- INA219 Sensor ---
Adafruit_INA219 ina219(0x40); // Specify address if non-default

// --- Global Variables ---
// Voltages & Power
float solarVoltage_raw_adc;
float batteryVoltage_actual;
float batteryLevel_percent;
float solarBusVoltage_V;
float solarShuntVoltage_mV;
float solarCurrent_mA;
float solarPower_mW;

// Light Control
int currentLedPwmValue;
int targetLedPwmValue; // Not actively used for smooth ramp in this version, direct set.
bool motionDetectedFlag; // True if valid motion sequence started
bool isDarkFlag;

// Charging & MPPT
bool conditionsForChargingFlag;
int mpptPwmValue;
float lastMpptPower_mW;
int mpptPwmStep = 2;
bool mpptPerturbDirectionUp = true;

// Timers
unsigned long lastJsonEmitMillis;
unsigned long lastMotionDetectMillis; // For debouncing PIR
unsigned long lastLightActivityMillis; // For light ON duration
unsigned long lastMpptUpdateMillis;

// Sleep Management
volatile bool motionWakeupOccurred = false;
volatile bool wdtWakeupOccurred = false;
uint16_t wdtSleepCyclesTarget = 0; // How many WDT cycles to sleep for
uint16_t wdtSleepCyclesCount = 0;  // How many WDT cycles completed

// --- Forward Declarations ---
void enterSleepCycle();
void disableWDT();

// --- ISRs ---
// INT0 ISR for Motion Sensor on Pin D2
ISR(INT0_vect) {
  motionWakeupOccurred = true;
  EIMSK &= ~(1 << INT0); // Disable INT0 to prevent re-triggering until explicitly re-enabled
}

// WDT ISR for Timed Wakeup
ISR(WDT_vect) {
  wdtWakeupOccurred = true;
  // WDT interrupt is automatically disabled by hardware after it fires.
  // WDIE must be set again to enable subsequent WDT interrupts.
  // MCUSR &= ~(1<<WDRF); // Clear WDRF if it were a reset, not needed for WDIE
}

// --- Setup Function ---
void setup() {
  // It's good practice to disable WDT early if it might have been left on
  disableWDT();

  Serial.begin(115200);
  while (!Serial && millis() < 2000); // Wait for serial, but timeout
  Serial.println(F("\nSystem Initializing..."));

  pinMode(solarPin, INPUT);
  pinMode(batteryPin, INPUT);
  pinMode(ledPwmPin, OUTPUT);
  pinMode(mpptPwmPin, OUTPUT);
  pinMode(motionPin, INPUT_PULLUP); // Use INPUT_PULLUP if PIR output is open-collector or if you need a defined state.
                                     // Most PIRs are active HIGH, so INPUT is fine. Revert to INPUT if PULLUP causes issues.

  // Initialize INA219
  if (!ina219.begin()) {
    Serial.println(F("Failed to find INA219 chip. Halting."));
    while (1) { delay(100); }
  }
  Serial.println(F("INA219 sensor initialized."));

  // LED PWM Frequency (Pin 3, Timer2) - aiming for ~976Hz
  TCCR2B = (TCCR2B & 0b11111000) | 0x04; // Prescaler 64

  analogWrite(ledPwmPin, PWM_MIN);
  currentLedPwmValue = PWM_MIN;

  mpptPwmValue = PWM_MIN; // Start with MPPT off
  analogWrite(mpptPwmPin, mpptPwmValue);
  lastMpptPower_mW = 0;

  // Initialize timers
  unsigned long currentMillis = millis();
  lastJsonEmitMillis = currentMillis;
  lastMotionDetectMillis = currentMillis;
  lastLightActivityMillis = currentMillis;
  lastMpptUpdateMillis = currentMillis;

  Serial.println(F("Setup Complete."));
}

// --- Main Loop ---
void loop() {
  // --- Handle Wakeup Events First ---
  if (motionWakeupOccurred) {
    // Serial.println(F("Woke up by MOTION"));
    motionWakeupOccurred = false; // Reset flag
    wdtSleepCyclesTarget = 0;     // Cancel any timed sleep
    wdtSleepCyclesCount = 0;
    motionDetectedFlag = true;    // Process this as a new motion event
    lastLightActivityMillis = millis(); // Reset light timer
    // Note: INT0 is currently disabled from ISR, will be re-enabled when going back to sleep or after light off
  }

  if (wdtWakeupOccurred) {
    // Serial.print(F("Woke up by WDT. Cycle ")); Serial.println(wdtSleepCyclesCount + 1);
    wdtWakeupOccurred = false; // Reset flag
    if (wdtSleepCyclesTarget > 0) {
      wdtSleepCyclesCount++;
    }
  }

  // --- Check if we need to go (back) to sleep ---
  if (wdtSleepCyclesTarget > 0 && wdtSleepCyclesCount < wdtSleepCyclesTarget && !motionDetectedFlag) {
    enterSleepCycle(); // This function will put MCU to sleep and execution halts here
    // After waking from sleep_cpu(), loop() restarts.
    // Flags (motionWakeupOccurred, wdtWakeupOccurred) will be checked at the top of the loop.
    return; // Essential to ensure loop restarts and checks flags immediately after sleep_cpu returns
  } else if (wdtSleepCyclesCount >= wdtSleepCyclesTarget) {
    wdtSleepCyclesTarget = 0; // Timed sleep finished
    wdtSleepCyclesCount = 0;
    // Serial.println(F("WDT timed sleep finished."));
  }

  // --- Normal Operations (skipped if sleeping) ---
  readAnalogSensors();
  readIna219Data();
  determineSystemState();

  if (isDarkFlag) {
    analogWrite(mpptPwmPin, PWM_MIN); // Ensure MPPT is off at night
    mpptPwmValue = PWM_MIN; // Update state variable
    handleMotionAndLight();   // Combines motion detection (if needed) and light control
  } else { // Daytime
    if (currentLedPwmValue > PWM_MIN) turnOffLedCompletely();
    if (conditionsForChargingFlag) {
      runMpptAlgorithm();
    } else {
      if (mpptPwmValue > PWM_MIN) { // Turn off MPPT if not charging
          analogWrite(mpptPwmPin, PWM_MIN);
          mpptPwmValue = PWM_MIN;
      }
    }
  }

  emitJsonData();
  checkAndInitiateSleep(); // Decide if system should go to sleep
} // End of loop()


// --- Sensor Reading ---
void readAnalogSensors() { /* ... unchanged ... */
  solarVoltage_raw_adc = analogRead(solarPin) * (ADC_REFERENCE_VOLTAGE / ADC_MAX_VALUE);
  float batteryPinVoltage = analogRead(batteryPin) * (ADC_REFERENCE_VOLTAGE / ADC_MAX_VALUE);
  batteryVoltage_actual = batteryPinVoltage * BATTERY_VOLTAGE_DIVIDER_RATIO;
  batteryLevel_percent = mapFloat(batteryVoltage_actual, BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE, 0.0, 100.0);
  batteryLevel_percent = constrain(batteryLevel_percent, 0.0, 100.0);
}
void readIna219Data() { /* ... unchanged ... */
  solarShuntVoltage_mV = ina219.getShuntVoltage_mV();
  solarBusVoltage_V = ina219.getBusVoltage_V();
  solarCurrent_mA = ina219.getCurrent_mA();
  solarPower_mW = ina219.getPower_mW();
  if (solarBusVoltage_V < 0.5 && solarCurrent_mA > 100) {
      solarCurrent_mA = 0;
      solarPower_mW = 0;
  }
}

// --- System State Determination ---
void determineSystemState() {
  if (solarBusVoltage_V < SOLAR_DARK_THRESHOLD_ACTUAL_VOLTAGE) {
    isDarkFlag = true;
  } else {
    isDarkFlag = false;
  }

  if (!isDarkFlag &&
      batteryVoltage_actual < BATTERY_CHARGE_TARGET_VOLTAGE &&
      solarBusVoltage_V > MIN_SOLAR_VOLTAGE_FOR_MPPT_ACTUAL &&
      solarPower_mW >= 0) { // Ensure power is not negative (though INA219 usually gives absolute current)
    conditionsForChargingFlag = true;
  } else {
    conditionsForChargingFlag = false;
  }
}

// --- MPPT Algorithm ---
void runMpptAlgorithm() { /* ... largely unchanged ... */
  if (millis() - lastMpptUpdateMillis > MPPT_UPDATE_INTERVAL_MS) {
    if (solarPower_mW > lastMpptPower_mW + 10) { // Add a small threshold to avoid noise
      // Power increased, keep going in the same direction
    } else if (solarPower_mW < lastMpptPower_mW - 10) {
      // Power decreased, reverse direction
      mpptPerturbDirectionUp = !mpptPerturbDirectionUp;
    } // else if power is similar, keep direction

    if (mpptPerturbDirectionUp) mpptPwmValue += mpptPwmStep;
    else mpptPwmValue -= mpptPwmStep;

    mpptPwmValue = constrain(mpptPwmValue, PWM_MIN, PWM_MAX);
    analogWrite(mpptPwmPin, mpptPwmValue);
    lastMpptPower_mW = solarPower_mW;
    lastMpptUpdateMillis = millis();
  }
}

// --- Motion and Light Control ---
void handleMotionAndLight() {
  unsigned long currentMillis = millis();

  // Check for new motion if not already processing one
  // Re-enable motion interrupt here if we are ready to detect new motion
  if (!motionDetectedFlag && wdtSleepCyclesTarget == 0) { // Only check for new hardware motion if not in timed sleep and no active motion
    if ((EIMSK & (1 << INT0)) == 0) { // If INT0 is disabled, re-enable it
        EIFR = (1 << INTF0); // Clear any pending INT0 flag
        EIMSK |= (1 << INT0); // Enable INT0
        // Serial.println(F("INT0 Re-enabled for motion"));
    }
    // No software debounce here as INT0 ISR handles the initial trigger.
    // motionDetectedFlag is set by ISR via motionWakeupOccurred flag handling in loop()
  }


  if (batteryVoltage_actual < BATTERY_PROTECTION_VOLTAGE) {
    if (currentLedPwmValue > PWM_MIN) Serial.println(F("Battery critically low, LED off."));
    targetLedPwmValue = PWM_MIN;
    motionDetectedFlag = false; // Stop processing this motion event
  } else if (motionDetectedFlag) { // motionDetectedFlag is true if motionWakeupOccurred was processed
    // Light is on due to motion
    if (currentMillis - lastLightActivityMillis < LIGHT_ON_DURATION_AFTER_MOTION_MS) {
      targetLedPwmValue = map(constrain(batteryVoltage_actual, BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE),
                           BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE,
                           PWM_DIM, PWM_MAX);
    } else {
      // Light duration expired
      motionDetectedFlag = false; // Reset for next motion
      targetLedPwmValue = PWM_DIM; // Go to dim after motion timeout
      // Serial.println(F("Motion light duration expired."));
    }
  } else { // No active motion sequence
    targetLedPwmValue = PWM_DIM; // Default to dim light when dark
  }

  // Set LED PWM (no smooth ramp in this version for simplicity, direct set)
  if (currentLedPwmValue != targetLedPwmValue) {
      currentLedPwmValue = targetLedPwmValue;
      currentLedPwmValue = constrain(currentLedPwmValue, PWM_MIN, PWM_MAX);
      analogWrite(ledPwmPin, currentLedPwmValue);
  }
}

// --- Turn Off LED ---
void turnOffLedCompletely() {
  if (currentLedPwmValue > PWM_MIN) {
      currentLedPwmValue = PWM_MIN;
      targetLedPwmValue = PWM_MIN;
      analogWrite(ledPwmPin, currentLedPwmValue);
  }
  motionDetectedFlag = false;
}

// --- JSON Data Emission ---
void emitJsonData() { /* ... largely unchanged ... */
  if (millis() - lastJsonEmitMillis > JSON_EMIT_INTERVAL_MS) {
    Serial.print(F("{"));
    // ... (previous JSON fields)
    Serial.print(F("\"sV_bus\":")); Serial.print(solarBusVoltage_V, 2);
    Serial.print(F(",\"sI_mA\":")); Serial.print(solarCurrent_mA, 1);
    Serial.print(F(",\"sP_mW\":")); Serial.print(solarPower_mW, 0);
    Serial.print(F(",\"batV\":")); Serial.print(batteryVoltage_actual, 2);
    Serial.print(F(",\"batPcnt\":")); Serial.print(batteryLevel_percent, 0);
    Serial.print(F(",\"isDark\":")); Serial.print(isDarkFlag);
    Serial.print(F(",\"isChg\":")); Serial.print(conditionsForChargingFlag);
    Serial.print(F(",\"mpptPWM\":")); Serial.print(mpptPwmValue);
    Serial.print(F(",\"motDet\":")); Serial.print(motionDetectedFlag); // True if light is on due to motion
    Serial.print(F(",\"ledPWM\":")); Serial.print(currentLedPwmValue);
    Serial.print(F(",\"slpCycRemain\":")); Serial.print(wdtSleepCyclesTarget > 0 ? (wdtSleepCyclesTarget - wdtSleepCyclesCount) : 0);
    Serial.println(F("}"));
    lastJsonEmitMillis = millis();
  }
}

// --- Sleep Management ---
void disableWDT() {
  // MCUSR &= ~(1<<WDRF); // optional: clear WDRF
  WDTCSR |= (1<<WDCE) | (1<<WDE); // allow changes
  WDTCSR = 0x00; // disable WDT
}

void configureWDTForInterrupt(uint8_t prescaler_bits) {
  // Prescaler bits examples:
  // 0b00000110 -> 1S
  // 0b00000111 -> 2S
  // 0b00100000 -> 4S
  // 0b00100001 -> 8S (Max)
  WDTCSR = (1 << WDCE) | (1 << WDE);              // Enable configuration change
  WDTCSR = (1 << WDIE) | prescaler_bits;          // Enable WDT Interrupt, set prescaler
}

void enterSleepCycle() {
  // Serial.println(F("Preparing to sleep..."));
  // Serial.flush(); // Ensure Serial buffer is empty

  // Turn off LED and MPPT before sleeping
  if (currentLedPwmValue > PWM_MIN) turnOffLedCompletely();
  if (mpptPwmValue > PWM_MIN) {
      analogWrite(mpptPwmPin, PWM_MIN);
      mpptPwmValue = PWM_MIN;
  }

  // Disable ADC and BOD for power saving
  ADCSRA &= ~(1 << ADEN); // Disable ADC
  // power_adc_disable(); // Alternative using avr/power.h
  
  // Configure INT0 for motion detection (Pin D2)
  // Wake on RISING edge for typical PIR sensors
  EICRA = (1 << ISC01) | (1 << ISC00); // Set INT0 to trigger on RISING edge
  EIFR = (1 << INTF0);    // Clear any existing INT0 interrupt flag
  EIMSK |= (1 << INT0);   // Enable INT0

  // Configure WDT for timed wakeup (e.g., 8 seconds cycle)
  // Max WDT period is 8s.
  configureWDTForInterrupt(0b00100001); // 8S timeout

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  noInterrupts();         // Timed sequence critical_section_1_start
  sleep_enable();
  sleep_bod_disable();    // Disable Brown-Out Detector
  interrupts();           // Timed sequence critical_section_1_end
  
  // Serial.println(F("Zzz...")); Serial.flush();
  sleep_cpu();            // Enter sleep mode. Execution stops here until an interrupt.
  // --- MCU WAKES UP HERE ---
  
  sleep_disable();        // Disable sleep mode bits
  // Interrupts (INT0, WDT) are disabled within their ISRs to prevent immediate re-triggering.
  
  // Re-enable ADC
  ADCSRA |= (1 << ADEN); // Enable ADC
  // power_adc_enable();

  // Serial.println(F("...Awake!")); Serial.flush();
  // Global interrupt flag (motionWakeupOccurred or wdtWakeupOccurred) will be set.
  // Loop will restart and handle the flag.
}

void checkAndInitiateSleep() {
  // Conditions to initiate a new timed sleep sequence:
  // 1. Battery is low-ish (e.g., below 50% to conserve) OR it's dark and idle for a while
  // 2. No current timed sleep sequence is active (wdtSleepCyclesTarget == 0)
  // 3. LED is effectively off (currentLedPwmValue <= PWM_DIM or PWM_MIN)
  // 4. No motion currently detected (motionDetectedFlag == false)

  bool shouldSleep = false;
  if (batteryLevel_percent < 40 && isDarkFlag) { // Aggressive sleep if battery is low
      shouldSleep = true;
      Serial.println(F("Low battery, initiating timed sleep."));
  } else if (isDarkFlag && currentLedPwmValue <= PWM_DIM && !motionDetectedFlag && 
             (millis() - lastLightActivityMillis > 300000UL) ) { // Dark, idle for 5 minutes
      shouldSleep = true;
      Serial.println(F("Dark and idle, initiating timed sleep."));
  }


  if (shouldSleep && wdtSleepCyclesTarget == 0) {
    wdtSleepCyclesCount = 0;
    uint8_t wdt_8s_periods = DEEP_SLEEP_TARGET_DURATION_S / 8;
    if (DEEP_SLEEP_TARGET_DURATION_S % 8 > 0) wdt_8s_periods++; // Account for partial cycle
    wdtSleepCyclesTarget = max(1, wdt_8s_periods); // Sleep for at least one 8s cycle

    Serial.print(F("Sleep sequence started. Target WDT cycles: ")); Serial.println(wdtSleepCyclesTarget);
    // The main loop will call enterSleepCycle() if wdtSleepCyclesTarget > 0
  }
}

// --- Utility Functions ---
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) { /* ... unchanged ... */
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
