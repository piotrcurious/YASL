// Arduino code for solar powered motion sensor lamp
// With INA219, basic MPPT, and ADVANCED SLEEP MODES
// Updated & fixed to be fully functional on AVR/ATmega328P

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <Arduino.h>

// --- Pin Definitions ---
const uint8_t solarPin = A0;       // Analog pin for a separate solar panel voltage sensor (optional)
const uint8_t batteryPin = A1;     // Analog pin for battery voltage sensor
const uint8_t ledPwmPin = 3;       // PWM pin for LED light (Timer2)
const uint8_t motionPin = 2;       // Digital pin for PIR motion sensor (INT0)
const uint8_t mpptPwmPin = 9;      // PWM pin for MPPT control (Timer1)

// --- System Parameters ---
const float ADC_REFERENCE_VOLTAGE = 5.0f;
const float ADC_MAX_VALUE = 1023.0f;
const float SOLAR_VOLTAGE_DIVIDER_RATIO = 4.0f; // e.g., Rtop=3Rbottom -> (Rtop+Rbottom)/Rbottom = 4
const float BATTERY_VOLTAGE_DIVIDER_RATIO = 3.0f; // e.g., (Rtop+Rbottom)/Rbottom = 3
const float SOLAR_DARK_THRESHOLD_ACTUAL_VOLTAGE = 2.0f; // INA219 bus voltage considered "dark"
const float BATTERY_MAX_VOLTAGE = 4.2f;
const float BATTERY_MIN_VOLTAGE = 3.0f;
const float BATTERY_PROTECTION_VOLTAGE = 3.2f;
const float BATTERY_CHARGE_TARGET_VOLTAGE = 4.1f;
const float MIN_SOLAR_VOLTAGE_FOR_MPPT_ACTUAL = 6.0f;
const int PWM_MAX = 255;
const int PWM_MIN = 0;
const int PWM_DIM = 20; // Slightly dimmer for power saving
const int PWM_STEP_DEFAULT = 2; // default MPPT step
const long JSON_EMIT_INTERVAL_MS = 30000;
const long MOTION_DEBOUNCE_MS = 200;
const long LIGHT_ON_DURATION_AFTER_MOTION_MS = 10000; // 10 seconds
const long MPPT_UPDATE_INTERVAL_MS = 1000;
const uint16_t DEEP_SLEEP_TARGET_DURATION_S = 60; // Desired deep sleep length (seconds)

// --- INA219 Sensor ---
Adafruit_INA219 ina219(0x40);

// --- Global Variables ---
// Measurements
float solarVoltage_raw_adc = 0.0f;
float batteryVoltage_actual = 0.0f;
float batteryLevel_percent = 0.0f;
float solarBusVoltage_V = 0.0f;
float solarShuntVoltage_mV = 0.0f;
float solarCurrent_mA = 0.0f;
float solarPower_mW = 0.0f;

// Light Control
int currentLedPwmValue = PWM_MIN;
int targetLedPwmValue = PWM_MIN;
volatile bool motionDetectedFlag = false;
bool isDarkFlag = true;

// Charging & MPPT
bool conditionsForChargingFlag = false;
int mpptPwmValue = PWM_MIN;
float lastMpptPower_mW = 0.0f;
int mpptPwmStep = PWM_STEP_DEFAULT;
bool mpptPerturbDirectionUp = true;

// Timers
unsigned long lastJsonEmitMillis = 0;
unsigned long lastMotionDetectMillis = 0;
unsigned long lastLightActivityMillis = 0;
unsigned long lastMpptUpdateMillis = 0;

// Sleep Management
volatile bool motionWakeupOccurred = false;
volatile bool wdtWakeupOccurred = false;
uint16_t wdtSleepCyclesTarget = 0;
uint16_t wdtSleepCyclesCount = 0;

// Forward declarations
void enterSleepCycle();
void disableWDT();
void configureWDTFor8sInterrupt();
void readAnalogSensors();
void readIna219Data();
void determineSystemState();
void runMpptAlgorithm();
void handleMotionAndLight();
void turnOffLedCompletely();
void emitJsonData();
void checkAndInitiateSleep();
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

// --- ISRs ---
// Motion (external INT0) -- PIR on D2 (INT0)
ISR(INT0_vect) {
  // Set a flag and disable INT0 to avoid re-triggering until handled.
  motionWakeupOccurred = true;
  // Disable INT0
  EIMSK &= ~(1 << INT0);
}

// Watchdog interrupt
ISR(WDT_vect) {
  wdtWakeupOccurred = true;
  // WDIE will be cleared on some AVR variants; we'll reconfigure before next sleep
}

// --- Setup ---
void setup() {
  // Ensure WDT off to start clean
  disableWDT();

  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 2000)) { /* wait with timeout */ }
  Serial.println(F("\nSystem Initializing..."));

  pinMode(solarPin, INPUT);
  pinMode(batteryPin, INPUT);
  pinMode(ledPwmPin, OUTPUT);
  pinMode(mpptPwmPin, OUTPUT);
  // Configure PIR pin as input; use INPUT_PULLUP if your PIR is open-collector or if you want defined idle level.
  // If your PIR is active HIGH and provides a driven output, change to INPUT.
  pinMode(motionPin, INPUT_PULLUP);

  if (!ina219.begin()) {
    Serial.println(F("Failed to find INA219 chip. Halting."));
    while (1) {
      delay(100);
    }
  }
  Serial.println(F("INA219 sensor initialized."));

  // Set Timer2 prescaler for PWM on pin 3 ~976Hz (prescaler 64)
  TCCR2B = (TCCR2B & 0b11111000) | 0x04;

  analogWrite(ledPwmPin, PWM_MIN);
  currentLedPwmValue = PWM_MIN;
  targetLedPwmValue = PWM_MIN;

  mpptPwmValue = PWM_MIN;
  analogWrite(mpptPwmPin, mpptPwmValue);
  lastMpptPower_mW = 0.0f;

  unsigned long currentMillis = millis();
  lastJsonEmitMillis = currentMillis;
  lastMotionDetectMillis = currentMillis;
  lastLightActivityMillis = currentMillis;
  lastMpptUpdateMillis = currentMillis;

  Serial.println(F("Setup Complete."));
}

// --- Main loop ---
void loop() {
  // Handle wakeup events
  if (motionWakeupOccurred) {
    motionWakeupOccurred = false;
    wdtSleepCyclesTarget = 0;
    wdtSleepCyclesCount = 0;
    motionDetectedFlag = true;
    lastLightActivityMillis = millis();
    Serial.println(F("Wake: motion event"));
    // INT0 is disabled by ISR; re-enable later when ready
  }

  if (wdtWakeupOccurred) {
    wdtWakeupOccurred = false;
    if (wdtSleepCyclesTarget > 0) {
      wdtSleepCyclesCount++;
      Serial.print(F("Wake: WDT cycle ")); Serial.println(wdtSleepCyclesCount);
    }
  }

  // Re-enter sleep cycles if we still need to and there is no active motion processing
  if (wdtSleepCyclesTarget > 0 && wdtSleepCyclesCount < wdtSleepCyclesTarget && !motionDetectedFlag) {
    enterSleepCycle();
    // execution resumes on wake, return to loop top
    return;
  } else if (wdtSleepCyclesCount >= wdtSleepCyclesTarget && wdtSleepCyclesTarget > 0) {
    // Completed requested cycles
    wdtSleepCyclesTarget = 0;
    wdtSleepCyclesCount = 0;
    Serial.println(F("Timed sleep target reached."));
  }

  // Normal runtime behavior
  readAnalogSensors();
  readIna219Data();
  determineSystemState();

  if (isDarkFlag) {
    // If dark, generally disable MPPT to save power unless you prefer otherwise
    if (mpptPwmValue > PWM_MIN) {
      analogWrite(mpptPwmPin, PWM_MIN);
      mpptPwmValue = PWM_MIN;
    }
    handleMotionAndLight();
  } else { // Daytime
    if (currentLedPwmValue > PWM_MIN) turnOffLedCompletely();
    if (conditionsForChargingFlag) {
      runMpptAlgorithm();
    } else {
      if (mpptPwmValue > PWM_MIN) {
        analogWrite(mpptPwmPin, PWM_MIN);
        mpptPwmValue = PWM_MIN;
      }
    }
  }

  emitJsonData();
  checkAndInitiateSleep();
}

// --- Sensor Reading ---
void readAnalogSensors() {
  // Read raw ADC for optional solar ADC input (may be unused if using INA219)
  float rawSolar = analogRead(solarPin) * (ADC_REFERENCE_VOLTAGE / ADC_MAX_VALUE);
  solarVoltage_raw_adc = rawSolar * SOLAR_VOLTAGE_DIVIDER_RATIO; // optional calibrated value

  float batteryPinVoltage = analogRead(batteryPin) * (ADC_REFERENCE_VOLTAGE / ADC_MAX_VALUE);
  batteryVoltage_actual = batteryPinVoltage * BATTERY_VOLTAGE_DIVIDER_RATIO;
  batteryLevel_percent = mapFloat(batteryVoltage_actual, BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE, 0.0f, 100.0f);
  batteryLevel_percent = constrain(batteryLevel_percent, 0.0f, 100.0f);
}

void readIna219Data() {
  // Read INA219 values (bus voltage, shunt, current, power)
  // Adafruit INA219 functions return floats
  solarShuntVoltage_mV = ina219.getShuntVoltage_mV();
  solarBusVoltage_V = ina219.getBusVoltage_V();
  solarCurrent_mA = ina219.getCurrent_mA();
  solarPower_mW = ina219.getPower_mW();

  // Defensive: if bus reads near zero but INA reports current, sanitize
  if (solarBusVoltage_V < 0.5f && solarCurrent_mA > 100.0f) {
    solarCurrent_mA = 0.0f;
    solarPower_mW = 0.0f;
  }
}

// --- System State Determination ---
void determineSystemState() {
  isDarkFlag = (solarBusVoltage_V < SOLAR_DARK_THRESHOLD_ACTUAL_VOLTAGE);

  conditionsForChargingFlag = (!isDarkFlag &&
                              batteryVoltage_actual < BATTERY_CHARGE_TARGET_VOLTAGE &&
                              solarBusVoltage_V > MIN_SOLAR_VOLTAGE_FOR_MPPT_ACTUAL &&
                              solarPower_mW >= 0.0f);
}

// --- MPPT Algorithm (perturb & observe simple) ---
void runMpptAlgorithm() {
  unsigned long now = millis();
  if ((long)(now - lastMpptUpdateMillis) < (long)MPPT_UPDATE_INTERVAL_MS) return;

  // Add hysteresis to avoid toggling on noise
  const float POWER_HYST = 10.0f;

  if (solarPower_mW > lastMpptPower_mW + POWER_HYST) {
    // keep direction
  } else if (solarPower_mW < lastMpptPower_mW - POWER_HYST) {
    // reverse direction
    mpptPerturbDirectionUp = !mpptPerturbDirectionUp;
  }
  // apply perturbation
  if (mpptPerturbDirectionUp) mpptPwmValue += mpptPwmStep;
  else mpptPwmValue -= mpptPwmStep;

  mpptPwmValue = constrain(mpptPwmValue, PWM_MIN, PWM_MAX);
  analogWrite(mpptPwmPin, mpptPwmValue);

  lastMpptPower_mW = solarPower_mW;
  lastMpptUpdateMillis = now;
}

// --- Motion and Light Control ---
void handleMotionAndLight() {
  unsigned long now = millis();

  // Re-enable INT0 if needed (we disabled it after ISR)
  if (!motionDetectedFlag && wdtSleepCyclesTarget == 0) {
    if ((EIMSK & (1 << INT0)) == 0) {
      EIFR = (1 << INTF0); // clear pending
      EIMSK |= (1 << INT0); // enable
    }
  }

  // If battery critically low, force LED off
  if (batteryVoltage_actual < BATTERY_PROTECTION_VOLTAGE) {
    if (currentLedPwmValue > PWM_MIN) Serial.println(F("Battery critically low, LED off."));
    targetLedPwmValue = PWM_MIN;
    motionDetectedFlag = false;
  } else if (motionDetectedFlag) {
    // If we have a motion event, keep LED on for the configured duration
    if ((long)(now - lastLightActivityMillis) < (long)LIGHT_ON_DURATION_AFTER_MOTION_MS) {
      // Scale brightness by battery voltage to save power when low
      int computed = map((int)constrain(batteryVoltage_actual * 100, (int)(BATTERY_MIN_VOLTAGE * 100), (int)(BATTERY_MAX_VOLTAGE * 100)),
                         (int)(BATTERY_MIN_VOLTAGE * 100), (int)(BATTERY_MAX_VOLTAGE * 100),
                         PWM_DIM, PWM_MAX);
      targetLedPwmValue = constrain(computed, PWM_MIN, PWM_MAX);
    } else {
      // Timeout expired -> go to dim
      motionDetectedFlag = false;
      targetLedPwmValue = PWM_DIM;
    }
  } else {
    // No active motion, default dim when dark
    targetLedPwmValue = PWM_DIM;
  }

  // Apply LED PWM if changed
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

// --- JSON Emission ---
void emitJsonData() {
  unsigned long now = millis();
  if ((long)(now - lastJsonEmitMillis) > (long)JSON_EMIT_INTERVAL_MS) {
    Serial.print(F("{"));
    Serial.print(F("\"sV_bus\":")); Serial.print(solarBusVoltage_V, 2);
    Serial.print(F(",\"sI_mA\":")); Serial.print(solarCurrent_mA, 1);
    Serial.print(F(",\"sP_mW\":")); Serial.print(solarPower_mW, 0);
    Serial.print(F(",\"batV\":")); Serial.print(batteryVoltage_actual, 2);
    Serial.print(F(",\"batPcnt\":")); Serial.print(batteryLevel_percent, 0);
    Serial.print(F(",\"isDark\":")); Serial.print(isDarkFlag ? "true" : "false");
    Serial.print(F(",\"isChg\":")); Serial.print(conditionsForChargingFlag ? "true" : "false");
    Serial.print(F(",\"mpptPWM\":")); Serial.print(mpptPwmValue);
    Serial.print(F(",\"motDet\":")); Serial.print(motionDetectedFlag ? "true" : "false");
    Serial.print(F(",\"ledPWM\":")); Serial.print(currentLedPwmValue);
    Serial.print(F(",\"slpCycRemain\":")); Serial.print(wdtSleepCyclesTarget > 0 ? (wdtSleepCyclesTarget - wdtSleepCyclesCount) : 0);
    Serial.println(F("}"));
    lastJsonEmitMillis = now;
  }
}

// --- Sleep Management ---
void disableWDT() {
  cli();
  wdt_reset();
  MCUSR &= ~(1 << WDRF);
  // WDCE and WDE must both be set in same operation
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = 0x00; // turn off WDT
  sei();
}

void configureWDTFor8sInterrupt() {
  // Configure watchdog for interrupt every 8s (no reset) using proper WDCE sequence.
  // According to ATmega328P datasheet, WDP3:0 = 1001 -> 8s
  cli();
  wdt_reset();
  // Enable change
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  // Set WDIE and WDP3 + WDP0 -> 8s interrupt mode
  WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);
  sei();
}

void enterSleepCycle() {
  Serial.println(F("Preparing to sleep..."));
  Serial.flush();

  // Turn off peripherals to save power
  if (currentLedPwmValue > PWM_MIN) turnOffLedCompletely();
  if (mpptPwmValue > PWM_MIN) {
    analogWrite(mpptPwmPin, PWM_MIN);
    mpptPwmValue = PWM_MIN;
  }

  // Disable ADC
  ADCSRA &= ~(1 << ADEN);

  // Configure INT0 to wake on rising edge (typical PIR active HIGH)
  EICRA = (1 << ISC01) | (1 << ISC00); // ISC01=1 ISC00=1 -> rising edge
  EIFR = (1 << INTF0); // clear any pending
  EIMSK |= (1 << INT0); // enable INT0

  // Configure WDT for 8s wakeup
  configureWDTFor8sInterrupt();

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  noInterrupts();
  sleep_enable();
  // Disable BOD while sleeping (if supported)
#if defined(SLEEP_BOD_DISABLE)
  sleep_bod_disable();
#endif
  interrupts();

  Serial.println(F("Going to sleep..."));
  sleep_cpu();

  // Execution resumes here after wake
  sleep_disable();

  // Re-enable ADC
  ADCSRA |= (1 << ADEN);

  // Clear any pending flags and re-enable global interrupts (ISRs set flags)
  EIFR = (1 << INTF0);
  Serial.println(F("Woke up from sleep."));
}

// --- Decide whether to initiate sleep ---
void checkAndInitiateSleep() {
  bool shouldSleep = false;

  if (batteryLevel_percent < 40.0f && isDarkFlag) {
    shouldSleep = true;
    Serial.println(F("Low battery, initiating timed sleep."));
  } else if (isDarkFlag && currentLedPwmValue <= PWM_DIM && !motionDetectedFlag &&
             (millis() - lastLightActivityMillis > 300000UL)) {
    shouldSleep = true;
    Serial.println(F("Dark and idle, initiating timed sleep."));
  }

  if (shouldSleep && wdtSleepCyclesTarget == 0) {
    wdtSleepCyclesCount = 0;
    uint16_t wdt_8s_periods = DEEP_SLEEP_TARGET_DURATION_S / 8;
    if (DEEP_SLEEP_TARGET_DURATION_S % 8 > 0) wdt_8s_periods++;
    if (wdt_8s_periods < 1) wdt_8s_periods = 1;
    wdtSleepCyclesTarget = wdt_8s_periods;

    Serial.print(F("Sleep sequence started. Target WDT cycles: "));
    Serial.println(wdtSleepCyclesTarget);
  }
}

// --- Simple utility map for floats ---
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
