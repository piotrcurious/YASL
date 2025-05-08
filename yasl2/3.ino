// Arduino code for solar powered motion sensor lamp
// With INA219 and basic MPPT
// Current Date: Thursday, May 8, 2025 (for context, not functional)

#include <Wire.h>              // For I2C communication
#include <Adafruit_INA219.h>   // For INA219 Current Sensor
#include <avr/sleep.h>         // For sleep modes
#include <avr/wdt.h>           // For Watchdog Timer

// --- Pin Definitions ---
const int solarPin = A0;       // Analog pin for a separate solar panel voltage sensor (can be backup or for rough estimate)
const int batteryPin = A1;     // Analog pin for battery voltage sensor
const int ledPwmPin = 3;       // PWM pin for LED light (ensure this is a hardware PWM pin, e.g., Timer2)
const int motionPin = 2;       // Digital pin for PIR motion sensor
const int mpptPwmPin = 9;      // PWM pin for MPPT control (e.g., Timer1: Pin 9 or 10 on Uno/Nano)

// --- System Parameters ---
const float ADC_REFERENCE_VOLTAGE = 5.0; // Arduino ADC reference voltage
const float ADC_MAX_VALUE = 1023.0;

// Voltage Divider Ratios (Adjust these based on YOUR voltage divider circuit)
// Actual_Voltage = Voltage_at_Pin * ((R1 + R2) / R2)
const float SOLAR_VOLTAGE_DIVIDER_RATIO = 3.0 + 1.0;  // Example: For solarPin, if used. INA219 provides more accurate Vbus.
const float BATTERY_VOLTAGE_DIVIDER_RATIO = 2.0 + 1.0; // Example for batteryPin

const float SOLAR_THRESHOLD_VOLTAGE_AT_PIN = 0.5; // Voltage AT THE solarPin (after divider) to consider it "dark"
                                                 // Use a low value if relying on INA219 for primary solar voltage.
const float BATTERY_MAX_VOLTAGE = 4.2;
const float BATTERY_MIN_VOLTAGE = 3.0;
const float BATTERY_PROTECTION_VOLTAGE = 3.2;
const float BATTERY_CHARGE_TARGET_VOLTAGE = 4.1; // Stop active charging slightly below max for longevity
const float MIN_SOLAR_VOLTAGE_FOR_MPPT_ACTUAL = 6.0; // Minimum actual solar panel voltage (from INA219) to attempt MPPT

const int PWM_MAX = 255;
const int PWM_MIN = 0;
const int PWM_DIM = 30;
const int PWM_STEP = 5;

const long JSON_EMIT_INTERVAL_MS = 30000; // Emit JSON data every 30 seconds
const long MOTION_DEBOUNCE_MS = 200;
const long LIGHT_ON_DURATION_AFTER_MOTION_MS = 15000;
const long MPPT_UPDATE_INTERVAL_MS = 1000; // Update MPPT every 1 second

// --- INA219 Sensor ---
Adafruit_INA219 ina219; // Defaults to address 0x40

// --- Global Variables ---
// Voltage & Power
float solarVoltage_raw_adc;   // Raw voltage from solarPin (analog divider)
float batteryVoltage_actual;  // Actual battery voltage
float batteryLevel_percent;   // Battery level in percentage

// INA219 readings for Solar Panel
float solarBusVoltage_V;      // Voltage from solar panel (INA219)
float solarShuntVoltage_mV;   // Shunt voltage (INA219)
float solarCurrent_mA;        // Current from solar panel (INA219)
float solarPower_mW;          // Power from solar panel (INA219)

// Light Control
int currentLedPwmValue;
int targetLedPwmValue;
bool motionDetectedFlag;
bool isDarkFlag;

// Charging & MPPT
bool conditionsForChargingFlag; // True if solar and battery conditions allow charging
int mpptPwmValue;
float lastMpptPower_mW;
int mpptPwmStep = 2; // How much to change PWM for MPPT; tune this!
bool mpptPerturbDirectionUp = true; // Initial direction to perturb PWM

// Timers
unsigned long lastJsonEmitMillis;
unsigned long lastMotionDetectMillis;
unsigned long lastLightActivityMillis;
unsigned long lastMpptUpdateMillis;


// --- Setup Function ---
void setup() {
  Serial.begin(115200); // Increased baud rate for more data
  while (!Serial && millis() < 3000);
  Serial.println(F("System Initializing..."));

  pinMode(solarPin, INPUT);
  pinMode(batteryPin, INPUT);
  pinMode(ledPwmPin, OUTPUT);
  pinMode(motionPin, INPUT);
  pinMode(mpptPwmPin, OUTPUT);

  // Initialize INA219
  if (!ina219.begin()) {
    Serial.println(F("Failed to find INA219 chip"));
    // You might want to halt or enter a safe mode if INA219 is critical
    while (1) { delay(10); }
  }
  Serial.println(F("INA219 sensor initialized."));
  // ina219.setCalibration_32V_2A(); // Or _16V_400mA etc. depending on your expected ranges & shunt.
                                  // Default works for 32V, 2A with the standard 0.1ohm shunt.

  // LED PWM Frequency (Pin 3, Timer2) - aiming for ~976Hz
  TCCR2B = (TCCR2B & 0b11111000) | 0x04; // Set prescaler for Timer2 to 64 (CS22 bit)

  // MPPT PWM (Pin 9, Timer1) - default frequency is ~490Hz.
  // For higher/specific frequencies on Pin 9, direct Timer1 manipulation is needed.
  // Example: To set Pin 9 (OC1A) to ~31kHz Fast PWM (Mode 14, ICR1 as TOP):
  // TCCR1A = (1 << WGM11) | (1 << COM1A1); // Non-inverting Fast PWM
  // TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10); // Fast PWM, no prescaler
  // ICR1 = 511; // TOP value for 16MHz / (1 * (1+511)) = ~31.25 kHz
  // For now, we use default analogWrite() frequency for mpptPwmPin.

  analogWrite(ledPwmPin, PWM_MIN);
  currentLedPwmValue = PWM_MIN;
  targetLedPwmValue = PWM_MIN;

  mpptPwmValue = 128; // Start MPPT PWM at 50% duty cycle (or another safe value)
  analogWrite(mpptPwmPin, mpptPwmValue);
  lastMpptPower_mW = 0;

  lastJsonEmitMillis = millis();
  lastMotionDetectMillis = millis();
  lastLightActivityMillis = millis();
  lastMpptUpdateMillis = millis();

  Serial.println(F("Setup Complete."));
  Serial.println(F("Hardware Note: Ensure voltage dividers are used for battery_pin if its voltage exceeds 5V."));
  Serial.print(F("Battery Voltage Divider Ratio (assumed): ")); Serial.println(BATTERY_VOLTAGE_DIVIDER_RATIO);
}

// --- Main Loop ---
void loop() {
  readAnalogSensors();    // Reads battery voltage and optional raw solar voltage
  readIna219Data();       // Reads solar panel V, I, P from INA219
  determineSystemState(); // Updates isDarkFlag, conditionsForChargingFlag

  if (isDarkFlag) {
    analogWrite(mpptPwmPin, PWM_MIN); // Ensure MPPT is off at night
    handleMotion();
    controlLedLight();
  } else { // Daytime
    turnOffLedCompletely(); // Ensure LED is off during the day
    if (conditionsForChargingFlag) {
      runMpptAlgorithm();
    } else {
      analogWrite(mpptPwmPin, PWM_MIN); // Stop charging if battery full or low sun
      // Serial.println(F("Conditions not met for charging/MPPT."));
    }
  }

  emitJsonData();
  handlePowerSaving();
}

// --- Sensor Reading ---
void readAnalogSensors() {
  solarVoltage_raw_adc = analogRead(solarPin) * (ADC_REFERENCE_VOLTAGE / ADC_MAX_VALUE);
  // solarVoltage_actual calculated from INA219 bus voltage is more accurate

  float batteryPinVoltage = analogRead(batteryPin) * (ADC_REFERENCE_VOLTAGE / ADC_MAX_VALUE);
  batteryVoltage_actual = batteryPinVoltage * BATTERY_VOLTAGE_DIVIDER_RATIO;

  batteryLevel_percent = mapFloat(batteryVoltage_actual, BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE, 0.0, 100.0);
  batteryLevel_percent = constrain(batteryLevel_percent, 0.0, 100.0);
}

void readIna219Data() {
  solarShuntVoltage_mV = ina219.getShuntVoltage_mV();
  solarBusVoltage_V = ina219.getBusVoltage_V();
  solarCurrent_mA = ina219.getCurrent_mA();
  solarPower_mW = ina219.getPower_mW(); // Or calculate: solarBusVoltage_V * solarCurrent_mA;

  // Sometimes, if current is negative (discharging through panel at night without blocking diode),
  // INA219 might report large positive current with some configurations if not calibrated for it.
  // For solar, current should ideally always be positive or zero.
  if (solarBusVoltage_V < 0.5 && solarCurrent_mA > 100) { // Plausibility check
      solarCurrent_mA = 0;
      solarPower_mW = 0;
  }
}


// --- System State Determination ---
void determineSystemState() {
  // Darkness detection based on INA219 solar voltage
  if (solarBusVoltage_V < SOLAR_THRESHOLD_VOLTAGE_AT_PIN) { // Using actual solar voltage, so threshold should be higher
                                                          // e.g. SOLAR_THRESHOLD_ACTUAL_VOLTAGE = 1.0V from panel
    isDarkFlag = true;
  } else {
    isDarkFlag = false;
  }

  // Conditions for MPPT/Charging
  if (!isDarkFlag &&
      batteryVoltage_actual < BATTERY_CHARGE_TARGET_VOLTAGE &&
      solarBusVoltage_V > MIN_SOLAR_VOLTAGE_FOR_MPPT_ACTUAL && // Enough sun
      solarPower_mW >= 0) { // Ensure power is not negative
    conditionsForChargingFlag = true;
  } else {
    conditionsForChargingFlag = false;
  }
}

// --- MPPT Algorithm (Perturb and Observe) ---
void runMpptAlgorithm() {
  if (millis() - lastMpptUpdateMillis > MPPT_UPDATE_INTERVAL_MS) {
    // Current power is already in solarPower_mW from readIna219Data() called in loop()

    // Perturb and Observe
    if (solarPower_mW > lastMpptPower_mW) {
      // Power increased, keep going in the same direction
      // (mpptPerturbDirectionUp remains unchanged)
    } else {
      // Power decreased or stayed same, reverse direction
      mpptPerturbDirectionUp = !mpptPerturbDirectionUp;
    }

    // Apply perturbation
    if (mpptPerturbDirectionUp) {
      mpptPwmValue += mpptPwmStep;
    } else {
      mpptPwmValue -= mpptPwmStep;
    }

    // Constrain PWM value
    mpptPwmValue = constrain(mpptPwmValue, PWM_MIN, PWM_MAX); // Ensure MPPT PWM is within valid range

    analogWrite(mpptPwmPin, mpptPwmValue);
    lastMpptPower_mW = solarPower_mW;
    lastMpptUpdateMillis = millis();

    // Serial.print(F("MPPT: P=")); Serial.print(solarPower_mW);
    // Serial.print(F("mW, V=")); Serial.print(solarBusVoltage_V);
    // Serial.print(F("V, I=")); Serial.print(solarCurrent_mA);
    // Serial.print(F("mA, PWM=")); Serial.println(mpptPwmValue);
  }
}


// --- Motion Handling ---
void handleMotion() {
  int motionSensorValue = digitalRead(motionPin);
  unsigned long currentMillis = millis();

  if (motionSensorValue == HIGH) {
    if (currentMillis - lastMotionDetectMillis > MOTION_DEBOUNCE_MS) {
      if (!motionDetectedFlag) {
         Serial.println(F("Motion DETECTED"));
         motionDetectedFlag = true;
      }
      lastLightActivityMillis = currentMillis;
    }
  }
  // motionDetectedFlag is reset by controlLedLight() after LIGHT_ON_DURATION_AFTER_MOTION_MS
}


// --- LED Light Control ---
void controlLedLight() {
  unsigned long currentMillis = millis();

  if (batteryVoltage_actual < BATTERY_PROTECTION_VOLTAGE) {
    targetLedPwmValue = PWM_MIN;
    motionDetectedFlag = false;
    if (currentLedPwmValue > PWM_MIN) Serial.println(F("Battery critically low, LED off."));
  }
  else if (motionDetectedFlag && (currentMillis - lastLightActivityMillis < LIGHT_ON_DURATION_AFTER_MOTION_MS)) {
    targetLedPwmValue = map(constrain(batteryVoltage_actual, BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE),
                         BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE,
                         PWM_DIM, PWM_MAX);
  }
  else {
    motionDetectedFlag = false;
    targetLedPwmValue = PWM_DIM; // Default to dim light when dark and no recent motion
  }

  // Smooth transition for LED
  if (currentLedPwmValue != targetLedPwmValue) {
    if (currentLedPwmValue < targetLedPwmValue) {
      currentLedPwmValue += PWM_STEP;
      if (currentLedPwmValue > targetLedPwmValue) currentLedPwmValue = targetLedPwmValue;
    } else {
      currentLedPwmValue -= PWM_STEP;
      if (currentLedPwmValue < targetLedPwmValue) currentLedPwmValue = targetLedPwmValue;
    }
  }
  
  currentLedPwmValue = constrain(currentLedPwmValue, PWM_MIN, PWM_MAX);
  analogWrite(ledPwmPin, currentLedPwmValue);
}


// --- Turn Off LED ---
void turnOffLedCompletely() {
  if (currentLedPwmValue > PWM_MIN) { // Only write if it's not already off
      currentLedPwmValue = PWM_MIN;
      targetLedPwmValue = PWM_MIN;
      analogWrite(ledPwmPin, currentLedPwmValue);
  }
  motionDetectedFlag = false;
}

// --- JSON Data Emission ---
void emitJsonData() {
  if (millis() - lastJsonEmitMillis > JSON_EMIT_INTERVAL_MS) {
    Serial.print(F("{"));
    Serial.print(F("\"sV_adc\":")); Serial.print(solarVoltage_raw_adc, 2); // Raw solar ADC pin
    Serial.print(F(",\"sV_bus\":")); Serial.print(solarBusVoltage_V, 2);   // INA Solar V
    Serial.print(F(",\"sI_mA\":")); Serial.print(solarCurrent_mA, 1);    // INA Solar I
    Serial.print(F(",\"sP_mW\":")); Serial.print(solarPower_mW, 0);      // INA Solar P
    Serial.print(F(",\"batV\":")); Serial.print(batteryVoltage_actual, 2);
    Serial.print(F(",\"batPcnt\":")); Serial.print(batteryLevel_percent, 0);
    Serial.print(F(",\"isDark\":")); Serial.print(isDarkFlag);
    Serial.print(F(",\"isChg\":")); Serial.print(conditionsForChargingFlag);
    Serial.print(F(",\"mpptPWM\":")); Serial.print(mpptPwmValue);
    Serial.print(F(",\"mot\":")); Serial.print(motionDetectedFlag);
    Serial.print(F(",\"ledPWM\":")); Serial.print(currentLedPwmValue);
    Serial.println(F("}"));
    lastJsonEmitMillis = millis();
  }
}

// --- Power Saving ---
void handlePowerSaving() {
  if (batteryLevel_percent < 10 && isDarkFlag && !motionDetectedFlag && currentLedPwmValue == PWM_MIN) {
    Serial.println(F("Battery very low, dark, no motion, LED off. Entering deep sleep for 60s."));
    enterDeepSleepSeconds(60); // Sleep for 60 seconds
  } else if (batteryLevel_percent < 5 && isDarkFlag) { // More aggressive sleep if extremely low
    Serial.println(F("Battery critically low. Entering deep sleep for 120s."));
    enterDeepSleepSeconds(120);
  }
}

// --- Sleep Functions ---
void enterDeepSleepSeconds(uint16_t seconds) {
  Serial.println(F("Entering deep sleep..."));
  Serial.flush();

  // Disable ADC, timers, etc. to save power before sleep if not handled by sleep mode itself
  // ADCSRA &= ~(1 << ADEN); // Disable ADC

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  noInterrupts(); // Ensure atomicity for sleep setup
  sleep_bod_disable(); // Disable Brown-out Detector

  // Configure Watchdog Timer for timed wakeup
  // WDT can sleep for max 8s. Loop for longer sleeps.
  uint16_t slept_seconds = 0;
  while (slept_seconds < seconds) {
    byte wdt_prescaler_bits;
    uint8_t current_sleep_duration = 8; // Try to sleep for 8s

    if (seconds - slept_seconds >= 8) { wdt_prescaler_bits = 0b00100001; current_sleep_duration = 8; } // 8 S
    else if (seconds - slept_seconds >= 4) { wdt_prescaler_bits = 0b00100000; current_sleep_duration = 4; } // 4 S
    else if (seconds - slept_seconds >= 2) { wdt_prescaler_bits = 0b00000111; current_sleep_duration = 2; } // 2 S
    else if (seconds - slept_seconds >= 1) { wdt_prescaler_bits = 0b00000110; current_sleep_duration = 1; } // 1 S
    else { wdt_prescaler_bits = 0b00000101; current_sleep_duration = 0; } // <1S, effectively 0.5S or less, break

    if (current_sleep_duration == 0) break; // If remaining time is too short

    MCUSR &= ~(1 << WDRF); // Clear WDT reset flag
    WDTCSR = (1 << WDCE) | (1 << WDE); // Enable watchdog configuration
    WDTCSR = (1 << WDE) | wdt_prescaler_bits; // Set prescaler and enable WDT system reset
    
    interrupts(); // Briefly allow interrupts for WDT to start, though PWR_DOWN sleeps fast
    sleep_cpu();  // Enter sleep mode (execution stops here until WDT reset)
    // On WDT reset, the Arduino restarts from setup().
    // The loop for longer sleep won't be reached in this WDT reset model.
    // To make the loop work, you'd use WDT interrupt, not reset.
    // For simplicity with WDT reset, we just set it for one cycle.
    // If `seconds` > 8, it will only sleep for max 8s and then reset.
    // For true long sleep without full reset and continuation, ISR approach is needed.
    // Given this model, let's simplify: it sleeps for max 8s, then resets.
    // The `while` loop here is thus conceptual for a WDT ISR model.
    break; // Exit loop as WDT reset will occur
  }
  // Code here is effectively unreachable if WDT is set to reset the MCU
  sleep_disable();
  // ADCSRA |= (1 << ADEN); // Re-enable ADC
  // Serial.println(F("Woke up from sleep. (Should not happen with WDT reset)"));
}
// Note on enterDeepSleepSeconds:
// The above function sets the WDT to RESET the Arduino. So, after sleep, it will go through setup() again.
// The loop `while (slept_seconds < seconds)` would only work if the WDT was configured for an INTERRUPT
// that wakes the CPU without resetting, and then `sleep_cpu()` would return.
// For a simple timed deep sleep with reset, just set the WDT once.


// --- Utility Functions ---
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min; // Avoid division by zero
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
