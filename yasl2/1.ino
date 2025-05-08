// Arduino code for solar powered motion sensor lamp
// Reviewed and Improved Version

#include <avr/sleep.h> // For sleep modes
#include <avr/wdt.h>   // For Watchdog Timer

// --- Pin Definitions ---
const int solarPin = A0;    // Analog pin for solar panel voltage sensor
const int batteryPin = A1;  // Analog pin for battery voltage sensor
const int pwmPin = 3;       // PWM pin for LED light (ensure this is a hardware PWM pin)
const int motionPin = 2;    // Digital pin for PIR motion sensor

// --- System Parameters ---
const float ADC_REFERENCE_VOLTAGE = 5.0; // Arduino ADC reference voltage
const float ADC_MAX_VALUE = 1023.0;

// NOTE: You MUST use voltage dividers for solarPin and batteryPin if voltages exceed ADC_REFERENCE_VOLTAGE.
// Adjust these R1 and R2 values based on YOUR voltage divider circuit.
// Voltage at Pin = Actual_Voltage * (R2 / (R1 + R2))
// So, Actual_Voltage = Voltage_at_Pin * ((R1 + R2) / R2)
const float SOLAR_VOLTAGE_DIVIDER_RATIO = 3.0 + 1.0; // Example: (10k + 5k) / 5k = 3.0. If no divider, set to 1.0
const float BATTERY_VOLTAGE_DIVIDER_RATIO = 2.0 + 1.0; // Example: (20k + 10k) / 10k = 3.0. If no divider, set to 1.0

const float SOLAR_THRESHOLD_VOLTAGE = 1.0; // Voltage from solar panel (at the pin, after divider) to consider it "dark"
const float BATTERY_MAX_VOLTAGE = 4.2;     // Maximum battery voltage for Li-ion
const float BATTERY_MIN_VOLTAGE = 3.0;     // Minimum battery voltage for Li-ion (critical)
const float BATTERY_PROTECTION_VOLTAGE = 3.2; // Turn off light if battery drops below this to protect it
const float BATTERY_CHARGE_TARGET_FRACTION = 0.90; // Charge up to 90% of batteryMax (e.g., 4.2V * 0.9 = 3.78V target if charging logic was here)
                                                 // For passive charging, this is more of a monitoring point.
const float MIN_SOLAR_VOLTAGE_FOR_CHARGING = 3.0; // Minimum solar panel output (actual, before divider) to consider it charging

const int PWM_MAX = 255;
const int PWM_MIN = 0;     // Light completely off
const int PWM_DIM = 30;    // A dim light level when no motion but dark
const int PWM_STEP = 5;    // Step size for ramping down

const long JSON_EMIT_INTERVAL_MS = 60000; // Emit JSON data every 60 seconds
const long MOTION_DEBOUNCE_MS = 200;      // Debounce time for motion sensor
const long LIGHT_ON_DURATION_AFTER_MOTION_MS = 15000; // Keep light on for 15s after last motion

// --- Global Variables ---
float solarVoltage_actual;    // Actual solar panel voltage
float batteryVoltage_actual;  // Actual battery voltage
float batteryLevel_percent;   // Battery level in percentage

int currentPwmValue;      // Current PWM value for LED light
int targetPwmValue;       // Target PWM for smooth transitions (not fully implemented here for ramp up)
bool motionDetectedFlag;  // True if motion is currently active
bool isDarkFlag;          // True if it's dark enough to operate the light
bool isChargingFlag;      // True if conditions suggest battery is charging

unsigned long lastJsonEmitMillis;
unsigned long lastMotionDetectMillis;
unsigned long lastLightActivityMillis; // For light on duration timer

// --- Setup Function ---
void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 3000); // Wait for serial, but timeout
  Serial.println("System Initializing...");

  pinMode(solarPin, INPUT);
  pinMode(batteryPin, INPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(motionPin, INPUT);

  // Set PWM frequency for Timer2 (pins 3 and 11 on Uno/Nano) to approx 976Hz (fast PWM)
  // TCCR2B = TCCR2B & B11111000 | B00000010; // Divisor = 8, ~3.9kHz
  TCCR2B = TCCR2B & 0b11111000 | 0x01; // Set no prescaler for Timer2 (pins 3, 11) -> ~31 kHz (might be too high for some drivers)
                                       // Default is ~490Hz (pins 5,6) or ~980Hz (pins 3,9,10,11)
                                       // Let's stick to a common one, often default works ok.
                                       // For pin 3 (OC2B): Default is good. If you need to change Timer2:
  // TCCR2B = (TCCR2B & 0b11111000) | 0x04; // ~244 Hz (divisor 64)
  // For simplicity, we can often rely on default PWM frequency or adjust if flickering/noise is an issue.
  // The original 1kHz setting: (TCCR2B & B11111000) | B00000011; // for Timer2, this is div 32, so ~976Hz.
  // TCCR2B = (TCCR2B & 0b11111000) | 0x03; // ~976 Hz on pins 3, 11 (Timer2)
  // Let's use the original approach for 1kHz on pin 3:
  TCCR2B = (TCCR2B & B11111000) | B00000011; // Sets divisor to 32 for Timer2 => 16MHz/32/256 = ~1.95kHz if phase-correct, or 16MHz/32/510 for some modes.
                                            // For Fast PWM (default mode for analogWrite): F_CPU / (N * 256). N=prescaler.
                                            // N=1 (CS20): 16MHz/256 = 62.5kHz
                                            // N=8 (CS21): 16MHz/(8*256) = 7.8kHz
                                            // N=32 (CS21|CS20): 16MHz/(32*256) = 1.95kHz
                                            // N=64 (CS22): 16MHz/(64*256) = 976Hz <-- This is likely what was intended for "around 1kHz"
  TCCR2B = (TCCR2B & 0b11111000) | 0x04; // Set prescaler for Timer2 to 64 (CS22 bit)

  analogWrite(pwmPin, PWM_MIN); // Start with light off
  currentPwmValue = PWM_MIN;
  targetPwmValue = PWM_MIN;

  lastJsonEmitMillis = millis();
  lastMotionDetectMillis = millis();
  lastLightActivityMillis = millis();

  Serial.println("Setup Complete.");
  Serial.println("Hardware Note: Ensure voltage dividers are used for solar and battery pins if their voltage exceeds 5V.");
  Serial.print("Solar Voltage Divider Ratio (assumed): "); Serial.println(SOLAR_VOLTAGE_DIVIDER_RATIO);
  Serial.print("Battery Voltage Divider Ratio (assumed): "); Serial.println(BATTERY_VOLTAGE_DIVIDER_RATIO);
}

// --- Main Loop ---
void loop() {
  readSensors();
  determineSystemState(); // isDark, isCharging

  if (isDarkFlag) {
    handleMotion();
    controlLight();
  } else { // Daytime
    turnOffLightCompletely(); // Ensure light is off during the day
    // Charging is passively managed by external hardware, we just monitor.
  }

  emitJsonData(); // Periodically send data
  handlePowerSaving(); // Check battery and decide on power modes
}

// --- Sensor Reading ---
void readSensors() {
  float solarPinVoltage = analogRead(solarPin) * (ADC_REFERENCE_VOLTAGE / ADC_MAX_VALUE);
  solarVoltage_actual = solarPinVoltage * SOLAR_VOLTAGE_DIVIDER_RATIO;

  float batteryPinVoltage = analogRead(batteryPin) * (ADC_REFERENCE_VOLTAGE / ADC_MAX_VALUE);
  batteryVoltage_actual = batteryPinVoltage * BATTERY_VOLTAGE_DIVIDER_RATIO;

  batteryLevel_percent = mapFloat(batteryVoltage_actual, BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE, 0.0, 100.0);
  batteryLevel_percent = constrain(batteryLevel_percent, 0.0, 100.0);
}

// --- System State Determination ---
void determineSystemState() {
  // Darkness detection
  // Use the voltage AT THE PIN for solarThreshold for simplicity, or adjust solarThreshold based on actual panel voltage
  float solarPinVoltage = solarVoltage_actual / SOLAR_VOLTAGE_DIVIDER_RATIO;
  if (solarPinVoltage < SOLAR_THRESHOLD_VOLTAGE) {
    isDarkFlag = true;
  } else {
    isDarkFlag = false;
  }

  // Charging detection (simplified: if it's daytime and battery isn't full)
  if (!isDarkFlag && batteryVoltage_actual < (BATTERY_MAX_VOLTAGE * BATTERY_CHARGE_TARGET_FRACTION) && solarVoltage_actual > MIN_SOLAR_VOLTAGE_FOR_CHARGING) {
    isChargingFlag = true;
  } else {
    isChargingFlag = false;
  }
}

// --- Motion Handling ---
void handleMotion() {
  int motionSensorValue = digitalRead(motionPin);
  unsigned long currentMillis = millis();

  if (motionSensorValue == HIGH) {
    if (currentMillis - lastMotionDetectMillis > MOTION_DEBOUNCE_MS) {
      if (!motionDetectedFlag) { // New motion detected
         Serial.println("Motion DETECTED");
         motionDetectedFlag = true;
         // Potentially set targetPwmValue to a brighter level here
      }
      lastLightActivityMillis = currentMillis; // Keep track of last motion time
    }
  } else { // motionSensorValue == LOW
    // No need to update lastMotionDetectMillis here, only when HIGH after debounce
    // motionDetectedFlag will be reset by controlLight() after LIGHT_ON_DURATION_AFTER_MOTION_MS
  }
}


// --- Light Control ---
void controlLight() {
  unsigned long currentMillis = millis();

  // Check if battery is too low to operate light
  if (batteryVoltage_actual < BATTERY_PROTECTION_VOLTAGE) {
    targetPwmValue = PWM_MIN;
    motionDetectedFlag = false; // Ensure it doesn't try to turn on again until battery recovers
    Serial.println("Battery critically low, turning off light.");
  }
  // If motion was detected recently, set light to bright
  else if (motionDetectedFlag && (currentMillis - lastLightActivityMillis < LIGHT_ON_DURATION_AFTER_MOTION_MS)) {
    // Set light brightness based on battery level (could be a fixed brightness too)
    targetPwmValue = map(constrain(batteryVoltage_actual, BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE),
                         BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE,
                         PWM_DIM, PWM_MAX); // Brighter if battery is fuller
    // No ramp-up implemented here, sets directly. For ramp: currentPwmValue should smoothly go to targetPwmValue
  }
  // If no motion for a while, or initial state in darkness
  else {
    motionDetectedFlag = false; // Reset flag as duration has passed
    // Dim the light if it was on, or keep it at DIM level if that's desired.
    // For now, just ramp down to DIM or OFF. Let's go to DIM.
    // If you want it to go completely OFF after timeout, set targetPwmValue = PWM_MIN;
    targetPwmValue = PWM_DIM;
    if (currentPwmValue > targetPwmValue) {
        currentPwmValue -= PWM_STEP;
        if (currentPwmValue < targetPwmValue) {
            currentPwmValue = targetPwmValue;
        }
    } else if (currentPwmValue < targetPwmValue) { // If current is less, e.g. was off, go to DIM
        currentPwmValue = targetPwmValue;
    }
  }
  
  // Smooth transition (simplified - only for ramp down for now)
  if (currentPwmValue > targetPwmValue) {
      if (millis() % 50 == 0) { // Slow down the ramp down speed
        currentPwmValue -= PWM_STEP;
        currentPwmValue = max(currentPwmValue, targetPwmValue);
      }
  } else {
      currentPwmValue = targetPwmValue; // Instantly set if target is higher or equal
  }
  
  currentPwmValue = constrain(currentPwmValue, PWM_MIN, PWM_MAX);
  analogWrite(pwmPin, currentPwmValue);
}


// --- Turn Off Light ---
void turnOffLightCompletely() {
  currentPwmValue = PWM_MIN;
  targetPwmValue = PWM_MIN;
  analogWrite(pwmPin, currentPwmValue);
  motionDetectedFlag = false; // Reset motion flag
}

// --- JSON Data Emission ---
void emitJsonData() {
  if (millis() - lastJsonEmitMillis > JSON_EMIT_INTERVAL_MS) {
    Serial.print(F("{"));
    Serial.print(F("\"solarV\":")); Serial.print(solarVoltage_actual, 2);
    Serial.print(F(",\"batteryV\":")); Serial.print(batteryVoltage_actual, 2);
    Serial.print(F(",\"batteryPcnt\":")); Serial.print(batteryLevel_percent, 0);
    Serial.print(F(",\"isDark\":")); Serial.print(isDarkFlag);
    Serial.print(F(",\"isCharging\":")); Serial.print(isChargingFlag);
    Serial.print(F(",\"motion\":")); Serial.print(motionDetectedFlag);
    Serial.print(F(",\"lightPWM\":")); Serial.print(currentPwmValue);
    Serial.println(F("}"));
    lastJsonEmitMillis = millis();
  }
}

// --- Power Saving ---
void handlePowerSaving() {
  // This function is called every loop, so actions should be idempotent or use timers.
  // Example: if battery is very low, prepare for deep sleep.
  if (batteryLevel_percent < 5 && isDarkFlag && !motionDetectedFlag) { // Example: Very low and no activity
    Serial.println("Battery very low, preparing for deep sleep if conditions persist.");
    // Consider adding a timer here: if low for X minutes, then sleep.
    // enterDeepSleep(15); // Sleep for 15 seconds
  } else if (batteryLevel_percent < 20) {
    // Implement low power mode (e.g., reduce polling, further dim light by default)
    // lowPowerMode();
  } else {
    // normalPowerMode();
  }
}

// --- Sleep Functions ---
// Call this function to enter sleep mode. Wakes up via WDT or external interrupt (e.g. motion pin).
void enterDeepSleep(int seconds) {
  Serial.println("Entering deep sleep...");
  Serial.flush(); // Ensure all serial data is sent

  // Configure motion pin as an interrupt to wake up
  // attachInterrupt(digitalPinToInterrupt(motionPin), wakeUpInterruptHandler, RISING); // Or LOW/CHANGE depending on PIR

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  // Setup Watchdog Timer for timed wakeup
  // Max sleep is 8s with WDT. For longer, loop sleep.
  // For 'seconds' > 8, you'd need to loop. This is a simplified example for one WDT cycle.
  byte wdt_prescaler = 0;
  if (seconds >= 8) wdt_prescaler = 0b00100001; // 8 S
  else if (seconds >= 4) wdt_prescaler = 0b00100000; // 4 S
  else if (seconds >= 2) wdt_prescaler = 0b00000111; // 2 S
  else if (seconds >= 1) wdt_prescaler = 0b00000110; // 1 S
  else wdt_prescaler = 0b00000101; // 0.5S, etc. Choose smallest that fits.

  // Disable WDT interrupts, just use it for reset/wakeup.
  WDTCSR = bit(WDCE) | bit(WDE); // Enable configuration change
  WDTCSR = wdt_prescaler;        // Set prescaler
  WDTCSR |= bit(WDE);            // Enable Watchdog System Reset

  // WDTCSR = bit(WDCE) | bit(WDE); // Enable Watchdog Change Enable
  // WDTCSR = bit(WDIE) | bit(WDP3) | bit(WDP0); // Enable WDT Interrupt, ~8s. (For interrupt based wakeup)


  noInterrupts(); // Ensure atomicity for sleep setup
  sleep_bod_disable(); // Disable Brown-out Detector (saves power)
  interrupts();     // Re-enable interrupts for WDT or pin change

  sleep_cpu(); // Enter sleep mode

  // --- CPU WAKES UP HERE ---
  sleep_disable();
  //detachInterrupt(digitalPinToInterrupt(motionPin)); // Important if pin interrupt was used
  Serial.println("Woke up from sleep.");
  // Re-initialize things if needed, WDT reset will restart the sketch.
  // If WDT interrupt was used, execution continues here.
}

void wakeUpInterruptHandler() {
  // This function is called when the motion pin interrupt fires.
  // It's kept short as it's an ISR.
  // sleep_disable(); // Disable sleep mode
  // detachInterrupt(digitalPinToInterrupt(motionPin)); // Detach to avoid re-triggering
  // No Serial.print in ISRs usually
}


// Watchdog Interrupt Service Routine (if WDT interrupt is enabled instead of reset)
ISR(WDT_vect) {
  // Woken up by WDT.
  // If WDE is set, it will reset. If WDIE is set, it comes here.
  // For simplicity, the enterDeepSleep above uses WDE (reset).
  // If using WDIE:
  // wdt_disable(); // Disable watchdog until next sleep cycle
}

// --- Utility Functions ---
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
// Stubs for other power modes if you want to implement them
void lowPowerMode() {
  // Reduce clock speed, disable unused peripherals, etc.
  Serial.println("Entering Low Power Mode (stub)");
}

void mediumPowerMode() {
  Serial.println("Entering Medium Power Mode (stub)");
}

void normalPowerMode() {
  // Restore full operation
  Serial.println("Entering Normal Power Mode (stub)");
}
*/
