#include <Wire.h>
#include <Adafruit_INA219.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

// --- INA219 for Solar Current Measurement ---
Adafruit_INA219 ina219;

// --- Pin Definitions ---
const int solarPin = A0;
const int batteryPin = A1;
const int pwmPin = 3;
const int motionPin = 2;
const int mpptPwmPin = 5; // NEW: PWM pin for MPPT control (buck converter control)

// --- Voltage Divider Ratios ---
const float SOLAR_VOLTAGE_DIVIDER_RATIO = 4.0; // Adjust as per your circuit
const float BATTERY_VOLTAGE_DIVIDER_RATIO = 3.0;

// --- System Parameters ---
const float ADC_REFERENCE_VOLTAGE = 5.0;
const float ADC_MAX_VALUE = 1023.0;
const float SOLAR_THRESHOLD_VOLTAGE = 1.0;
const float BATTERY_MAX_VOLTAGE = 4.2;
const float BATTERY_MIN_VOLTAGE = 3.0;
const float BATTERY_PROTECTION_VOLTAGE = 3.2;
const float BATTERY_CHARGE_TARGET_FRACTION = 0.90;
const float MIN_SOLAR_VOLTAGE_FOR_CHARGING = 3.0;

// --- PWM Values ---
const int PWM_MAX = 255;
const int PWM_MIN = 0;
const int PWM_DIM = 30;
const int PWM_STEP = 5;

const int MPPT_PWM_MAX = 255;
const int MPPT_PWM_MIN = 0;
int mpptDuty = 128; // Initial duty for MPPT

// --- Timers ---
const long JSON_EMIT_INTERVAL_MS = 60000;
const long MOTION_DEBOUNCE_MS = 200;
const long LIGHT_ON_DURATION_AFTER_MOTION_MS = 15000;

// --- State Variables ---
float solarVoltage_actual, batteryVoltage_actual, batteryLevel_percent;
float solarCurrent_mA = 0.0;
float solarPower_mW = 0.0;
float lastPower = 0.0;

int currentPwmValue = 0;
int targetPwmValue = 0;
bool motionDetectedFlag = false;
bool isDarkFlag = false;
bool isChargingFlag = false;

unsigned long lastJsonEmitMillis = 0;
unsigned long lastMotionDetectMillis = 0;
unsigned long lastLightActivityMillis = 0;
unsigned long lastMpptMillis = 0;

// --- Setup ---
void setup() {
  Serial.begin(9600);
  Wire.begin();
  ina219.begin();

  pinMode(solarPin, INPUT);
  pinMode(batteryPin, INPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(mpptPwmPin, OUTPUT);
  pinMode(motionPin, INPUT);

  analogWrite(pwmPin, PWM_MIN);
  analogWrite(mpptPwmPin, mpptDuty);

  currentPwmValue = PWM_MIN;
  targetPwmValue = PWM_MIN;

  Serial.println("System Initialized with INA219 and MPPT.");
}

// --- Loop ---
void loop() {
  readSensors();
  determineSystemState();

  if (isDarkFlag) {
    handleMotion();
    controlLight();
  } else {
    turnOffLightCompletely();
  }

  mpptTracking();
  emitJsonData();
  handlePowerSaving();
}

// --- Sensor Reading ---
void readSensors() {
  float solarPinVoltage = analogRead(solarPin) * (ADC_REFERENCE_VOLTAGE / ADC_MAX_VALUE);
  solarVoltage_actual = solarPinVoltage * SOLAR_VOLTAGE_DIVIDER_RATIO;

  float batteryPinVoltage = analogRead(batteryPin) * (ADC_REFERENCE_VOLTAGE / ADC_MAX_VALUE);
  batteryVoltage_actual = batteryPinVoltage * BATTERY_VOLTAGE_DIVIDER_RATIO;

  batteryLevel_percent = mapFloat(batteryVoltage_actual, BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE, 0.0, 100.0);
  batteryLevel_percent = constrain(batteryLevel_percent, 0.0, 100.0);

  // INA219 Measurements
  solarCurrent_mA = ina219.getCurrent_mA();
  float shuntVoltage_mV = ina219.getShuntVoltage_mV();
  float busVoltage = ina219.getBusVoltage_V();
  solarPower_mW = busVoltage * solarCurrent_mA;
}

// --- System State ---
void determineSystemState() {
  float solarPinVoltage = solarVoltage_actual / SOLAR_VOLTAGE_DIVIDER_RATIO;
  isDarkFlag = (solarPinVoltage < SOLAR_THRESHOLD_VOLTAGE);

  isChargingFlag = (!isDarkFlag &&
                    batteryVoltage_actual < (BATTERY_MAX_VOLTAGE * BATTERY_CHARGE_TARGET_FRACTION) &&
                    solarVoltage_actual > MIN_SOLAR_VOLTAGE_FOR_CHARGING);
}

// --- MPPT (Perturb and Observe) ---
void mpptTracking() {
  static unsigned long mpptInterval = 1000; // 1 second intervals

  if (millis() - lastMpptMillis > mpptInterval) {
    lastMpptMillis = millis();

    float deltaPower = solarPower_mW - lastPower;
    if (deltaPower > 0) {
      mpptDuty = constrain(mpptDuty + 1, MPPT_PWM_MIN, MPPT_PWM_MAX);
    } else {
      mpptDuty = constrain(mpptDuty - 1, MPPT_PWM_MIN, MPPT_PWM_MAX);
    }
    analogWrite(mpptPwmPin, mpptDuty);
    lastPower = solarPower_mW;
  }
}

// --- Motion ---
void handleMotion() {
  int motionSensorValue = digitalRead(motionPin);
  unsigned long currentMillis = millis();

  if (motionSensorValue == HIGH && currentMillis - lastMotionDetectMillis > MOTION_DEBOUNCE_MS) {
    if (!motionDetectedFlag) {
      Serial.println("Motion DETECTED");
      motionDetectedFlag = true;
    }
    lastLightActivityMillis = currentMillis;
  }
}

// --- Light Control ---
void controlLight() {
  unsigned long currentMillis = millis();

  if (batteryVoltage_actual < BATTERY_PROTECTION_VOLTAGE) {
    targetPwmValue = PWM_MIN;
    motionDetectedFlag = false;
    Serial.println("Battery critically low. Light OFF.");
  } else if (motionDetectedFlag && currentMillis - lastLightActivityMillis < LIGHT_ON_DURATION_AFTER_MOTION_MS) {
    targetPwmValue = map(constrain(batteryVoltage_actual, BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE),
                         BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE,
                         PWM_DIM, PWM_MAX);
  } else {
    motionDetectedFlag = false;
    targetPwmValue = PWM_DIM;
  }

  // Ramp down smoothly
  if (currentPwmValue > targetPwmValue) {
    if (millis() % 50 == 0) {
      currentPwmValue -= PWM_STEP;
      currentPwmValue = max(currentPwmValue, targetPwmValue);
    }
  } else {
    currentPwmValue = targetPwmValue;
  }

  currentPwmValue = constrain(currentPwmValue, PWM_MIN, PWM_MAX);
  analogWrite(pwmPin, currentPwmValue);
}

void turnOffLightCompletely() {
  currentPwmValue = PWM_MIN;
  targetPwmValue = PWM_MIN;
  analogWrite(pwmPin, currentPwmValue);
  motionDetectedFlag = false;
}

// --- Data Emit ---
void emitJsonData() {
  if (millis() - lastJsonEmitMillis > JSON_EMIT_INTERVAL_MS) {
    lastJsonEmitMillis = millis();
    Serial.print(F("{\"solarV\":")); Serial.print(solarVoltage_actual, 2);
    Serial.print(F(",\"solarI\":")); Serial.print(solarCurrent_mA, 2);
    Serial.print(F(",\"solarP\":")); Serial.print(solarPower_mW, 2);
    Serial.print(F(",\"batteryV\":")); Serial.print(batteryVoltage_actual, 2);
    Serial.print(F(",\"batteryPcnt\":")); Serial.print(batteryLevel_percent, 0);
    Serial.print(F(",\"isDark\":")); Serial.print(isDarkFlag);
    Serial.print(F(",\"isCharging\":")); Serial.print(isChargingFlag);
    Serial.print(F(",\"mpptDuty\":")); Serial.print(mpptDuty);
    Serial.println(F("}"));
  }
}

// --- Utility ---
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void handlePowerSaving() {
  // Sleep modes could be added here.
}
