
// Define some constants for pins, parameters and formulas
#define SOLAR_PIN A0 // Analog pin for solar panel voltage
#define SOLAR_CURRENT_PIN A1 // Analog pin for solar panel current
#define BATTERY_PIN A2 // Analog pin for battery voltage
#define BATTERY_CURRENT_PIN A3 // Analog pin for battery current
#define PWM_PIN 3 // PWM pin for MPPT control
#define BATTERY_PWM_PIN 5 // PWM pin for battery protection control
#define MOTION_PIN 2 // Digital pin for motion sensor output
#define LED_PIN 6 // PWM pin for LED control
#define USB_PIN 7 // Digital pin for USB port control

#define R1 10000 // Resistor value for voltage divider (ohms)
#define R2 1000 // Resistor value for voltage divider (ohms)
#define R_SHUNT 0.01 // Shunt resistor value for current measurement (ohms)
#define V_REF 5 // Reference voltage for analog pins (volts)
#define V_MAX 12.6 // Maximum voltage for battery (volts)
#define V_MIN 9 // Minimum voltage for battery (volts)
#define I_MAX 5 // Maximum current for battery (amps)
#define P_MAX 50 // Maximum power for solar panel (watts)

// Define some variables for storing measurements and calculations
float solar_voltage = 0; // Solar panel voltage (volts)
float solar_current = 0; // Solar panel current (amps)
float solar_power = 0; // Solar panel power (watts)
float battery_voltage = 0; // Battery voltage (volts)
float battery_current = 0; // Battery current (amps)
float battery_soc = 0; // Battery state of charge (%)
int pwm_duty = 0; // PWM duty cycle for MPPT control (%)
int battery_pwm_duty = 0; // PWM duty cycle for battery protection control (%)
bool motion_state = false; // Motion sensor state (true or false)
int led_duty = 0; // PWM duty cycle for LED control (%)
bool usb_state = false; // USB port state (true or false)

// Define some parameters for MPPT algorithm
#define DELTA_DUTY 1 // Increment or decrement of PWM duty cycle (%)
#define DELTA_POWER 0.1 // Threshold for power change detection (watts)

// Define some parameters for motion detection algorithm
#define DELAY_TIME 30 // Time to keep the LED on after motion detected (seconds)
#define BRIGHTNESS_LEVEL 255 // Maximum brightness level for LED (0-255)
#define PRESENCE_INTENSITY 5 // Number of consecutive motion detections to increase brightness level

// Define some parameters for USB port control
#define USB_THRESHOLD 80 // Minimum battery SOC to enable USB port (%)

// Initialize the serial port for communication with Wi-Fi module
Serial.begin(9600);

// Initialize the pins as input or output
pinMode(SOLAR_PIN, INPUT);
pinMode(SOLAR_CURRENT_PIN, INPUT);
pinMode(BATTERY_PIN, INPUT);
pinMode(BATTERY_CURRENT_PIN, INPUT);
pinMode(PWM_PIN, OUTPUT);
pinMode(BATTERY_PWM_PIN, OUTPUT);
pinMode(MOTION_PIN, INPUT);
pinMode(LED_PIN, OUTPUT);
pinMode(USB_PIN, OUTPUT);

// Set the initial states of the pins
digitalWrite(PWM_PIN, LOW);
digitalWrite(BATTERY_PWM_PIN, LOW);
digitalWrite(LED_PIN, LOW);
digitalWrite(USB_PIN, LOW);

void loop() {
  // Measure the solar panel voltage and current
  solar_voltage = analogRead(SOLAR_PIN) * V_REF / 1024 * (R1 + R2) / R2;
  solar_current = analogRead(SOLAR_CURRENT_PIN) * V_REF / 1024 / R_SHUNT;

  // Calculate the solar panel power
  solar_power = solar_voltage * solar_current;

  // Measure the battery voltage and current
  battery_voltage = analogRead(BATTERY_PIN) * V_REF / 1024 * (R1 + R2) / R2;
  battery_current = analogRead(BATTERY_CURRENT_PIN) * V_REF / 1024 / R_SHUNT;

  // Calculate the battery state of charge
  battery_soc = (battery_voltage - V_MIN) / (V_MAX - V_MIN) * 100;

  // Implement the MPPT algorithm
  mppt();

  // Implement the battery protection algorithm
  battery_protection();

  // Read the motion sensor state
  motion_state = digitalRead(MOTION_PIN);

  // Implement the motion detection algorithm
  motion_detection();

  // Implement the USB port control algorithm
  usb_control();

  // Send data in JSON format to Wi-Fi module
  send_data();

}

// Function to implement the MPPT algorithm
void mppt() {
  // Declare some static variables to store previous values
  static float prev_power = 0; // Previous solar panel power (watts)
  static int prev_duty = 0; // Previous PWM duty cycle (%)
  static int direction = 1; // Direction of PWM duty cycle change (+1 or -1)

  // Compare the current power with the previous power
  if (solar_power > prev_power + DELTA_POWER) {
    // If the power increased, keep the same direction and increment the duty cycle
    pwm_duty = prev_duty + direction * DELTA_DUTY;
  } else if (solar_power < prev_power - DELTA_POWER) {
    // If the power decreased, reverse the direction and decrement the duty cycle
    pwm_duty = prev_duty - direction * DELTA_DUTY;
    direction = -direction;
  } else {
    // If the power did not change significantly, keep the same duty cycle
    pwm_duty = prev_duty;
  }

  // Constrain the duty cycle between 0 and 100
  pwm_duty = constrain(pwm_duty, 0, 100);

  // Set the PWM output to the calculated duty cycle
  analogWrite(PWM_PIN, pwm_duty * 255 / 100);

  // Update the previous power and duty cycle values
  prev_power = solar_power;
  prev_duty = pwm_duty;
}

// Function to implement the battery protection algorithm
void battery_protection() {
  // Declare some static variables to store previous values
  static bool charging_state = false; // Charging state of the battery (true or false)
  static bool discharging_state = false; // Discharging state of the battery (true or false)

  // Check if the battery voltage is above the maximum voltage
  if (battery_voltage > V_MAX) {
    // If yes, stop charging the battery by turning off the MOSFET switch
    battery_pwm_duty = 0;
    analogWrite(BATTERY_PWM_PIN, battery_pwm_duty);
    charging_state = false;
  } else if (battery_voltage < V_MAX - 0.1 && !charging_state) {
    // If no, and the battery is not already charging, start charging the battery by turning on the MOSFET switch
    battery_pwm_duty = 100;
    analogWrite(BATTERY_PWM_PIN, battery_pwm_duty);
    charging_state = true;
  }

  // Check if the battery voltage is below the minimum voltage
  if (battery_voltage < V_MIN) {
    // If yes, stop discharging the battery by turning off the LED light
    led_duty = 0;
    analogWrite(LED_PIN, led_duty);
    discharging_state = false;
  } else if (battery_voltage > V_MIN + 0.1 && !discharging_state) {
    // If no, and the battery is not already discharging, start discharging the battery by turning on the LED light
    led_duty = BRIGHTNESS_LEVEL;
    analogWrite(LED_PIN, led_duty);
    discharging_state = true;
  }
}

// Function to implement the motion detection algorithm
void motion_detection() {
  // Declare some static variables to store previous values
  static unsigned long prev_time = 0; // Previous time when motion was detected (milliseconds)
  static int presence_count = 0; // Number of consecutive motion detections

  // Check if the motion sensor state is true
  if (motion_state) {
    // If yes, update the previous time and increment the presence count
    prev_time = millis();
    presence_count++;
  } else {
    // If no, check if the current time is more than the delay time after the previous time
    if (millis() - prev_time > DELAY_TIME * 1000) {
      // If yes, reset the presence count
      presence_count = 0;
    }
  }

  // Constrain the presence count between 0 and PRESENCE_INTENSITY
  presence_count = constrain(presence_count, 0, PRESENCE_INTENSITY);

  // Calculate the LED duty cycle based on the presence count
  led_duty = map(presence_count, 0, PRESENCE_INTENSITY, 0, BRIGHTNESS_LEVEL);

  // Set the PWM output to the calculated duty cycle
  analogWrite(LED_PIN, led_duty);
}

// Function to implement the USB port control algorithm
void usb_control() {
  // Check if the battery state of charge is above the USB threshold
  if (battery_soc > USB_THRESHOLD) {
    // If yes, enable the USB port by turning on the MOSFET switch
    usb_state = true;
    digitalWrite(USB_PIN, usb_state);
  } else {
    // If no, disable the USB port by turning off the MOSFET switch
    usb_state = false;
    digitalWrite(USB_PIN, usb_state);
  }
}

// Function to send data in JSON format to Wi-Fi module
void send_data() {
  // Declare a static variable to store the previous time
  static unsigned long prev_time = 0; // Previous time when data was sent (milliseconds)

  // Check if the current time is more than 10 minutes after the previous time
  if (millis() - prev_time > 10 * 60 * 1000) {
    // If yes, update the previous time and create a JSON object
    prev_time = millis();
    String json = "{";
    json += "\"solar_voltage\":" + String(solar_voltage) + ",";
    json += "\"solar_current\":" + String(solar_current) + ",";
    json += "\"solar_power\":" + String(solar_power) + ",";
    json += "\"battery_voltage\":" + String(battery_voltage) + ",";
    json += "\"battery_current\":" + String(battery_current) + ",";
    json += "\"battery_soc\":" + String(battery_soc) + ",";
    json += "\"motion_state\":" + String(motion_state) + ",";
    json += "\"led_duty\":" + String(led_duty) + ",";
    json += "\"usb_state\":" + String(usb_state);
    json += "}";

    // Send the JSON object to the serial port
    Serial.println(json);
  }
}
