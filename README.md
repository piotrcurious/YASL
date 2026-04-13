# YASL
Yet Another Solar Lamp
Created by BingAI, Consolidated

## Project Overview
YASL is a solar-powered motion sensor lamp project. This repository contains various iterations of the firmware, ranging from basic implementations to advanced versions with MPPT charging and sophisticated sleep modes.

## Consolidated Version: `YASL_Consolidated.ino` (v2.4)
The `YASL_Consolidated.ino` file represents the most stable and feature-rich version of the firmware, combining the best ideas from all previous iterations.

### Key Features:
- **Synchronous Switching**: High-efficiency buck conversion using complementary FET driving with hardware dead-time protection.
- **SMC MPPT**: Advanced Sliding Mode Control algorithm for fast and stable Maximum Power Point Tracking.
- **Dynamic Vref Compensation**: Per-cycle supply voltage measurement via internal 1.1V bandgap for accurate operation when powered directly from a battery.
- **Low-Light Optimized**: Dynamic startup thresholds and predictive MPPT "Smart Kick" for maximum overcast harvesting.
- **3-Stage Charging**: Fully implemented Bulk, Absorption (CV), and Float stages for battery longevity.
- **Sensorless Mode**: Physically-derived solar current inference model for operation without an INA219 sensor.
- **Robust Sensors**: Averaged ADC readings and non-blocking I2C recovery logic.
- **Motion Detection**: PIR sensor integration with smooth PWM dimming transitions.
- **Advanced Power Management**:
  - Periodic Watchdog Timer (WDT) sleep for system checks.
  - External Interrupt (INT0) for immediate wake-up upon motion detection.
  - Peripheral shutdown (ADC, etc.) during sleep.
- **JSON Telemetry**: Serial output in JSON format for easy integration with Wi-Fi modules or data loggers.

## Testing
The project includes a mock Arduino environment in `test_env/` to verify compilation and basic loop execution on non-Arduino systems.

### Running Tests
The test environment requires `g++` to be installed on your system. To test the files, use the provided Python scripts:

**Full Validation Suite:**
```bash
python3 test_env/test_harness.py        # Compilation check
python3 test_env/unit_tests.py          # Math logic validation
python3 test_env/simulate_scenario.py   # Full day/night cycle simulation
python3 test_env/stress_test.py         # Solar fluctuation stress
python3 test_env/test_vcc_dependency.py # Voltage reference accuracy test
python3 test_env/test_synchronous.py    # PWM timing and dead-time verification
```

## Disclaimer
The code in this repository is experimental. While efforts have been made to consolidate best practices, you should test and debug it yourself on your specific hardware.

First, let me explain the basic idea of how to create Arduino code for a solar powered motion sensor lamp. You will need the following components:

- A solar panel that can provide enough power for your LED light and charge your battery
- A battery that can store the energy from the solar panel and power your LED light when needed
- A charge controller that can regulate the charging and discharging of the battery and prevent overcharging or overdischarging
- A motion sensor that can detect the presence of humans or animals and trigger the LED light
- A LED light that can provide enough illumination for your desired area
- An Arduino board that can control the logic and communication of the system
- Some resistors, capacitors, transistors, diodes, wires and connectors to connect everything together

The basic logic of the code is as follows:

- Measure the voltage and current of the solar panel using analog pins of the Arduino
- Calculate the power output of the solar panel using Ohm's law (P = V * I)
- Use a PWM pin of the Arduino to control a MOSFET transistor that acts as a switch between the solar panel and the battery
- Implement a MPPT algorithm that adjusts the duty cycle of the PWM signal to maximize the power transfer from the solar panel to the battery
- Measure the voltage and current of the battery using analog pins of the Arduino
- Calculate the state of charge (SOC) of the battery using a simple formula (SOC = V / Vmax * 100%)
- Use another PWM pin of the Arduino to control another MOSFET transistor that acts as a switch between the battery and the LED light
- Implement a battery protection algorithm that prevents overcharging or overdischarging of the battery by turning on or off the MOSFET switch
- Read the output of the motion sensor using a digital pin of the Arduino
- Implement a motion detection algorithm that turns on or off the LED light based on the motion sensor output and some predefined parameters (such as delay time, brightness level, etc.)
- Use a serial port of the Arduino to send data in JSON format to a Wi-Fi module (such as ESP8266) that can connect to your phone app or web server
- Use another digital pin of the Arduino to control a USB port that can charge your phone or other devices

Here is some sample code that illustrates some parts of this logic. It is not complete or tested, so you will need to modify it according to your needs and specifications. I have added some comments to explain what each part does.


Source: Conversation with Bing, 5/1/2023
(1) DIY Solar Motion Sensor Security Light - Instructables. https://www.instructables.com/DIY-Solar-Motion-Sensor-LED-Light/.
(2) Arduino - Motion Sensor | Arduino Tutorial - Arduino Getting Started. https://arduinogetstarted.com/tutorials/arduino-motion-sensor.
(3) The Solar Lamp Works as Needed | Arduino Project Hub. https://projecthub.arduino.cc/cvzeljko/f6e4b395-9265-44b5-864a-93efce5bc547.
(4) The Perfect Automatic Lighting System Using Arduino + LDR + PIR. https://www.instructables.com/Ultimate-Automatic-Lighting-System-Using-Arduino-L/.
(5) 1kW Arduino MPPT Solar Charge Controller (ESP32 + WiFi). https://www.instructables.com/DIY-1kW-MPPT-Solar-Charge-Controller/.
(6) ARDUINO MPPT SOLAR CHARGE CONTROLLER (Version-3.0). https://www.instructables.com/ARDUINO-SOLAR-CHARGE-CONTROLLER-Version-30/.
(7) Arduino Powered Solar Battery Charger : 7 Steps - Instructables. https://www.instructables.com/Arduino-powered-Solar-Battery-Charger/.
