# YASL
Yet Another Solar Lamp
Created by BingAI, Consolidated

## Project Overview
YASL is a solar-powered motion sensor lamp project. This repository contains various iterations of the firmware, ranging from basic implementations to advanced versions with MPPT charging and sophisticated sleep modes.

## Consolidated Version: `YASL_Consolidated.ino`
The `YASL_Consolidated.ino` file represents the most stable and feature-rich version of the firmware, combining the best ideas from all previous iterations.

### Key Features:
- **MPPT Charging**: Perturb & Observe (P&O) algorithm to maximize solar panel efficiency.
- **CC/CV Logic**: Constant Current and Constant Voltage charging phases for battery safety and longevity.
- **Robust Sensors**: Averaged ADC readings for battery and solar monitoring to reduce noise.
- **Motion Detection**: PIR sensor integration with smooth PWM dimming transitions.
- **Advanced Power Management**:
  - Periodic Watchdog Timer (WDT) sleep for system checks.
  - External Interrupt (INT0) for immediate wake-up upon motion detection.
  - Peripheral shutdown (ADC, etc.) during sleep.
- **JSON Telemetry**: Serial output in JSON format for easy integration with Wi-Fi modules or data loggers.

## Testing
The project includes a mock Arduino environment in `test_env/` to verify compilation and basic loop execution on non-Arduino systems.

### Running Tests
To test the files, use the provided Python harness:
```bash
python3 test_env/test_harness.py
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
