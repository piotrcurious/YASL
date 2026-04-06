
#include "Arduino.h"
#include "avr/interrupt.h"

int TCCR2B = 0;
int WDTCSR = 0;
int MCUSR = 0;
int WDRF = 3; // ATmega328P bit 3
int EIMSK = 0;
int INT0 = 0; // Bit 0
int EIFR = 0;
int INTF0 = 0; // Bit 0
int ADCSRA = 0;
int ADEN = 7; // Bit 7
int EICRA = 0;
int ISC01 = 1; // Bit 1
int ISC00 = 0; // Bit 0
int WDCE = 4;  // Bit 4
int WDE = 3;   // Bit 3
int WDIE = 6;  // Bit 6
int WDP3 = 5;  // Bit 5
int WDP0 = 0;  // Bit 0

MockSerial Serial;
MockWire Wire;

SimSensors sim = { 12.0f, 0.0f, 100.0f, 3.7f, false, true };

unsigned long current_time_ms = 0;

unsigned long millis() {
    return current_time_ms;
}

void update_sim() {
    current_time_ms += 100; // Step 100ms
}

void delay(unsigned long ms) {
    current_time_ms += ms;
}

void pinMode(int pin, int mode) {}
void digitalWrite(int pin, int val) {}

int digitalRead(int pin) {
    if (pin == 2) return sim.motion ? HIGH : LOW;
    return LOW;
}

int analogRead(int pin) {
    if (pin == A1) { // Battery divider ratio was 3.0. Vpin = Vbat / 3.0
        float v_pin = sim.batteryV / 3.0f;
        return (int)(v_pin * 1023.0f / 5.0f);
    }
    if (pin == A0) { // Solar divider ratio was 4.0
        float v_pin = sim.solarBusV / 4.0f;
        return (int)(v_pin * 1023.0f / 5.0f);
    }
    return 0;
}

void analogWrite(int pin, int val) {}

void set_sleep_mode(int mode) {}
void sleep_enable() {}
void sleep_mode() {}
void sleep_disable() {}
void sleep_bod_disable() {}
void sleep_cpu() {
    std::cout << "[SIM] SLEEP_CPU called" << std::endl;
    // Advance time and potentially trigger WDT ISR
    current_time_ms += 8000; // Simulate one WDT cycle
    if (WDTCSR & (1 << WDIE)) {
        if (WDT_handler) {
            std::cout << "[SIM] Triggering WDT_vect" << std::endl;
            WDT_handler();
        } else {
             std::cout << "[SIM] WDT_handler not defined" << std::endl;
        }
    }
}
void wdt_disable() {}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
