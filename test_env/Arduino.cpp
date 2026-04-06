#include "Arduino.h"
#include "avr/interrupt.h"

int TCCR2B = 0;
int WDTCSR = 0;
int MCUSR = 0;
int WDRF = 0;
int EIMSK = 0;
int INT0 = 0;
int EIFR = 0;
int INTF0 = 0;
int ADCSRA = 0;
int ADEN = 0;
int EICRA = 0;
int ISC01 = 0;
int ISC00 = 0;
int WDCE = 0;
int WDE = 0;
int WDIE = 0;
int WDP3 = 0;
int WDP0 = 0;

MockSerial Serial;
MockWire Wire;

unsigned long millis() {
    static auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
}

void delay(unsigned long ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void pinMode(int pin, int mode) {}
void digitalWrite(int pin, int val) {}
int digitalRead(int pin) { return LOW; }
int analogRead(int pin) { return 512; }
void analogWrite(int pin, int val) {}

void set_sleep_mode(int mode) {}
void sleep_enable() {}
void sleep_mode() {}
void sleep_disable() {}
void sleep_bod_disable() {}
void sleep_cpu() {}
void wdt_disable() {}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
