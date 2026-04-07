#include <cstdlib>

#include "Arduino.h"
#include "avr/interrupt.h"
#include "EEPROM.h"

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
MockEEPROM EEPROM;

uint8_t mock_eeprom_storage[1024] = {0};
uint8_t MockEEPROM::read(int addr) { return mock_eeprom_storage[addr % 1024]; }
void MockEEPROM::write(int addr, uint8_t val) { mock_eeprom_storage[addr % 1024] = val; }
void MockEEPROM::update(int addr, uint8_t val) { write(addr, val); }

SimSensors sim = { 12.0f, 18.0f, 0.0f, 100.0f, 3.7f, 2.0f, 10.0f, 0.0, 0.0, false, true };

unsigned long current_time_ms = 0;

unsigned long millis() {
    return current_time_ms;
}

void update_sim() {
    float chargeAH = sim.batteryCapAH * (sim.batteryV - 3.0f) / (4.2f - 3.0f);
    unsigned long step_ms = 100;
    current_time_ms += step_ms;

    // Net current (Solar in - System out)
    float netMA = sim.solarCurrentMA - sim.systemCurrentMA;
    float deltaAH = (netMA / 1000.0f) * (step_ms / 3600000.0f);

    // Stats
    if (sim.solarCurrentMA > 0) sim.harvestedMAH += (sim.solarCurrentMA) * (step_ms / 3600000.0);
    sim.consumedMAH += (sim.systemCurrentMA) * (step_ms / 3600000.0);

    chargeAH += deltaAH;
    if (chargeAH < 0) chargeAH = 0;
    if (chargeAH > sim.batteryCapAH) chargeAH = sim.batteryCapAH;

    // Linear Vbat model: 3.0V (0%) to 4.2V (100%)
    sim.batteryV = 3.0f + (chargeAH / sim.batteryCapAH) * (4.2f - 3.0f);
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
    float noise = ((float)rand() / (float)RAND_MAX - 0.5f) * 0.05f; // +/- 25mV noise
    if (pin == A1) { // Battery divider ratio was 3.0. Vpin = Vbat / 3.0
        float v_pin = (sim.batteryV + noise) / 3.0f;
        return (int)(v_pin * 1023.0f / 5.0f);
    }
    if (pin == A0) { // Solar divider ratio was 4.0
        float v_pin = (sim.solarBusV + noise) / 4.0f;
        return (int)(v_pin * 1023.0f / 5.0f);
    }
    return 0;
}

void analogWrite(int pin, int val) {
    if (pin == 3) { // LED
        // Assume LED at 100% takes 500mA
        sim.systemCurrentMA = 10.0f + (val / 255.0f) * 500.0f;
    }
    if (pin == 9) { // MPPT Buck Converter
        // Improved Solar Model: P-V Curve Simulation
        // MPP is usually around 0.8 * Voc. Let's assume Voc=20V, Vmpp=16V.
        // Power curve: P = G * (1 - ((V - Vmpp)/(Voc - Vmpp))^2) * Pmpp
        // For simulation, we control V via PWM duty. Higher duty -> more load -> lower V.
        float duty = (float)val / 255.0f;
        sim.solarBusV = sim.solarOCV * (1.0f - duty * 0.8f); // Duty 1.0 -> V = 4V

        float Vmpp = sim.solarOCV * 0.8f;
        float diff = (sim.solarBusV - Vmpp) / (sim.solarOCV - Vmpp);
        float power_factor = 1.0f - (diff * diff);
        if (power_factor < 0) power_factor = 0;

        float max_p_mw = 50000.0f; // 50W
        float p_mw = power_factor * max_p_mw;

        if (sim.solarBusV > 0.1f) {
            sim.solarCurrentMA = p_mw / sim.solarBusV;
        } else {
            sim.solarCurrentMA = 0;
        }
    }
}

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
