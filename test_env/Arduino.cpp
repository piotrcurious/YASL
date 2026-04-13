#include <cstdlib>
#include <iostream>
#include <cmath>

#include "Arduino.h"
#include "avr/interrupt.h"

#define DEFAULT 1
#define INTERNAL 2
#define EXTERNAL 3

#include "EEPROM.h"

int TCCR2A = 0;
int TCCR2B = 0;
int TCCR1A = 0;
int TCCR1B = 0;
int ICR1 = 0;
int OCR1A = 0;
int OCR1B = 0;
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

SimSensors sim = { 12.0f, 18.0f, 0.0f, 100.0f, 3.5f, 1.0f, 10.0f, 3.5f, 0.0, 0.0, 25.0f, 0.05f, true, false, true };
uint8_t current_adc_ref = DEFAULT;

unsigned long current_time_ms = 0;

unsigned long millis() {
    return current_time_ms;
}



























void update_sim() {
    float duty = (float)OCR1A / 1023.0f;
    float R_conv = 0.05f, R_sync = 0.02f, V_diode = 0.5f, Vbat = sim.batteryV;
    float Isc = sim.solarCurrentMA;
    if (duty < 0.001f) { sim.solarBusV = sim.solarOCV; sim.solarCurrentMA_actual = 0; }
    else {
        float Iout_est = 100.0f;
        for(int i=0; i<30; i++) {
            float V_drop_idle = sim.sync_mode ? ((Iout_est/1000.0f) * R_sync * (1.0f - duty)) : (V_diode * (1.0f - duty));
            float V_drop_active = (Iout_est/1000.0f) * R_conv * duty;
            sim.solarBusV = (Vbat + V_drop_idle + V_drop_active) / (duty > 0.001 ? duty : 0.001);
            if (sim.solarBusV > sim.solarOCV) sim.solarBusV = sim.solarOCV;
            sim.solarCurrentMA_actual = Isc * (1.0f - exp(sim.solarBusV - sim.solarOCV));
            if (sim.solarCurrentMA_actual < 0) sim.solarCurrentMA_actual = 0;
            Iout_est = sim.solarCurrentMA_actual / (duty > 0.001 ? duty : 0.001);
        }
    }
    float solarOutMA = (duty > 0.01f) ? (sim.solarCurrentMA_actual / (duty > 0.001 ? duty : 0.001)) : 0;
    sim.batteryV += ((solarOutMA - 15.0f) / 1000.0f) * (100.0f / 3600000.0f);
    sim.vcc = sim.batteryV; sim.solarShuntV = sim.solarCurrentMA_actual * 0.01;
    current_time_ms += 100;
}

void delay(unsigned long ms) {
    current_time_ms += ms;
}

void analogReference(uint8_t mode) {
    current_adc_ref = mode;
}

void pinMode(int pin, int mode) {}
void digitalWrite(int pin, int val) {}

int digitalRead(int pin) {
    if (pin == 2) return sim.motion ? HIGH : LOW;
    return LOW;
}

int analogRead(int pin) {
    float noise = ((float)rand() / (float)RAND_MAX - 0.5f) * 0.05f; // +/- 25mV noise
    float ref = (current_adc_ref == INTERNAL) ? 1.1f : sim.vcc;

    if (pin == A1) {
        float ratio = (current_adc_ref == INTERNAL) ? 5.54f : 3.0f;
        float v_pin = (sim.batteryV + noise) / ratio;
        return (int)(v_pin * 1023.0f / ref);
    }
    if (pin == A0) {
        float ratio = (current_adc_ref == INTERNAL) ? 31.3f : 4.0f;
        float v_pin = (sim.solarBusV + noise) / ratio;
        return (int)(v_pin * 1023.0f / ref);
    }
    if (pin == 14) { // Mocking 1.1V Bandgap measurement (channel 0x0E = 14)
        // This is only valid in DEFAULT ref mode (measuring bandgap against Vcc)
        // Formula: ADC = 1.1 * 1023 / Vcc
        return (int)(1.1f * 1023.0f / sim.vcc);
    }
    return 0;
}

void analogWrite(int pin, int val) {
    if (pin == 3) { // LED
        // Assume LED at 100% takes 500mA
        sim.systemCurrentMA = 10.0f + (val / 255.0f) * 500.0f;
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

void MockWire::beginTransmission(uint8_t addr) {
}

int MockWire::endTransmission() {
    return sim.ina219_ok ? 0 : 4; // 4 = other error
}

void MockWire::write(uint8_t val) {
}
