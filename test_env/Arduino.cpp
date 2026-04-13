#include <cstdlib>
#include <iostream>
#include <cmath>
#include <algorithm>

#include "Arduino.h"
#include "avr/interrupt.h"

#define DEFAULT 1
#define INTERNAL 2
#define EXTERNAL 3

#include "EEPROM.h"
#include "Adafruit_INA219.h"
#include "Wire.h"

int TCCR2A = 0;
int TCCR2B = 0;
int TCCR1A = 0;
int TCCR1B = 0;
int ICR1 = 0;
int OCR1A = 0;
int OCR1B = 0;
int WDTCSR = 0;
int MCUSR = 0;
int WDRF = 3;
int EIMSK = 0;
int INT0 = 0;
int EIFR = 0;
int INTF0 = 0;
int ADCSRA = 0;
int ADEN = 7;
int EICRA = 0;
int ISC01 = 1;
int ISC00 = 0;
int WDCE = 4;
int WDE = 3;
int WDIE = 6;
int WDP3 = 5;
int WDP0 = 0;

MockSerial Serial;
MockWire Wire;
MockEEPROM EEPROM;

uint8_t mock_eeprom_storage[1024] = {0};
uint8_t MockEEPROM::read(int addr) { return mock_eeprom_storage[addr % 1024]; }
void MockEEPROM::write(int addr, uint8_t val) { mock_eeprom_storage[addr % 1024] = val; }
void MockEEPROM::update(int addr, uint8_t val) { write(addr, val); }

SimSensors sim = { 12.0f, 18.0f, 0.0f, 100.0f, 0.0f, 3.5f, 1.0f, 10.0f, 3.5f, 0.0, 0.0, 25.0f, 0.2f, true, false, true };
uint8_t current_adc_ref = DEFAULT;

unsigned long current_time_ms = 0;

unsigned long millis() {
    return current_time_ms;
}

void update_sim() {
    float duty = (float)OCR1A / 1023.0f;
    float current_Voc = sim.solarOCV;

    // Non-ideal model for a small PV buck converter
    float R_ds_on = 0.05f;
    float R_inductor = 0.50f;   // Realistic DCR for a small power inductor
    float R_sync = 0.05f;
    float V_diode = 0.70f;      // Vf for a common diode
    float f_sw = 15600.0f;
    float t_tr_tf = 400e-9f;
    float I_q = 0.025f;
    float P_gate = 0.050f;

    float Vbat = sim.batteryV;
    float Isc = sim.solarCurrentMA;

    // In Synchronous mode, low-side FET is complementary to high-side.
    bool active_sync = sim.sync_mode;

    if (duty < 0.01f) {
        sim.solarBusV = current_Voc;
        sim.solarCurrentMA_actual = 0;
    } else {
        float Iout_est = 100.0f;
        for(int i=0; i<60; i++) {
            float Iout_A = Iout_est / 1000.0f;
            if (Iout_A < 0.001f) Iout_A = 0.001f;

            float P_cond_hs = (Iout_A * Iout_A) * (R_ds_on + R_inductor) * duty;
            float P_cond_ls = active_sync ? ((Iout_A * Iout_A) * R_sync * (1.0f - duty)) : (Iout_A * V_diode * (1.0f - duty));
            float P_sw = 0.5f * sim.solarBusV * Iout_A * f_sw * t_tr_tf;
            float total_loss_W = P_cond_hs + P_cond_ls + P_sw + P_gate;

            float target_Vsolar = (Vbat * Iout_A + total_loss_W) / (Iout_A * duty);
            if (target_Vsolar > current_Voc) target_Vsolar = current_Voc;
            if (target_Vsolar < Vbat + 0.1f) target_Vsolar = Vbat + 0.1f;

            float Vt = 2.0f;
            sim.solarCurrentMA_actual = Isc * (1.0f - exp((target_Vsolar - current_Voc) / Vt));
            if (sim.solarCurrentMA_actual < 0) sim.solarCurrentMA_actual = 0;

            sim.solarBusV = target_Vsolar;
            Iout_est = sim.solarCurrentMA_actual / duty;
        }
    }

    float solarOutMA = (duty > 0.01f) ? (sim.solarCurrentMA_actual / duty) : 0;
    float netMA = solarOutMA - (I_q * 1000.0f);

    double step_hours = 0.1 / 3600.0;
    sim.harvestedMAH += (double)solarOutMA * step_hours;
    sim.consumedMAH += (double)sim.systemCurrentMA * step_hours;

    sim.batteryV += (float)(netMA / 1000.0f * (1.0f / sim.batteryCapAH) * step_hours * 100.0f);
    sim.vcc = std::min(5.0f, sim.batteryV);

    sim.solarShuntV = sim.solarCurrentMA_actual * 0.01f;
    current_time_ms += 100;
}

void delay(unsigned long ms) {
    while (ms >= 100) {
        update_sim();
        ms -= 100;
    }
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
    float noise = 0;
    float ref = (current_adc_ref == INTERNAL) ? 1.1f : sim.vcc;

    if (pin == A1) {
        float ratio = (current_adc_ref == INTERNAL) ? 5.54f : 3.0f;
        float v_pin = (sim.batteryV + noise) / ratio;
        return (int)std::max(0.0f, std::min(1023.0f, v_pin * 1023.0f / ref));
    }
    if (pin == A0) {
        float ratio = (current_adc_ref == INTERNAL) ? 31.3f : 4.0f;
        float v_pin = (sim.solarBusV + noise) / ratio;
        return (int)std::max(0.0f, std::min(1023.0f, v_pin * 1023.0f / ref));
    }
    if (pin == 14) {
        return (int)(1.1f * 1023.0f / sim.vcc);
    }
    return 0;
}

void analogWrite(int pin, int val) {
    if (pin == 3) {
        sim.systemCurrentMA = 10.0f + (val / 255.0f) * 500.0f;
    }
}

void set_sleep_mode(int mode) {}
void sleep_enable() {}
void sleep_mode() {}
void sleep_disable() {}
void sleep_bod_disable() {}
void sleep_cpu() {
    for(int i=0; i<80; i++) update_sim();
    if (WDTCSR & (1 << WDIE)) {
        if (WDT_handler) WDT_handler();
    }
}
void wdt_disable() {}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void MockWire::begin() {}
void MockWire::beginTransmission(uint8_t addr) {}
int MockWire::endTransmission() { return sim.ina219_ok ? 0 : 4; }
void MockWire::write(uint8_t val) {}
int MockWire::read() { return 0; }
uint8_t MockWire::requestFrom(uint8_t addr, uint8_t len) { return 0; }

bool Adafruit_INA219::begin() { return sim.ina219_ok; }
float Adafruit_INA219::getBusVoltage_V() { return sim.solarBusV; }
float Adafruit_INA219::getShuntVoltage_mV() { return sim.solarShuntV; }
float Adafruit_INA219::getCurrent_mA() { return sim.solarCurrentMA_actual; }
float Adafruit_INA219::getPower_mW() { return getBusVoltage_V() * getCurrent_mA(); }
void Adafruit_INA219::setCalibration_32V_2A() {}
