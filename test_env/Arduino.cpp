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

SimSensors sim = { 12.0f, 18.0f, 0.0f, 100.0f, 0.0f, 3.6f, 1.0f, 10.0f, 3.6f, 0.0, 0.0, 25.0f, 0.2f, true, false, true, false };
uint8_t current_adc_ref = DEFAULT;

unsigned long current_time_ms = 0;

unsigned long millis() {
    return current_time_ms;
}

void update_sim() {
    float duty = (float)OCR1A / 1023.0f;
    float current_Voc = sim.solarOCV;
    float f_sw = 15600.0f;

    // --- Hardware Parameters ---
    float L0, R_dcr, I_sat, C_pw;
    if (sim.low_spec_inductor) {
        L0 = 22.0e-6f;
        R_dcr = 0.250f;
        I_sat = 1.5f;
        C_pw = 40.0e-12f;
    } else {
        L0 = 22.0e-6f;
        R_dcr = 0.046f;
        I_sat = 5.3f;
        C_pw = 15.0e-12f;
    }

    float R_ds_on = 0.040f;
    float R_sync = 0.040f;
    float V_diode = 0.55f;
    float t_tr_tf = 600e-9f;

    float C_in_esr = 0.050f;
    float V_bat_ocv = 3.60f;
    float R_bat_int = 0.60f;

    float I_q = 0.025f;
    float P_gate = 0.120f;

    float Isc = sim.solarCurrentMA;
    bool active_sync = sim.sync_mode && (OCR1B > OCR1A) && (OCR1B < 1023);

    if (duty < 0.001f) {
        sim.solarBusV = current_Voc;
        sim.solarCurrentMA_actual = 0;
        sim.batteryV = V_bat_ocv;
    } else {
        // Simplified power model for stability
        float Iout_est = 100.0f;
        for(int i=0; i<30; i++) {
            float Iout_A = Iout_est / 1000.0f;
            float P_loss = (Iout_A * Iout_A) * (R_dcr + R_ds_on) * duty;
            P_loss += active_sync ? ((Iout_A * Iout_A) * R_sync * (1.0f-duty)) : (Iout_A * V_diode * (1.0f-duty));
            P_loss += 0.5f * sim.solarBusV * Iout_A * f_sw * t_tr_tf;
            P_loss += P_gate;

            float terminal_Vbat = V_bat_ocv + Iout_A * R_bat_int;
            float target_Vsolar = (terminal_Vbat * Iout_A + P_loss) / (Iout_A * duty + 1e-9f);
            if (target_Vsolar > current_Voc) target_Vsolar = current_Voc;
            if (target_Vsolar < terminal_Vbat) target_Vsolar = terminal_Vbat + 0.01f;

            float Vt = 2.0f;
            float Ipanel_new = Isc * (1.0f - exp((target_Vsolar - current_Voc) / Vt));
            if (Ipanel_new < 0) Ipanel_new = 0;

            sim.solarCurrentMA_actual = Ipanel_new;
            sim.solarBusV = target_Vsolar;
            Iout_est = sim.solarCurrentMA_actual / duty;
        }
        sim.batteryV = V_bat_ocv + (Iout_est / 1000.0f) * R_bat_int;
    }

    float solarOutMA = (duty > 0.02f) ? (sim.solarCurrentMA_actual / duty) : 0;
    sim.harvestedMAH += (double)solarOutMA * (0.1 / 3600.0);
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
        float v_pin = sim.batteryV / ratio;
        return (int)std::max(0.0f, std::min(1023.0f, v_pin * 1023.0f / ref));
    }
    if (pin == A0) {
        float ratio = (current_adc_ref == INTERNAL) ? 31.3f : 4.0f;
        float v_pin = sim.solarBusV / ratio;
        return (int)std::max(0.0f, std::min(1023.0f, v_pin * 1023.0f / ref));
    }
    if (pin == 14) return (int)(1.1f * 1023.0f / sim.vcc);
    return 0;
}

void analogWrite(int pin, int val) {
    if (pin == 3) sim.systemCurrentMA = 10.0f + (val / 255.0f) * 500.0f;
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
