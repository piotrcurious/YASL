#include <cstdlib>
#include <iostream>
#include <cmath>
#include <algorithm>

#include "Arduino.h"
#include "EEPROM.h"
#include "Wire.h"
#include "ESP8266WiFi.h"
#include "Adafruit_INA219.h"

int TCCR2A=0, TCCR2B=0, TCCR1A=0, TCCR1B=0, ICR1=1023, OCR1A=0, OCR1B=0, WDTCSR=0, MCUSR=0, WDRF=0, ADCSRA=0, ADEN=0, EICRA=0, ISC01=0, ISC00=0, WDCE=0, WDE=0, WDIE=0, WDP3=0, WDP0=0, EIMSK=0, INT0=0, EIFR=0, INTF0=0;

MockSerial Serial;
MockWire Wire;
MockEEPROM EEPROM;
MockWiFiClass WiFi;

uint8_t mock_eeprom_storage[1024];
uint8_t MockEEPROM::read(int addr) { return mock_eeprom_storage[addr % 1024]; }
void MockEEPROM::write(int addr, uint8_t val) { mock_eeprom_storage[addr % 1024] = val; }
void MockEEPROM::update(int addr, uint8_t val) { write(addr, val); }
void MockEEPROM::commit() {}

SimSensors sim = { 12.0f, 18.0f, 0.0f, 100.0f, 0.0f, 12.0f, 12.0f, 0.1f, 10.0f, 100.0f, 5.0f, 0.0, 0.0, 25.0f, 0.2f, true, false, true, false, 15600.0f };
uint8_t current_adc_ref = 1;
unsigned long current_time_ms = 0;

unsigned long millis() { return current_time_ms; }

void update_sim() {
    float duty = (ICR1 > 0) ? ((float)OCR1A / (float)ICR1) : 0.0f;
    duty = std::max(0.001f, std::min(0.999f, duty));

    float Voc = sim.solarOCV;
    float Isc_A = sim.solarCurrentMA / 1000.0f;
    float f_sw = sim.freq_hz;
    bool active_sync = sim.sync_mode && (OCR1B > OCR1A);

    float Rdcr = sim.low_spec_inductor ? 0.5f : 0.046f;
    float Rfet = 0.050f;
    float Vbat_ocv = sim.batteryOCV;

    // Numerical Solver: Solve for I_out
    float low = 0.0f, high = std::max(0.1f, Isc_A / (duty + 0.01f) + 2.0f);
    for(int i=0; i<40; i++) {
        float mid = (low + high) / 2.0f;
        float Ipan = mid * duty;
        float Vt = 1.5f;
        float Vsol = Voc - Vt * std::log(std::max(1e-9f, Ipan / (Isc_A + 0.001f) + 1.0f) / 1.0f) * 5.0f;
        if (Vsol < 0) Vsol = 0;

        float R_ind_eff = Rdcr * (1.0f + 0.15f * sqrt(f_sw / 15600.0f));
        float P_sw = (0.5f * Vsol * mid * f_sw * 100e-9f) + 0.02f * (f_sw / 15600.0f);
        float V_loss = mid * (Rfet * duty + (active_sync ? Rfet : 0.01f) * (1.0f - duty) + R_ind_eff)
                     + (active_sync ? 0.05f : 0.6f * (1.0f - duty))
                     + (P_sw / (mid + 0.001f));

        if (Vsol * duty - V_loss > Vbat_ocv) low = mid;
        else high = mid;
    }

    float I_out = low;
    float I_pan = I_out * duty;
    sim.solarCurrentMA_actual = std::max(0.0f, I_pan * 1000.0f);
    float Vt = 1.5f;
    sim.solarBusV = Voc - Vt * std::log(std::max(1e-9f, I_pan / (Isc_A + 0.001f) + 1.0f) / 1.0f) * 5.0f;
    if (sim.solarBusV < 0) sim.solarBusV = 0;

    sim.batteryV = Vbat_ocv + I_out * 0.05f;
    sim.vcc = 5.0f;
    sim.solarShuntV = sim.solarCurrentMA_actual * 0.01f;
    current_time_ms += 100;
}

void delay(unsigned long ms) {
    while (ms >= 100) { update_sim(); ms -= 100; }
    current_time_ms += ms;
}

void analogReference(uint8_t mode) { current_adc_ref = mode; }
void pinMode(int pin, int mode) {}
void digitalWrite(int pin, int val) {}
int digitalRead(int pin) { return (pin == 2) ? (sim.motion ? 1 : 0) : 0; }
int analogRead(int pin) {
    float ref = (current_adc_ref == 2) ? 1.1f : 5.0f;
    if (pin == 0 || pin == 14) return (int)((sim.solarBusV / 4.0f) * 1023.0f / ref);
    if (pin == 1 || pin == 15) return (int)((sim.batteryV / 3.0f) * 1023.0f / ref);
    if (pin == 14) return (int)(1.1f * 1023.0f / sim.vcc);
    return 0;
}
void analogWrite(int pin, int val) {}
void set_sleep_mode(int mode) {}
void sleep_enable() {}
void sleep_mode() {}
void sleep_disable() {}
void sleep_bod_disable() {}
void sleep_cpu() { for(int i=0; i<80; i++) update_sim(); }
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
