#include <cstdlib>

#include "Arduino.h"
#include "avr/interrupt.h"
#include "EEPROM.h"

int TCCR2A = 0;
int TCCR2B = 0;
int TCCR1A = 0;
int TCCR1B = 0;
int ICR1 = 0;
int OCR1A = 0;
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

SimSensors sim = { 12.0f, 18.0f, 0.0f, 100.0f, 3.7f, 2.0f, 10.0f, 0.0, 0.0, 25.0f, 0.2f, false, true };

unsigned long current_time_ms = 0;

unsigned long millis() {
    return current_time_ms;
}

void update_sim() {
    // Non-Ideal Buck Converter Model:
    // Vbat = (Vsolar * Duty) - (Iout * R_conv) - V_diode
    // Iin = Iout * Duty (ignoring switching losses for now)
    // R_conv(T) = R_base * (1 + alpha * (T - 25))

    float alpha = 0.004f; // Thermal coefficient (Copper/Silicon avg)
    float R_curr = sim.R_conv_base * (1.0f + alpha * (sim.tempC - 25.0f));
    float V_diode = 0.4f; // Schottky or Body Diode drop

    float duty = (float)OCR1A / 1023.0f;

    if (duty < 0.01f) {
        sim.solarBusV = sim.solarOCV;
        sim.solarCurrentMA = 0;
    } else {
        // We need to find Iout that satisfies both the Panel and the Converter.
        // Ipanel(Vsolar) = Iout * Duty
        // Vsolar = (Vbat + V_diode + Iout * R_curr) / Duty

        // Iterative solver for equilibrium
        float Iout_est = 0.0f;
        for (int i=0; i<5; ++i) {
            float Vsolar_est = (sim.batteryV + V_diode + (Iout_est/1000.0f) * R_curr) / duty;
            if (Vsolar_est > sim.solarOCV) Vsolar_est = sim.solarOCV;

            // PV Panel Model: simplified diode equation
            float Isc = 3000.0f; // 3A
            float Io = 0.001f;
            float Vt = 2.0f;
            float Ipanel = Isc - Io * (exp(Vsolar_est / (Vt * (sim.tempC + 273.15f) / 298.15f)) - 1.0f);

            // Limit to MPP - simplified for stable sim testing
            if (Vsolar_est < 15.0f) {
                Ipanel *= (Vsolar_est / 15.0f);
            }
            if (Ipanel < 0) Ipanel = 0;

            Iout_est = Ipanel / duty;
            sim.solarBusV = Vsolar_est;
            sim.solarCurrentMA = Ipanel;
        }
    }

    float chargeAH = sim.batteryCapAH * (sim.batteryV - 3.0f) / (4.2f - 3.0f);
    unsigned long step_ms = 100;
    current_time_ms += step_ms;

    // Net current (Solar in - System out)
    float solarOutMA = (duty > 0.01f) ? (sim.solarCurrentMA / duty) : 0;
    float netMA = solarOutMA - sim.systemCurrentMA;
    float deltaAH = (netMA / 1000.0f) * (step_ms / 3600000.0f);

    // Stats
    if (sim.solarCurrentMA > 0) sim.harvestedMAH += (sim.solarCurrentMA) * (step_ms / 3600000.0);
    sim.consumedMAH += (sim.systemCurrentMA) * (step_ms / 3600000.0);

    // Self-heating: simple model
    // dT = (P_loss * R_thermal - (T - T_ambient)) * dt / C_thermal
    float P_loss = (solarOutMA / 1000.0f) * (solarOutMA / 1000.0f) * R_curr;
    sim.tempC += (P_loss * 10.0f - (sim.tempC - 25.0f)) * (step_ms / 10000.0f);

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
