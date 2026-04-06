
#ifndef ADAFRUIT_INA219_H
#define ADAFRUIT_INA219_H
#include <stdint.h>
#include "Arduino.h"
class Adafruit_INA219 {
public:
    Adafruit_INA219(uint8_t addr = 0x40) {}
    bool begin() { return sim.ina219_ok; }
    float getBusVoltage_V() { return sim.solarBusV; }
    float getShuntVoltage_mV() { return sim.solarShuntV; }
    float getCurrent_mA() { return sim.solarCurrentMA; }
    float getPower_mW() { return sim.solarBusV * sim.solarCurrentMA; }
};
#endif
