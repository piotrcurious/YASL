
#ifndef ADAFRUIT_INA219_H
#define ADAFRUIT_INA219_H
#include <stdint.h>
#include "Arduino.h"
class Adafruit_INA219 {
public:
    Adafruit_INA219(uint8_t addr = 0x40) {}
    bool begin();
    float getBusVoltage_V();
    float getShuntVoltage_mV();
    float getCurrent_mA();
    float getPower_mW();
    void setCalibration_32V_2A();
};
#endif
