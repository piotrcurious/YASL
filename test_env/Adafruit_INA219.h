#ifndef ADAFRUIT_INA219_H
#define ADAFRUIT_INA219_H
#include <stdint.h>
class Adafruit_INA219 {
public:
    Adafruit_INA219(uint8_t addr = 0x40) {}
    bool begin() { return true; }
    float getBusVoltage_V() { return 12.0f; }
    float getShuntVoltage_mV() { return 1.0f; }
    float getCurrent_mA() { return 100.0f; }
    float getPower_mW() { return 1200.0f; }
};
#endif
