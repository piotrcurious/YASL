
#ifndef WIRE_MOCK_H
#define WIRE_MOCK_H
#include "Arduino.h"
class MockWire {
public:
    void begin();
    void beginTransmission(uint8_t addr);
    int endTransmission();
    void write(uint8_t val);
    int read();
    uint8_t requestFrom(uint8_t addr, uint8_t len);
};
extern MockWire Wire;
#endif
