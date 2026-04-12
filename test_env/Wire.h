#ifndef WIRE_MOCK_H
#define WIRE_MOCK_H
#include <cstdint>
class MockWire {
public:
    void begin() {}
    void beginTransmission(uint8_t addr);
    int endTransmission();
    void write(uint8_t val);
    uint8_t requestFrom(uint8_t addr, uint8_t len) { return 0; }
    int available() { return 0; }
    int read() { return 0; }
};
extern MockWire Wire;
#endif
