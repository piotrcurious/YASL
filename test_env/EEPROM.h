
#ifndef EEPROM_MOCK_H
#define EEPROM_MOCK_H
#include <stdint.h>
#include <stddef.h>
class MockEEPROM {
public:
    uint8_t read(int addr);
    void write(int addr, uint8_t val);
    void update(int addr, uint8_t val);
    void commit();

    // Support for get/put templates
    template< typename T > T &get( int idx, T &t ) {
        uint8_t *ptr = (uint8_t*) &t;
        for (size_t i = 0; i < sizeof(T); i++) {
            *ptr++ = read(idx + i);
        }
        return t;
    }
    template< typename T > const T &put( int idx, const T &t ) {
        const uint8_t *ptr = (const uint8_t*) &t;
        for (size_t i = 0; i < sizeof(T); i++) {
            update(idx + i, *ptr++);
        }
        return t;
    }
};
extern MockEEPROM EEPROM;
#endif
