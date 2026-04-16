#ifndef IR_REMOTE_MOCK_H
#define IR_REMOTE_MOCK_H

#include <stdint.h>

struct decode_results {
    uint32_t value;
    int decode_type;
    uint16_t bits;
};

#define NEC 1
#define RC5 2
#define RC6 3
#define UNKNOWN -1

#define NEC_PROTOCOL 1
#define NEC_DISPLAY_CONFIRM_REQUEST 0x1234
#define RC5_LEARN_START 0x1
#define RC5_LEARN_CONFIRM 0x2
#define RC5_LEARN_EXIT 0x3
#define RC5_USER_LEAVING 0x4

class IRrecv {
public:
    IRrecv(int pin) {}
    void enableIRIn() {}
    bool decode(decode_results* results) { return false; }
    void resume() {}
};

class IRsend {
public:
    IRsend(int pin = 0) {}
    void begin() {}
    void send(int protocol, uint32_t data, int bits) {}
    void sendNEC(uint32_t data, int bits) {}
};

#endif
