
#ifndef NTPCLIENT_H
#define NTPCLIENT_H
class NTPClient {
public:
    NTPClient(void* udp) {}
    void begin() {}
    void update() {}
    unsigned long getEpochTime() { return 0; }
};
#endif
