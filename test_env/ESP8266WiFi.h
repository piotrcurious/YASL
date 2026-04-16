
#ifndef ESP8266WIFI_H
#define ESP8266WIFI_H
#define WL_CONNECTED 1
class MockWiFiClass {
public:
    int status() { return WL_CONNECTED; }
};
extern MockWiFiClass WiFi;
#endif
