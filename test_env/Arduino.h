
#ifndef ARDUINO_MOCK_H
#define ARDUINO_MOCK_H

#include <iostream>
#include <string>
#include <cmath>
#include <chrono>
#include <thread>
#include <map>
#include <vector>
#include <stdint.h>
#include <sstream>
#include <queue>
#include <algorithm>

#include "Wire.h"

typedef uint8_t byte;
typedef bool boolean;

#define HIGH 0x1
#define LOW  0x0
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define PI 3.1415926535897932384626433832795

// Pin definitions
#define A0 0
#define A1 1
#define A2 2
#define A3 3
#define A4 4
#define A5 5

// AVR Registers / Bits for compatibility
extern int TCCR2B;
extern int TCCR1A;
extern int TCCR1B;
extern int ICR1;
extern int OCR1A;
#define _BV(bit) (1 << (bit))
#define COM1A1 7
#define WGM11 1
#define WGM12 3
#define WGM13 4
#define CS10 0

extern int MCUSR;
extern int WDRF;
extern int EIMSK;
extern int INT0;
extern int EIFR;
extern int INTF0;
extern int ADCSRA;
extern int ADEN;
extern int EICRA;
extern int ISC01;
extern int ISC00;
extern int WDCE;
extern int WDE;
extern int WDIE;
extern int WDP3;
extern int WDP0;

#define B11111000 0xF8
#define B00000001 0x01
#define B00000011 0x03
extern int WDTCSR;
#define WDP2 2
#define WDP1 1
#define SLEEP_MODE_PWR_DOWN 0
#define bit(b) (1UL << (b))

void set_sleep_mode(int mode);
void sleep_enable();
void sleep_mode();
void sleep_disable();
void sleep_bod_disable();
void sleep_cpu();

#define noInterrupts()
#define interrupts()
#define cli()
#define sei()
#define wdt_reset()
void wdt_disable();

// Standard Arduino functions
unsigned long millis();
void delay(unsigned long ms);
void pinMode(int pin, int mode);
void digitalWrite(int pin, int val);
int digitalRead(int pin);
int analogRead(int pin);
void analogWrite(int pin, int val);

long map(long x, long in_min, long in_max, long out_min, long out_max);

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))

#define F(x) x

#define SIMULATION 1

// String class mock (Move up so MockSerial can use it)
class String : public std::string {
public:
    String(const char* s = "") : std::string(s) {}
    String(const std::string& s) : std::string(s) {}
    String(int i) {
        std::stringstream ss;
        ss << i;
        *this = ss.str();
    }
    String(float f) {
        std::stringstream ss;
        ss << f;
        *this = ss.str();
    }
    String(double d) {
        std::stringstream ss;
        ss << d;
        *this = ss.str();
    }
    String(long l) {
        std::stringstream ss;
        ss << l;
        *this = ss.str();
    }
    String(bool b) {
        *this = b ? "1" : "0";
    }
    String& operator+=(const String& other) {
        this->std::string::append(other);
        return *this;
    }
    String& operator+=(const char* s) {
        this->std::string::append(s);
        return *this;
    }
    float toFloat() const {
        try { return std::stof(*this); } catch(...) { return 0.0f; }
    }
    String substring(int from) const {
        if (from >= (int)length()) return "";
        return String(this->substr(from));
    }
    void trim() {
        auto it1 = std::find_if(begin(), end(), [](char c) { return !isspace(c); });
        auto it2 = std::find_if(rbegin(), rend(), [](char c) { return !isspace(c); });
        if (it1 == end()) erase(begin(), end());
        else erase(it2.base(), end()), erase(begin(), it1);
    }
    char operator[](int i) const { return std::string::operator[](i); }
};

// Mock Serial class
class MockSerial {
    std::queue<char> rx_buffer;
public:
    void begin(unsigned long baud) {}
    void print(const std::string& s) { std::cout << s; }
    void print(const char* s) { std::cout << s; }
    void print(float f) { std::cout << f; }
    void print(float f, int p) { std::cout << f; }
    void print(int i) { std::cout << i; }
    void print(long l) { std::cout << l; }
    void print(unsigned int i) { std::cout << i; }
    void print(unsigned long l) { std::cout << l; }
    void print(double d) { std::cout << d; }
    void print(bool b) { std::cout << (b ? "1" : "0"); }
    void println() { std::cout << std::endl; }
    void println(const std::string& s) { std::cout << s << std::endl; }
    void println(const char* s) { std::cout << s << std::endl; }
    void println(float f) { std::cout << f << std::endl; }
    void println(float f, int p) { std::cout << f << std::endl; }
    void println(int i) { std::cout << i << std::endl; }
    void println(long l) { std::cout << l << std::endl; }
    void println(double d) { std::cout << d << std::endl; }
    void println(bool b) { std::cout << (b ? "1" : "0") << std::endl; }
    void flush() {}
    int available() { return rx_buffer.size(); }
    int read() {
        if (rx_buffer.empty()) return -1;
        char c = rx_buffer.front();
        rx_buffer.pop();
        return c;
    }
    int peek() {
        if (rx_buffer.empty()) return -1;
        return rx_buffer.front();
    }
    float parseFloat() {
        std::string s;
        while(available() && (isspace(peek()) || (!isdigit(peek()) && peek() != '-' && peek() != '.'))) read();
        while(available() && (isdigit(peek()) || peek() == '-' || peek() == '.')) s += (char)read();
        try { return std::stof(s); } catch(...) { return 0.0f; }
    }
    long parseInt() {
        std::string s;
        while(available() && (isspace(peek()) || (!isdigit(peek()) && peek() != '-'))) read();
        while(available() && (isdigit(peek()) || peek() == '-')) s += (char)read();
        try { return std::stol(s); } catch(...) { return 0; }
    }
    operator bool() { return true; }

    // Sim helper
    void sim_input(const std::string& s) {
        for (char c : s) rx_buffer.push(c);
    }
    String readStringUntil(char terminator) {
        String res = "";
        while(available()) {
            char c = read();
            if (c == terminator) break;
            res += c;
        }
        return res;
    }
};

extern MockSerial Serial;

inline String operator+(const String& a, const String& b) {
    String res = a;
    res += b;
    return res;
}
inline String operator+(const String& a, const char* b) {
    String res = a;
    res += b;
    return res;
}
inline String operator+(const char* a, const String& b) {
    String res = a;
    res += b;
    return res;
}

// --- Simulation Extensions ---
struct SimStats {
    double harvestedMAH;
    double consumedMAH;
};

struct SimSensors {
    float solarBusV;
    float solarOCV; // Open Circuit Voltage
    float solarShuntV;
    float solarCurrentMA;
    float batteryV;
    float batteryCapAH;
    float systemCurrentMA;
    double harvestedMAH;
    double consumedMAH;
    bool  motion;
    bool  ina219_ok;
};
extern SimSensors sim;
void update_sim();

#endif // ARDUINO_MOCK_H
