
#ifndef TIMELIB_MOCK_H
#define TIMELIB_MOCK_H
#include <stdint.h>
#include <time.h>
static inline int hour() { return 12; }
static inline int minute() { return 0; }
static inline int second() { return 0; }
static inline unsigned long now() { return 0; }
static inline void setTime(int h, int m, int s, int d, int mo, int y) {}
#endif
