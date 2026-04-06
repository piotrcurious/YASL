
#ifndef INTERRUPT_MOCK_H
#define INTERRUPT_MOCK_H
#include <iostream>
#define ISR(vector) extern "C" void vector()
#define WDT_vect WDT_handler
#define INT0_vect INT0_handler

extern "C" {
    void WDT_handler() __attribute__((weak));
    void INT0_handler() __attribute__((weak));
}
#endif
