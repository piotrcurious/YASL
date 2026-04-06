#ifndef INTERRUPT_MOCK_H
#define INTERRUPT_MOCK_H
#define ISR(vector) extern "C" void vector()
#define WDT_vect WDT_handler
#endif
