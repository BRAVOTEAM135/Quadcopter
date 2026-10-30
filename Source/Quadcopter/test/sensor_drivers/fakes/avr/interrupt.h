#ifndef TEST_AVR_INTERRUPT_H
#define TEST_AVR_INTERRUPT_H
#define TWI_vect test_twi_interrupt
#define ISR(vector) void vector()
void test_twi_interrupt();
#endif
