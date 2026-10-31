#ifndef TEST_AVR_INTERRUPT_H
#define TEST_AVR_INTERRUPT_H
#define TWI_vect test_twi_interrupt
#define TIMER1_COMPA_vect test_timer1_interrupt
#define ISR(vector) void vector()
#define cli() (SREG &= ~(1 << 7))
/// Expose the real TWI interrupt handler under a test-callable name.
void test_twi_interrupt();
/// Expose the real Timer1 compare handler under a test-callable name.
void test_timer1_interrupt();
#endif
