#ifndef TEST_AVR_IO_H
#define TEST_AVR_IO_H
#include <stdint.h>
extern volatile uint8_t TWCR, TWSR, TWBR, TWDR;
extern volatile uint8_t DDRB, PORTB, PINB;
extern volatile uint8_t SREG, TCCR1A, TCCR1B, TIMSK1, TIFR1;
extern volatile uint16_t TCNT1, OCR1A;
enum { TWINT = 7, TWEA = 6, TWSTA = 5, TWSTO = 4, TWEN = 2, TWIE = 0 };
enum { DDB5 = 5, PORTB5 = 5, PINB5 = 5, OCF1A = 1, OCIE1A = 1, WGM12 = 3, CS11 = 1 };
#endif
