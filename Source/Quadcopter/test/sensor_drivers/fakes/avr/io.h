#ifndef TEST_AVR_IO_H
#define TEST_AVR_IO_H
#include <stdint.h>
extern volatile uint8_t TWCR, TWSR, TWBR, TWDR;
extern volatile uint8_t DDRB, PORTB, PINB;
extern volatile uint8_t DDRD, PORTD;
extern volatile uint8_t TCCR0A, TCCR0B, TIMSK0, TIFR0, TCNT0, OCR0A, OCR0B;
extern volatile uint8_t TCCR2A, TCCR2B, TIMSK2, TIFR2, TCNT2, OCR2A, OCR2B;
extern volatile uint8_t SREG, TCCR1A, TCCR1B, TIMSK1, TIFR1;
extern volatile uint16_t TCNT1, OCR1A;
enum { TWINT = 7, TWEA = 6, TWSTA = 5, TWSTO = 4, TWEN = 2, TWIE = 0 };
enum { DDB5 = 5, PORTB5 = 5, PINB5 = 5, OCF1A = 1, OCIE1A = 1, WGM12 = 3, CS11 = 1 };
enum { PD3 = 3, PD5 = 5, PD6 = 6, PB3 = 3 };
enum { COM0A1 = 7, COM0B1 = 5, WGM00 = 0, CS01 = 1, CS00 = 0,
       FOC0A = 7, FOC0B = 6, OCF0A = 1, OCF0B = 2, TOV0 = 0 };
enum { COM2A1 = 7, COM2B1 = 5, WGM20 = 0, CS22 = 2,
       FOC2A = 7, FOC2B = 6, OCF2A = 1, OCF2B = 2, TOV2 = 0 };
#endif
