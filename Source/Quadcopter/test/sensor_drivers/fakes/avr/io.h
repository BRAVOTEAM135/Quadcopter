#ifndef TEST_AVR_IO_H
#define TEST_AVR_IO_H
#include <stdint.h>
extern volatile uint8_t TWCR, TWSR, TWBR, TWDR;
enum { TWINT = 7, TWEA = 6, TWSTA = 5, TWSTO = 4, TWEN = 2, TWIE = 0 };
#endif
