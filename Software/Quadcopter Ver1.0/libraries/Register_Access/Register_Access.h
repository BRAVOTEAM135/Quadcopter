/*
 * Header File Rules
 * -----------------
 * - DO included all function prototyes that are public. E.g. adc_init(), adc_config(), adc_read()
 * - DON'T  include private functions. E.g. adc_scale()
 * - DON'T include executable lines of code or variable declarations
 * - DON'T expose any variables in a header - pass variables by reference E.g. void swap(int *x, int *y);
 * - Comments in here are for the user of the libary
 * 
 * Header File Comments:
 * ---------------------
 * Packages up the wire libary and allows for I2C device to read easily through
 * this API. Functions: read_reg() and write_reg()
 *
 */
	
/* Protect the header file from multiple inclusions */
 #ifndef REGISTER_ACCESS_H_
 #define REGISTER_ACCESS_H_

/*----- #include ... -----*/
#include "Arduino.h"
	

/*----- Global Function Prototypes -----*/
 /* Start the I2C bus */
 void init_reg();
 
 /* Reads I2C Register */
 int read_reg(int deviceAddress, byte address);
 
 /* Writes to the I2C Register */
 void write_reg(int deviceAddress, byte address, byte val);



/* Header protection close */
 #endif
