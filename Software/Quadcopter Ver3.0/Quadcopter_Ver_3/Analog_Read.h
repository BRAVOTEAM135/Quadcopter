/*
 * Analog_Read.h
 *
 *  Created on: 28 Dec 2016
 *      Author: Kevin
 */

#ifndef ANALOG_READ_H_
#define ANALOG_READ_H_
#include "Arduino.h"
// High when a value is ready to be read
extern volatile int readFlag;

// Value to store analog result
extern volatile int analogVal;


void init_analog_read(); // Initialise the ADC for analog read
float smoothing(); // Filters the analog read singal to minamise the noise



#endif /* ANALOG_READ_H_ */
