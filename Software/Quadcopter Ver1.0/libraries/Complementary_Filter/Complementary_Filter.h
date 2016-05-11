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
 *
 *
 *
 */
#include "Arduino.h"
/* Protect the header file from multiple inclusions */
 #ifndef COMP_FIL_H_
 #define COMP_FIL_H_

/*----- #include ... -----*/

		

/*----- Global Function Prototypes -----*/
/* Reads the filter angles */
float x_angle_comp_fil();
float y_angle_comp_fil();
float z_angle_comp_fil();


/* Header protection close */
 #endif

 /* END OF FILE */