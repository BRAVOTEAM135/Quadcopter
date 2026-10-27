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
 * https://www.sparkfun.com/products/9836
 *
 *
 */
 #include "Arduino.h"
/* Protect the header file from multiple inclusions */
 #ifndef ADXL345_H_
 #define ADXL345_H_

/*----- #include ... -----*/

/* Tells the cmpiler to use C calling syntax instead of C++ */
/*
#ifdef __cplusplus
    extern "C" {
#endif
	*/	

/*----- Global Function Prototypes -----*/

/* Starts the I2C coms */
void init_ADXL345();

/* When this is called the ADXL345 must be stationary */
void config_ADXL345();

/* reads the calculated postional angle */
float read_angle_x_ADXL345();
float read_angle_y_ADXL345();
float read_angle_z_ADXL345();


/* Tells the compiler to end using C syntax */
/*
#ifdef __cplusplus
    }
#endif
*/
/* Header protection close */
 #endif
