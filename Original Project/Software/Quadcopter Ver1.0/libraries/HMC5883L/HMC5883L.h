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
 * Documentation
 * https://www.sparkfun.com/products/10530
 * http://www.loveelectronics.co.uk/Tutorials/8/hmc5883l-tutorial-and-arduino-library
 *
 *
 */

/* Protect the header file from multiple inclusions */
 #ifndef HMC5883L_H_
 #define HMC5883L_H_

/*----- #include ... -----*/


		

/*----- Global Function Prototypes -----*/
/* Establishes Comms with device */
void init_HMC5883L();

/* Write all configuration to device - see .cpp */
void config_HMC5883L();

/* Reads the scaled magnetometer values */
float scale_x_HMC5883L();
float scale_y_HMC5883L();
float scale_z_HMC5883L();


/* Provides heading in radians */
float rad_heading_HMC5883L();

/* Provides heading in degrees */
float deg_heading_HMC5883L();

/* Header protection close */
 #endif

 /* END OF FILE */