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
 #ifndef L3G4200D_H_
 #define L3G4200D_H_

/*----- #include ... -----*/


		

/*----- Global Function Prototypes -----*/
/* Initialise the comms for the gyro*/
void init_L3G4200D();

/* Configures and calibrate the gyroscope*/
void config_L3G4200D();

/* Read out the calculated angle from the gyro */
float read_angle_x_L3G4200D();
float read_angle_y_L3G4200D();
float read_angle_z_L3G4200D();

/* used to feed the angle from the complementary filter back into the gyro equation to minimise the drift*/
float read_fil_angle_x_L3G4200D(float filAngle);
float read_fil_angle_y_L3G4200D(float filAngle);
float read_fil_angle_z_L3G4200D(float filAngle);

/* Gets the gyro rate degrees per second - dps. This is used for the rate controller */
float read_rate_x_L3G4200D();
float read_rate_y_L3G4200D();
float read_rate_z_L3G4200D();

/* Header protection close */
 #endif

 /* END OF FILE */