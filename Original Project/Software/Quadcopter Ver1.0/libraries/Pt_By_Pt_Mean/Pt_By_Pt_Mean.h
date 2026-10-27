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

/* Protect the header file from multiple inclusions */
 #ifndef PT_BY_PT_MEAN_H_
 #define PT_BY_PT_MEAN_H_

/*----- #include ... -----*/


		

/*----- Global Function Prototypes -----*/
// Called every cycle of the loop. Input is a scalar float. output is the mean
float average_rate_x (float data);
float average_rate_y (float data);
float average_rate_z (float data);

float average_thro(float data);
float average_rudd(float data);
float average_alie(float data);
float average_elev(float data);


float average_acc_x (float data);
float average_acc_y (float data);
float average_acc_z (float data);
/* Header protection close */
 #endif

 /* END OF FILE */