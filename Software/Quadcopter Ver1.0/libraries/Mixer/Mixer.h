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
 #ifndef MIXER_H_
 #define MIXER_H_

/*----- #include ... -----*/


		

/*----- Global Function Prototypes -----*/
void config_mixer(int qConfig); // 1 = + and 2 = X
void roll_mixer(float throVal, float mixVal);
void pitch_mixer(float throVal, float mixVal);
void yaw_mixer();



/* Header protection close */
 #endif

 /* END OF FILE */