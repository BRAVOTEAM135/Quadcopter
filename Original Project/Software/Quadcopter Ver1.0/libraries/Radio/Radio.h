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
 #ifndef RADIO_H_
 #define RADIO_H_

/*----- #include ... -----*/

		

/*----- Global Function Prototypes -----*/
void init_radio(); // initializes the pin mode, registers the intterupts and sets up the timer

float read_thro(); // indivual read of each axis and scaled
float read_rudd();
float read_elev();
float read_alie();

void read_radio(); // Reads the current value from the radio


float read_raw_thro(); /// Reads the raw value from each axis
float read_raw_rudd();
float read_raw_elev();
float read_raw_alie();

/* Header protection close */
 #endif

 /* END OF FILE */