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
 #ifndef TIMER_1_H_
 #define TIMER_1_H_

/*----- #include ... -----*/


		

/*----- Global Function Prototypes -----*/

/* initializes the timer */
void init_timer();

/* Returns the micro second count value */
unsigned long get_micros();


/* Take a milli second delay value */
void delay_1(unsigned long millisDelay);



/* Header protection close */
 #endif
