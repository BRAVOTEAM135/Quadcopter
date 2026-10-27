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
 #ifndef MOTORS_H_
 #define MOTORS_H_

/*----- #include ... -----*/


		

/*----- Global Function Prototypes -----*/
void init_motors();

/* Arms the electronic Speed Controllers
 * NOTE: This function will need to be modified for each ESC
 */
void config_motors();

/* Input is 0 - 100%   */
void write_motor_1(int M1);
void write_motor_2(int M2);
void write_motor_3(int M3);
void write_motor_4(int M4);


/* Header protection close */
 #endif

 /* END OF FILE */