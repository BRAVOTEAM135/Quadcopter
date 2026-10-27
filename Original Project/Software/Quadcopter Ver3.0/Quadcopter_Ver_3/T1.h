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
 #ifndef T1_H_
 #define T1_H_

/*----- #include ... -----*/


		

/*----- Global Function Prototypes -----*/
void init_t1();
uint32_t get_count_t1();
float get_micros_t1();
void delay_t1(uint16_t ms);

extern volatile unsigned long task_0_count; // Heart beat on pin 13
extern volatile unsigned long task_1_count;
extern volatile unsigned long task_2_count;
extern volatile unsigned long task_3_count;
extern volatile unsigned long task_4_count;
extern volatile unsigned long task_5_count;

/* Header protection close */
 #endif

 /* END OF FILE */
