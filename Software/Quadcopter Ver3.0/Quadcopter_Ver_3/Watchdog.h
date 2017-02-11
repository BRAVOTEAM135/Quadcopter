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
 #ifndef WATCHDOG_H_
 #define WATCHDOG_H_

/*----- #include ... -----*/


		

/*----- Global Function Prototypes -----*/
void init_watchdog();
void reset_watchdog();


// These constants are defined in the avr/wdt.h libary
/*
 * WDTO_15MS	15ms
 * WDTO_30MS	40ms
 * WDTO_60MS	60ms
 * WDTO_120MS	120ms
 * WDTO_250MS	250ms
 * WDTO_500MS	500ms
 * WDTO_1S		1s
 * WDTO_2S		2s
 * WDTO_4S		4s
 * WDTO_8S		8s
 */
#define WD_TIMEOUT WDTO_30MS

/* Header protection close */
 #endif

 /* END OF FILE */
