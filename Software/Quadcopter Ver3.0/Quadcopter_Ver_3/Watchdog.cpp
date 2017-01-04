/*
 * Source File Rules:
 * ------------------
 * - Constants MUST be defined with capitals. E.g #define VALUE 10
 * - Functions MUST follow the following standard. E.g. adc_init(), adc_write()
 * - Variables Must follow the following standard. E.g. adcValueX, adcValueY
 * - Call by reference for variables. E.g.
 *
 * int a = 100;
 * int b = 200;
 * 
 * swap(&a, &b); //pass variable address in
 * 
 * Where
 * void swap(int *x, int *y){ // using the address will point to value (*x)
 * 	int temp;
 * 	temp = *x;
 * 	*x = *y;
 * 	*y = temp;
 * }
 * }
 * 
 * Source File Comments:
 * ---------------------
 *
 *
 *
 */

/*----- #include -----*/
 #include "Arduino.h" /* Required to use the arduino libary */
 #include "avr/wdt.h"
 #include "Watchdog.h"

/*----- Constants (#define) -----*/
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


/*----- (Module) Local Variables (Static) -----*/


/*----- Local Function -----*/


/*----- Global Functions -----*/
void init_watchdog(){
	
	cli(); // Disbale all interupts
	wdt_reset(); //Reset the watchdog
	/*Enter watchdog configuration mode*/
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	
	/*Enable interupt (WDIE), Enable Reset (WDE) */
	WDTCSR = (1<<WDIE) | (1<<WDE);
	
	/*Set watchdog time out */
	wdt_enable(WD_TIMEOUT);
	
	/* Enable global interupts */
	sei();

}

// Pets the watchdog, if WD is not pet within the timeout will cause board to reboot
void reset_watchdog(){
	wdt_reset();
}

ISR(WDT_Vect){
}



/* END OF FILE */
