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
 * Provides a tick with 0.5us period
 * 2MHz clock due to the prescaler
 * 
 */

/*----- #include -----*/
 #include "Arduino.h" /* Required to use the arduino libary */
#include "T1.h"


/*----- Constants (#define) -----*/



/*----- (Module) Local Variables (Static) -----*/
volatile unsigned long overflow_count = 0;	//Holds the count for how many timers timer has overflowed
volatile uint16_t count_load = 63535; //Start value of the timer
/*
 *  count_load
 *  This is calculated from the following equation
 *  2^16 - 16MHz/Pre-Scaler/Desired Frequency
 *
 *  2^16 as Timer 1 is a 16bit timer
 */

/*----- Local Function -----*/
/* Interrupt Service Routine */
ISR(TIMER1_OVF_vect){ // Called every time the timer overflows - in this case 32.798ms (0.5us * 65536)
	overflow_count++;
	TCNT1 = count_load; //#KR

	 task_0_count++; // Heart beat on pin 13
	 task_1_count++;
	 task_2_count++;
	 task_3_count++;
}

/*----- Global Functions -----*/
void init_t1(){
	/* Configure Timer */
	TCCR1B |= _BV(CS11); // Set the prescaler to 8 giving a derived clock of 2MHz (0.5us) tick period
	TCCR1B &= ~(_BV(CS12) | _BV(CS10)); //Clear the other CS bits

	TIMSK1 |= _BV(TOIE1); // Enable interreupts

	TCCR1A &= ~(_BV(WGM11) | _BV(WGM10)); // Normal wave for generation - only count up
	TCCR1B &= ~(_BV(WGM13) | _BV(WGM12)); // Clear all bits

	//TCNT1 = 0x0000;
	TCNT1 = count_load;
}
unsigned long get_count_t1(){
	uint8_t SREG_old = SREG; //take copy of AVR Status reg
	cli(); //Critical section of code - disable interrupts
	uint16_t tcnt1_save = TCNT1;
	boolean flag_save = TOV1;
  
  if (flag_save){
    tcnt1_save = TCNT1;
    overflow_count++;
    TIFR1 |= _BV(TOV1);
  }
  //unsigned long total_count = (overflow_count*65536) + tcnt1_save;
  volatile unsigned long total_count = (overflow_count*2000) + tcnt1_save;
  SREG = SREG_old;
  
  return total_count;
}
float get_micros_t1(){
	return get_count_t1()/2.0;
}

void delay_t1(unsigned int ms){
	static unsigned long startTime = get_micros_t1();
	while (ms > 0){
		if ((get_micros_t1() - startTime) >= 1000){ //ie 1ms of time has elapsed
			startTime = get_micros_t1();			// set start time value
			ms--;									// decrement the counter

		}
	}
}

/* END OF FILE */
