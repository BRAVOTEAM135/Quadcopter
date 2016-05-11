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
 * Timer 1 is a 16 bit timer
 * TCCR1A - Timer/Counter1 Control Register A - Set WGM (Waveform Generation Mode)
 * TCCR1B - Timer/Counter2 Control Register B

 *
 */

/*----- #include -----*/
 #include "Arduino.h" /* Required to use the arduino libary */
 #include "Timer_1.h"



/*----- Constants (#define) -----*/
#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define PRESCALER 64

/*----- (Module) Local Variables (Static) -----*/
static volatile unsigned long overflow_count;
static volatile unsigned long timer1_overflow_count;

/*----- Local Function -----*/
/* configures timer 1 */


ISR(TIMER1_OVF_vect) {
	overflow_count++; //Capture number of times timer overflows
	}	
	

/*----- Global Functions -----*/
void init_timer(){
	
	//This is the impllementation that the arduino uses
	TCCR1B |= _BV(CS11) | _BV(CS10);	//Set Prescale to 64 clk/64

	TCCR1A |= _BV(WGM10);  				//8-bit Fast PWM      
	TCCR1B |= _BV(WGM12);  				// TOV1 Flag set TOP = 0x00FF = 255

	TIMSK1 |= (1 << TOIE1); 			// enable timer overflow interrupt
	
	
	

	}
	
	
unsigned long get_micros() {
	
  unsigned long m;
  uint8_t oldSREG = SREG;  					//SREG - AVR Status Register
  uint8_t t;  								// Used to hold the timer value
  
  cli();  									// Clear interrupt - blocks interrupts
  	
  m = overflow_count;  						//Store Timer overflow count locally

  t = TCNT1;  								// Store timer count locally
  	
  if ((TIFR1 & _BV(TOV1)) && (t < 255))  // Incase overflows while in function
    m++; // timer overflow count
  
  SREG = oldSREG; 							// Reset the AVR status Register back to what it was
  return ((m << 8) + t) * (PRESCALER / clockCyclesPerMicrosecond());	// Will generate an interupt every ~1ms 
 // return ((t + (m * 255)) * (4)) ; 			// 64/16 = 4 64 PRESCALE, 16 NUMER OF SCYLE IN 1 MICROSEC
  
	
	
	
	
	
  
  }

  
 
void delay_1(unsigned long millisDelay){
		
		unsigned long startTime = get_micros();

	while (millisDelay > 0){
		if((get_micros() - startTime) >= 1000){ //i.e. 1ms of time has elapsed
			millisDelay--;						// decrement the count
			startTime = get_micros();			// set start time value
		}
	}
	
}



/* END OF FILE */
