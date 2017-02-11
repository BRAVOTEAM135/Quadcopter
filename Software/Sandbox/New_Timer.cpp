/* 
* ==============================================================
* ===== Defines =====
* ==============================================================
* This is defined in the Arduino.h header
*/
#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )

// the prescaler is set so that timer1 ticks every 8 clock cycles, and the
// the overflow handler is called every 2 ticks.
#define MICROSECONDS_PER_TIMER1_OVERFLOW (clockCyclesToMicroseconds(8 * 2))

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER1_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER1_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

// =============================================================

/* 
* ==============================================================
* ===== Variables =====
* ==============================================================
*/
volatile unsigned long timer1_overflow_count = 0;
volatile unsigned long timer1_millis = 0;
static unsigned char timer1_fract = 0;

static uint16_t timer1_init_count = 65533; // init_count = 65536 - 16Mhz/Prescaler/Desired Freq

// =============================================================




/* 
* ==============================================================
* ===== Function =====
* ==============================================================
*/
// ----- Init Timer 1 -----
// Timer one is used as timer for system. GetMicros and delays
cli(); //Disable all interrupts
TCCR1A = 0;
TCCR1B = 0;

TCNT1 = timer1_init_count;
// TCCR1 - set WGM waveform generation mode
// Fast PWM 10 bit (0x03FF) 12 11 10
sbi(TCCR1A, WGM10);
sbi(TCCR1A, WGM11);
sbi(TCCR1B, WGM12);

// set the prescaler
sbi(TCCR1B, CS11); // 8 Bit Scaler

// TIMSK - enable overflow interrupt
TIMSK1 |= (1 << TOIE1);
sei(); // Enable all Interrupts



// ----- Tick Count (us) -----
// This is give the tick count in us Micro Seconds
unsigned long tickCountUs(){
	
	// m holds the intterupt over flow count i.e. how many times has OVF
	unsigned long m;
	
	// Take a copy of the SREG
	uint8_t oldSREG = SREG; // SREG is the AVR Status Register
	// 7	6	5	4	3	2	1	0
	// I	T	H	S	V	N	Z	C
	/*
	The Status Register bits are:
	C - Carry Flag
	Z - Zero Flag
	N - Negative Flag
	V - Overflow Flag
	S - Sign Flag
	H - Half Carry
	T - Bit Copy Storage
	I - Global Interrupt Enable Flag
	*/
	
	uint8_t tl, th;
	uint16_t t;
	
	// No Interrupts
	cli(); 
	
	// timer1_overflow_count is incremented everytime the T1 OVF
	m = timer1_overflow_count
	
	// TCNT1 - Timer/Counter 1 - Holds the count value
	// Get the two 8bit registers that hold the 16 word for the count
	tl = TCNT1L;
	th = TCNT1H;
	
	// Combine the 2 bytes into a single 2 byte word
	t = (th << 8) + tl; // Note: This line take 380ns
	
	// Timer/Counter 1 Interrupt Flag Register
	// Check the Overflow Flag AND the Count
	//if ((TIFR1 & _BV(TOV1)) && (t < 65535)) // Logicsal && both items are true.
	//	m++;
	
	SREG = oldSREG;
	
	return 8 / clockCyclesPerMicrosecond();
}


// ----- delay Microseconds -----
/* Delay for the given number of microseconds.  Assumes a 1, 8, 12, 16, 20 or 24 MHz clock. */
void delayMicroseconds(unsigned int us)
{
	// call = 4 cycles + 2 to 4 cycles to init us(2 for constant delay, 4 for variable)

	// calling avrlib's delay_us() function with low values (e.g. 1 or
	// 2 microseconds) gives delays longer than desired.
	//delay_us(us);
	
	// for the 16 MHz clock on most Arduino boards

	// for a one-microsecond delay, simply return.  the overhead
	// of the function call takes 14 (16) cycles, which is 1us
	if (us <= 1) return; //  = 3 cycles, (4 when true)

	// the following loop takes 1/4 of a microsecond (4 cycles)
	// per iteration, so execute it four times for each microsecond of
	// delay requested.
	us <<= 2; // x4 us, = 4 cycles

	// account for the time taken in the preceeding commands.
	// we just burned 19 (21) cycles above, remove 5, (5*4=20)
	// us is at least 8 so we can substract 5
	us -= 5; // = 2 cycles,
	
	
		// busy wait
	__asm__ __volatile__ (
		"1: sbiw %0,1" "\n\t" // 2 cycles
		"brne 1b" : "=w" (us) : "0" (us) // 2 cycles
	);
	// return = 4 cycles
}


// =============================================================
// ===== Overflow ISR =====
// =============================================================
ISR(TIMER1_OVF_vect){
	TCNT1 = timer1_init_count;
	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	unsigned long m = timer1_millis;
	unsigned char f = timer1_fract;

	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}

	timer1_fract = f;
	timer1_millis = m;
	timer1_overflow_count++;
	
}

// =============================================================
