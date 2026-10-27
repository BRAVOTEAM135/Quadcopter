// the prescaler is sett so that timer1 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks

//#define MICROSECONDS_PER_TIMER1_OVERFLOW (clockCyclesPerMicrosecond(8 * 256))


volatile unsigned long timer1OverflowCount = 0;
void setup() {
Serial.begin(115200);
init_t();


}

void loop() {
  
  //unsigned long aa = micros(); 
  unsigned long a = mmicros();
 // delay(500);
 ddelay(500);
 // while (a - mmicros() > 500000){}
  
  
 unsigned long b = mmicros();
  //unsigned long bb = micros();
  //Serial.println(b - a);
  //Serial.println(bb - aa);
  //Serial.println((b-a)*0.0000005);
  //Serial.println((b-a)*0.5);
  Serial.println(a);
  Serial.println(b);
  Serial.println((b-a));

  Serial.println();


//Serial.println(mmicros());
//Serial.println(micros());

  
}

// Wehn TImer 1 counter overflows this will occur every 1/clk * prescaler * timer max
ISR (TIMER1_OVF_vect){
	timer1OverflowCount++; // Increment the overflow count
        }

void init_t(){
sei();

// Fast PWM 8 bit, TOP = 0x00FF, Update of OCR1x at BOTTOM, TOV1 Flag Set on = TOP
// sbi(TCCR1A, WGM10);
// sbi(TCCR1B, WGM12);
//TCCR1A |= _BV(WGM10);
//TCCR1B |= _BV(WGM12);

//Fast PWM 10 bit
TCCR1A |= _BV(WGM10) | _BV(WGM11);
TCCR1B |= _BV(WGM12);

// Set timer 1 prescale factor to 64
// sbi(TCCR1B, CS10);
// sbi(TCCR1B, CS11);
TCCR1B |= _BV(CS11); // Prescale 8 


// Enable Timer 1 overflow interrupt
//sbi(TIMSK1, TOIE1);
TIMSK1 = _BV(TOIE1);

}

unsigned long mmicros(){
  unsigned long m;
  uint8_t oldSREG = SREG; 
  int t; 
  
  
  cli();
  m = timer1OverflowCount;
  t = TCNT1; // current count
  
  // incase interrupt whie we are in the function
  // 8 bit 
  //if ((TIFR1 & _BV(TOV1)) && (t < 255)){
  
  // 10 bit  
  if ((TIFR1 & _BV(TOV1)) && (t < 1023)){
    m++;
    }
    
    SREG = oldSREG;

    //return ((m << 8) + t);// * (8 / clockCyclesPerMicrosecond());
    return (((m << 10) + t)) * 4;

    //return ((m << 8) + t) * (8 / clockCyclesPerMicrosecond());
  }



void ddelay(unsigned long ms){
  uint16_t start = mmicros();
    while (ms > 0){
    //yield();
    
    if ((mmicros() - start) >= 20000000) {
      ms--;
      start += 20000000;
      //start += 1000;
      }
    }
  }
/*
arduino
timer 0 fast pwm
top = 0xFF
update on  BOTTOM
TOV flag set on TOP

notes BOTTOM = 0x00, MAX = oxFF


Timer 1 fast pwm 8bit 



*/
