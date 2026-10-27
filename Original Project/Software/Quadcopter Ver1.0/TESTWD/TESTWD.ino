#include <avr/wdt.h>
int i = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("ALIVE");
  //i=0;
  cli(); // Disbale all interupts
	wdt_reset(); //Reset the watchdog
	/*Enter watchdog configuration mode*/
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	
	/*Enable interupt (WDIE), Enable Reset (WDE) */
	WDTCSR = (1<<WDIE) | (1<<WDE);
	
	/*Set watchdog time out */
	wdt_enable(WDTO_8S);
	
	/* Enable global interupts */
	sei();


wdt_enable(WDTO_8S);
}

void loop() {
  Serial.println(i);
  i++;
  delay(500);
  // put your main code here, to run repeatedly:

}

ISR (WDT_Vect){
  Serial.println("DEAD");
}
