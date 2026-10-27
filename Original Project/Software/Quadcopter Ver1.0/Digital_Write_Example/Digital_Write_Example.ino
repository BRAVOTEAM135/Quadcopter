void setup() {
 
 Serial.begin(9600);
  // put your setup code here, to run once:
DDRB |= DDRB | B00100000;
}

void loop() {

//PORTB = ~PORTB;
//PORTB ^= B00100000;
//PORTB ^= (1<<5);
PORTB ^= _BV(5);

Serial.println(PINB, BIN); // Read Digital Pin
Serial.println(PINB & _BV(2));
Serial.println((PINB & _BV(2) & 2));
Serial.println(PINB & (1<<2),BIN);
Serial.println(PINB & bit(2),BIN); 

if((PINB & _BV(2) & 2) == 2){
Serial.println("HIGH");
}
else
Serial.println("LOW");

Serial.println();
delay(1000);
}
