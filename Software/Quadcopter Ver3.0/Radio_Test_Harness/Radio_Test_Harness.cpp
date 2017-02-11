#include <Arduino.h>
volatile unsigned long task_0_count = 0; // Heart beat on pin 13
volatile unsigned long task_1_count = 0;
volatile unsigned long task_2_count = 0;
volatile unsigned long task_3_count = 0;
volatile unsigned long task_3a_count = 0;

#include "T1.h"


unsigned long current_time;
byte last_channel_1;
unsigned long timer_1;
int receiver_input_channel_1;


void setup() {
	Serial.begin(115200);
	//init_t1();

	pinMode(13, OUTPUT);
	PORTB = PORTB & B11011111;
	// PCICR - Pin Change Interrupt Control Register
	PCICR |= (1 << PCIE2);    // set PCIE2 to enable PCMSK2 scan
	PCMSK2 |= (1 << PCINT18);  // set PCINT18 (digital input 2) to trigger an interrupt on state change
}

void loop() {
	Serial.println(receiver_input_channel_1);
	delay_t1(250);
}

//Thro is on Pin 2
ISR(PCINT2_vect){

	PORTB = PORTB | B00100000;
  //Channel 1=========================================
  if(last_channel_1 == 0 && (PIND & B00000100) ){         //Input 2 changed from 0 to 1
    //timer_1 = get_micros_t1();                                 //Set timer_1 to get_micros_t1()
    timer_1 = micros();
    last_channel_1 = 1;                                 //Remember current input state

  }
  else if(last_channel_1 == 1 && !(PIND & B00000100)){  //Input 2 changed from 1 to 0
    //receiver_input_channel_1 = get_micros_t1() - timer_1;      //Channel 1 is get_micros_t1() - timer_1
    receiver_input_channel_1 = micros() - timer_1;
    last_channel_1 = 0;                                 //Remember current input state

  }
  PORTB = PORTB & B11011111;

}
