/*
 * ISRs.cpp
 *
 *  Created on: 20 Dec 2016
 *      Author: Kevin
 */

#include "PinChangeInt.h"
#include "Sensor.h"
#include "Radio.h"
#include "T1.h"

//----- FUNCTIONS -----
// Init ISR for sensor
void init_sensor_isr() {
	pinMode(MPU_INT_PIN, INPUT_PULLUP);
	PCintPort::attachInterrupt(MPU_INT_PIN, mpuISR, RISING);
}

// Init ISR's for Radio
void init_radio_isr(){
	pinMode(THRO_IN_PIN, INPUT_PULLUP);
	pinMode(RUDD_IN_PIN, INPUT_PULLUP);
	pinMode(ELEV_IN_PIN, INPUT_PULLUP);
	pinMode(ALIE_IN_PIN, INPUT_PULLUP);
	pinMode(AUX1_IN_PIN, INPUT_PULLUP);
	pinMode(GEAR_IN_PIN, INPUT_PULLUP);

	// using the PinChangeInt library, attach the interrupts
	// used to read the channels
	PCintPort::attachInterrupt(THRO_IN_PIN, calcThro, CHANGE);//2
	PCintPort::attachInterrupt(RUDD_IN_PIN, calcRudd, CHANGE);//8
	PCintPort::attachInterrupt(ELEV_IN_PIN, calcElev, CHANGE);//7
	PCintPort::attachInterrupt(ALIE_IN_PIN, calcAlie, CHANGE);//4
	PCintPort::attachInterrupt(AUX1_IN_PIN, calcAux1, CHANGE);//9
	PCintPort::attachInterrupt(GEAR_IN_PIN, calcGear, CHANGE);//10
}

//----- INTERRUPT SERVICE ROUTINES (ISR) -----
//Sensor ISR
void mpuISR() {
	mpuInterrupt = true;
};


// Radio ISRs
void calcThro() {
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value

  if((PIND & _BV(PD2)) == _BV(PD2)){ // PD2

  //if(digitalRead(THRO_IN_PIN) == HIGH) {
    ulThroStart = get_count_t1();
	}
  else {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unThroInShared = (uint16_t)(get_count_t1() - ulThroStart);

    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= THRO_FLAG;
	}
};

void calcAlie(){
	 if ((PIND & _BV(PD4)) == _BV(PD4)) { //PD4
  //if(digitalRead(ALIE_IN_PIN) == HIGH) {
    ulAlieStart = get_count_t1();
	}
  else {
    unAlieInShared = (uint16_t)(get_count_t1() - ulAlieStart);
    bUpdateFlagsShared |= ALIE_FLAG;
	}
};

void calcElev(){
if ((PIND & _BV(PD7)) == _BV(PD7)){ //PD7
  //if(digitalRead(ELEV_IN_PIN) == HIGH)	{
    ulElevStart = get_count_t1();
	}
  else  {
    unElevInShared = (uint16_t)(get_count_t1() - ulElevStart);
    bUpdateFlagsShared |= ELEV_FLAG;
	}
};

void calcRudd() {
 if((PINB & _BV(PB0)) == _BV(PB0)){ // PB0
  //if(digitalRead(RUDD_IN_PIN) == HIGH) {
    ulRuddStart = get_count_t1();
	}
  else {
    unRuddInShared = (uint16_t)(get_count_t1() - ulRuddStart);
    bUpdateFlagsShared |= RUDD_FLAG;
	}
};

void calcGear(){
	 if ((PINB & _BV(PB1)) == _BV(PB1)) { //PB1
  //if(digitalRead(ALIE_IN_PIN) == HIGH) {
    ulGearStart = get_count_t1();
	}
  else {
    unGearInShared = (uint16_t)(get_count_t1() - ulGearStart);
    bUpdateFlagsShared |= GEAR_FLAG;
	}
};

void calcAux1(){
	 if ((PINB & _BV(PB2)) == _BV(PB2)) { //PB2
  //if(digitalRead(ALIE_IN_PIN) == HIGH) {
    ulAux1Start = get_count_t1();
	}
  else {
    unAux1InShared = (uint16_t)(get_count_t1() - ulAux1Start);
    bUpdateFlagsShared |= AUX1_FLAG;
	}
};
