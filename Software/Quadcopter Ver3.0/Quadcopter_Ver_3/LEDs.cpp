/*
 * LEDs.cpp
 *
 *  Created on: 21 Dec 2016
 *      Author: Kevin
 */
#include "LEDs.h"


void init_a_leds(){

	// Using Analog Pins as Digital genral purpose
	pinMode(GPIO0, OUTPUT);
	pinMode(GPIO1, OUTPUT);
	pinMode(GPIO2, OUTPUT);
	//pinMode(GPIO3, OUTPUT);
}

void init_d_leds(){
	// DEBUG: configure Pin13 LED
	pinMode(LED_PIN, OUTPUT);
}

void set_led(uint8_t pin){
	if (pin == 13) PORTB = PORTB | B00100000; // Set Pin 13
	if (ENABLE){
		if (pin == A0) PORTC = PORTC | B00000001; // Set A0: GPIO0
		if (pin == A1) PORTC = PORTC | B00000010; // Set A1: GPIO1
		if (pin == A2) PORTC = PORTC | B00000100; // Set A2: GPIO2
		if (pin == A3) PORTC = PORTC | B00001000; // Set A3: GPIO3
	}
}

void clear_led(uint8_t pin){
	if (pin == 13) PORTB = PORTB & B11011111; // Clear Pin 13
	if (ENABLE){
		if (pin == A0) PORTC = PORTC & B11111110; // Clear A0: GPIO0
		if (pin == A1) PORTC = PORTC & B11111101; // Clear A1: GPIO1
		if (pin == A2) PORTC = PORTC & 111111011; // Clear A2: GPIO2
		if (pin == A3) PORTC = PORTC & 111110111; // Clear A3: GPIO3
	}
}

void toggle_led(uint8_t pin){
	if (pin == 13) PORTB = PORTB ^ B00100000; // Toggle Pin 13
	if (ENABLE) {
		if (pin == A0) PORTC = PORTC ^ B00000001; // Toggle A0: GPIO0
		if (pin == A1) PORTC = PORTC ^ B00000010; // Toggle A1: GPIO1
		if (pin == A2) PORTC = PORTC ^ B00000100; // Toggle A2: GPIO2
		if (pin == A3) PORTC = PORTC ^ B00001000; // Toggle A3: GPIO3
	}
}
