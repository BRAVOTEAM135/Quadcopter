#include "ledBlink.h"
#include <avr/io.h>


void ledBlink::init() {
  // Initialization implementation

  // Data Direction Register B (DDRB) controls the direction of the pins on port B.
  // Configure PB5 (Arduino D13) as an output
    DDRB |= (1 << DDB5);
}

void ledBlink::ledOff() {
  // PORTB controls the output state of the pins on port B.
  // LED off implementation
   PORTB &= ~(1 << PORTB5);
}

void ledBlink::ledOn() {
  // LED on implementation
   PORTB |= (1 << PORTB5);
}

void ledBlink::ledToggle() {
  // LED toggle implementation
   PINB = (1 << PINB5);
}

void ledBlink::close() {
  // Close implementation
  PORTB &= ~(1 << PORTB5);
}