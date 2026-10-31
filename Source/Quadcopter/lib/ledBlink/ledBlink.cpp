#include "ledBlink.h"
#include <avr/io.h>


/// Configure the onboard LED pin D13/PB5 as an output.
void ledBlink::init() {
  // Initialization implementation

  // Data Direction Register B (DDRB) controls the direction of the pins on port B.
  // Configure PB5 (Arduino D13) as an output
    DDRB |= (1 << DDB5);
    initialized_ = true;
}

/// Turn the onboard LED off if the driver is initialized.
void ledBlink::ledOff() {
  if (!initialized_) {
    return;
  }
  // PORTB controls the output state of the pins on port B.
  // LED off implementation
   PORTB &= ~(1 << PORTB5);
}

/// Turn the onboard LED on if the driver is initialized.
void ledBlink::ledOn() {
  if (!initialized_) {
    return;
  }
  // LED on implementation
   PORTB |= (1 << PORTB5);
}

/// Invert the onboard LED output if the driver is initialized.
void ledBlink::ledToggle() {
  if (!initialized_) {
    return;
  }
  // LED toggle implementation
   PINB = (1 << PINB5);
}

/// Turn the LED off and release its pin as an input; ignore later LED commands until init().
void ledBlink::close() {
  if (!initialized_) {
    return;
  }
  PORTB &= ~(1 << PORTB5);
  DDRB &= ~(1 << DDB5);
  initialized_ = false;
}
