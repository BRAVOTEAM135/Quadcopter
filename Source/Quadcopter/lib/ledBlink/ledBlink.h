#ifndef LEDBLINK_H
#define LEDBLINK_H

#include "driverStatus.h"

class ledBlink {
  private:
    bool initialized_ = false;
  public:
    /// Create the software object without starting hardware; call init() during setup.
    ledBlink() = default;
    /// Configure the onboard LED pin D13/PB5 as an output.
    void init();
    /// Turn the onboard LED off if the driver is initialized.
    void ledOff();
    /// Turn the onboard LED on if the driver is initialized.
    void ledOn();
    /// Invert the onboard LED output if the driver is initialized.
    void ledToggle();
    /// Turn the LED off and release its pin as an input; ignore later LED commands until
    /// init().
    void close();
    /// Return whether the LED pin is configured; LED writes have no pending work.
    DriverStatus getStatus() const { return {initialized_, false, DriverError::None}; }
  };
  
#endif  
