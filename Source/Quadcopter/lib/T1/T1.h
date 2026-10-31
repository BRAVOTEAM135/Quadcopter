/*
 * Timer1 Driver
 *
 * Hardware: ATmega328P
 * CPU Frequency: 16MHz
 * Timer Prescaler: 8
 *
 * Timer Frequency: 2MHz
 * Timer Resolution: 0.5us
 * Compare Match Interval: 1ms
 */

/* Protect the header file from multiple inclusions */
 #ifndef T1_H_
 #define T1_H_

#include <stdint.h>
#include "driverStatus.h"

class T1 {
public:
    /// Create the software object without starting hardware; call init() during setup.
    T1() = default;
  
    /// Configure and start Timer1 with 0.5 us ticks and a 1 ms interrupt; reset elapsed time to
    /// zero.
    void init();

    /// Stop Timer1 and its interrupt, disconnect timer outputs and reset the clock to zero.
    void close();

    /// Return whether the clock is running; reading status does not change time.
    DriverStatus getStatus() const;
    
    /// Read elapsed 0.5 us timer ticks using an atomic snapshot; return zero if closed.
    uint32_t get_count();
    
    /// Read elapsed whole microseconds using an atomic snapshot; return zero if closed.
    uint32_t get_micros();
    
    /// Wait for the requested milliseconds, blocking the caller; return immediately if the
    /// timer is closed.
    void delay(uint16_t ms); 

}; 

/* Header protection close */
#endif

/* END OF FILE */
