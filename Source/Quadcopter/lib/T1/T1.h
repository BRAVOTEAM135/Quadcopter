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

class T1 {
public:
    T1() = default;
  
    // Initialise Timer1
    void init();
    
    // Get elapsed timer ticks (0.5us per tick)
    uint32_t get_count();
    
    // Get elapsed time in whole microseconds
    uint32_t get_micros();
    
    // Blocking delay in milliseconds
    void delay(uint16_t ms); 

}; 

/* Header protection close */
#endif

/* END OF FILE */
