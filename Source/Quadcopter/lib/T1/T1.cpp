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

#include "T1.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>


/*----- Constants -----*/
namespace {

    constexpr uint16_t T1_TICKS_PER_MS = 2000;
    constexpr uint16_t T1_COMPARE_VALUE = T1_TICKS_PER_MS - 1;

    /*----- Module Local Variables -----*/
    volatile uint32_t t1Milliseconds = 0;

    /*----- Internal Structures -----*/
    struct T1Snapshot {
        uint32_t milliseconds;
        uint16_t timerTicks;
    };

    /*----- Internal Functions -----*/
    T1Snapshot read_snapshot() {
        T1Snapshot snapshot;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            // Read the software millisecond counter
            snapshot.milliseconds = t1Milliseconds;

            // Read the current hardware timer value
            snapshot.timerTicks = TCNT1;

            // Check for a pending compare match
            bool pendingMatch = (TIFR1 & (1 << OCF1A)) != 0;

            // Account for an interrupt that has not yet executed
            if (pendingMatch &&
                snapshot.timerTicks < (T1_TICKS_PER_MS / 2)) {
                snapshot.milliseconds++;
            }
        }
        return snapshot;
    }
}


/*----- Interrupt Service Routine -----*/

// Timer1 Compare Match A interrupt
// Executes every 1ms

ISR(TIMER1_COMPA_vect) {

    t1Milliseconds++;

}


/*----- Global Functions -----*/

void T1::init() {

    // Save current interrupt state
    uint8_t oldSreg = SREG;

    // Disable interrupts
    cli();

    // Stop Timer1
    TCCR1B = 0;

    // Clear Timer1 control registers
    TCCR1A = 0;

    // Disable Timer1 interrupts
    TIMSK1 = 0;

    // Reset hardware counter
    TCNT1 = 0;

    // Reset software counter
    t1Milliseconds = 0;

    // Set compare match value
    OCR1A = T1_COMPARE_VALUE;

    // Clear pending compare match interrupt flag
    TIFR1 = (1 << OCF1A);

    // Enable Timer1 Compare Match A interrupt
    TIMSK1 = (1 << OCIE1A);

    // Configure CTC mode with prescaler of 8
    TCCR1B = (1 << WGM12) | (1 << CS11);

    // Restore previous interrupt state
    SREG = oldSreg;
}


uint32_t T1::get_count() {

    T1Snapshot snapshot = read_snapshot();

    // Calculate total elapsed timer ticks
    uint32_t ticks =
        (snapshot.milliseconds * T1_TICKS_PER_MS)
        + snapshot.timerTicks;

    return ticks;
}


uint32_t T1::get_micros() {

    T1Snapshot snapshot = read_snapshot();

    // Convert milliseconds to microseconds
    uint32_t micros =
        (snapshot.milliseconds * 1000UL)
        + (snapshot.timerTicks / 2);

    return micros;
}


void T1::delay(uint16_t ms) {

    // Record starting time
    uint32_t start = get_micros();

    // Convert delay to microseconds
    uint32_t duration = (uint32_t)ms * 1000UL;

    // Wait until requested duration has elapsed
    while ((uint32_t)(get_micros() - start) < duration) {

        // Blocking delay
    }
}

/* END OF FILE */