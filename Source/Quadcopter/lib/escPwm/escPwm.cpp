/*
 * ESC PWM driver - ATmega328P, 16 MHz
 * ----------------------------------
 * Timer0 and Timer2 each have two hardware outputs. Together they generate
 * the four command signals sent to the Electronic Speed Controllers (ESCs).
 * The ESCs supply power to the motors; these pins only send throttle commands.
 * Timer1 remains available to the T1 driver for the application clock.
 *
 * Motor | Arduino pin | AVR pin | Timer output | Width register
 * ------|-------------|---------|--------------|---------------
 *   1   |     D3      |   PD3   |     OC2B     |    OCR2B
 *   2   |     D5      |   PD5   |     OC0B     |    OCR0B
 *   3   |     D6      |   PD6   |     OC0A     |    OCR0A
 *   4   |     D11     |   PB3   |     OC2A     |    OCR2A
 *
 * Application calls:
 *   init()   -> prepare hardware; outputs stay low, without throttle pulses.
 *   config() -> send minimum pulses on all channels for initial ESC arming.
 *   write_motor_1() ... write_motor_4() -> change one channel's pulse width.
 *   close()  -> stop pulses immediately and hold the pins low.
 *
 * PWM timing:
 *   Mode: phase-correct PWM, counting 0 -> 255 -> 0 (510 counter ticks).
 *   Prescaler: divide the 16 MHz CPU clock by 64.
 *   Counter tick = 64 / 16 MHz = 4 us.
 *   Frame period = 510 * 4 us = 2040 us, approximately 490 Hz.
 *   High pulse width = 2 * OCR * 4 us = OCR * 8 us.
 *
 * The high pulse length carries the throttle command. The frame period stays
 * fixed as throttle changes; 1000..2000 are microseconds, not percentages.
 * This rate follows the original timer setup. Compatibility with the fitted
 * ESCs, and their calibrated throttle endpoints, still need hardware verification.
 *
 * Datasheet reference: ATmega328P, Timer0/Timer2 phase-correct PWM sections.
 */

/*----- Includes -----*/
#include "escPwm.h"

#include <avr/io.h>
#include <util/atomic.h>

#if F_CPU != 16000000UL
#error "escPwm PWM requires a 16 MHz CPU clock."
#endif

/*----- Constants and helper function, private to this source file -----*/
// The unnamed namespace keeps these names local to this driver.
namespace {
    // Command limits in microseconds. config() uses the minimum for arming.
    constexpr uint16_t MIN_PULSE_US = 1000;
    constexpr uint16_t MAX_PULSE_US = 2000;

    // Pin masks: a 1 marks a pin owned by this driver. Other port pins are preserved.
    // (1 << PD3), for example, places a 1 in bit 3: 00001000.
    constexpr uint8_t PORT_D_OUTPUTS = (1 << PD3) | (1 << PD5) | (1 << PD6);
    constexpr uint8_t PORT_B_OUTPUTS = (1 << PB3);

    /*
     * Convert a pulse width in microseconds into an 8-bit OCR register value.
     * Each step adds 8 us because phase-correct PWM counts in both directions.
     *
     * Requested width | OCR value | Actual high pulse
     * ----------------|-----------|------------------
     *     1000 us     |    125    |     1000 us
     *     1500 us     |    188    |     1504 us
     *     2000 us     |    250    |     2000 us
     *
     * Clamp first so the result stays in range. Adding 4 before dividing by 8
     * rounds to the nearest step rather than always rounding down.
     * static_cast<uint8_t> converts the result to the OCR register's 8-bit type.
     */
    uint8_t pulseToCompare(uint16_t pulseUs) {
        if (pulseUs < MIN_PULSE_US) pulseUs = MIN_PULSE_US;
        if (pulseUs > MAX_PULSE_US) pulseUs = MAX_PULSE_US;
        return static_cast<uint8_t>((pulseUs + 4) / 8);
    }
}

/*----- Public functions -----*/

/*
 * Prepare all four hardware PWM channels, initially held low.
 * Call once from setup(), before config(). Calling again while open does nothing,
 * so it does not unexpectedly reset a running motor command.
 */
void escPwm::init() {
    if (initialized_) return;

    // Temporarily disable interrupts while changing related registers together.
    // ATOMIC_RESTORESTATE restores the previous interrupt state on leaving the block.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // 1. Stop the timer clocks before changing modes and initial widths.
        // Clearing TCCRnB clears its CS (clock select) bits, stopping that counter.
        TCCR0B = 0;
        TCCR2B = 0;

        // TIMSKn: Timer Interrupt Mask Register. A 1 enables an interrupt.
        // Bit:      7  6  5  4  3     2        1       0
        // TIMSK0:   -  -  -  -  -   OCIE0B   OCIE0A   TOIE0
        // TIMSK2:   -  -  -  -  -   OCIE2B   OCIE2A   TOIE2
        // Zero disables compare/overflow interrupts; hardware still generates PWM.
        // This also disables Arduino's Timer0 clock interrupt. Use T1 for timing.
        TIMSK0 = 0;
        TIMSK2 = 0;

        // Disconnect the compare outputs and clear the old waveform mode.
        TCCR0A = 0;
        TCCR2A = 0;

        // 2. Prepare the output pins. PORT stores the ordinary GPIO output level;
        // DDR (Data Direction Register) selects input=0 or output=1.
        // &= ~MASK clears just our bits; |= MASK sets just our bits.
        // Write the low level BEFORE making pins outputs, avoiding a GPIO high.
        PORTD &= ~PORT_D_OUTPUTS;
        PORTB &= ~PORT_B_OUTPUTS;
        DDRD |= PORT_D_OUTPUTS;
        DDRB |= PORT_B_OUTPUTS;

        // 3. Clear the timers' internal output latches as well as the GPIO latches.
        // COMnA1/COMnB1 select "clear output on compare" in this non-PWM mode.
        // FOC (Force Output Compare) triggers that action without running the timer.
        // These strobe bits act on write; this temporary setting is replaced below.
        // Clearing here prevents a leftover high output when restarting after close().
        TCCR0A = (1 << COM0A1) | (1 << COM0B1);
        TCCR0B = (1 << FOC0A) | (1 << FOC0B);
        TCCR2A = (1 << COM2A1) | (1 << COM2B1);
        TCCR2B = (1 << FOC2A) | (1 << FOC2B);

        // 4. OCR means Output Compare Register: it stores each channel's width.
        // Load values before enabling PWM mode, while the registers are unbuffered.
        // OCR=0 in non-inverting phase-correct PWM means continuously low.
        // It is different from a 1000 us minimum-throttle pulse; config() sends those.
        OCR2B = 0; // Motor 1: D3.
        OCR0B = 0; // Motor 2: D5.
        OCR0A = 0; // Motor 3: D6.
        OCR2A = 0; // Motor 4: D11.
        // TCNT is the current timer count. Begin both counting sequences at zero.
        TCNT0 = 0;
        TCNT2 = 0;

        // TIFR: Timer Interrupt Flag Register. OCF=compare flag, TOV=overflow flag.
        // These flags are cleared by WRITING A 1, not by writing zero.
        // Clear any events left pending from the previous timer configuration.
        TIFR0 = (1 << OCF0A) | (1 << OCF0B) | (1 << TOV0);
        TIFR2 = (1 << OCF2A) | (1 << OCF2B) | (1 << TOV2);

        /*
         * 5. Set waveform generation and connect both outputs on each timer.
         * TCCRnA = Timer/Counter Control Register A.
         *
         * Bit:      7       6       5       4     3  2    1      0
         * TCCR0A: COM0A1 COM0A0 COM0B1 COM0B0   -  -  WGM01  WGM00
         * TCCR2A: COM2A1 COM2A0 COM2B1 COM2B0   -  -  WGM21  WGM20
         * Value:    1       0       1       0     0  0    0      1
         *
         * COMnA/B=10: non-inverting output. Clear on compare while counting UP,
         * and set on compare while counting DOWN. The high pulse straddles zero.
         * WGMn2:n0=001: mode 1, phase-correct PWM with TOP fixed at 255.
         * WGMn2 is in TCCRnB and remains zero when the clock is started below.
         */
        TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00);
        TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20);

        /*
         * 6. Start the timers by choosing their clock prescalers.
         * TCCRnB = Timer/Counter Control Register B.
         *
         * Bit:      7      6     5  4     3     2     1     0
         * TCCR0B: FOC0A  FOC0B   -  -   WGM02  CS02  CS01  CS00
         * Value:    0      0     0  0     0     0     1     1
         * TCCR2B: FOC2A  FOC2B   -  -   WGM22  CS22  CS21  CS20
         * Value:    0      0     0  0     0     1     0     0
         *
         * The clock-select encodings differ: Timer0 uses 011 for /64;
         * Timer2 uses 100 for /64. Both give 490.196 Hz with this PWM mode.
         * Do not reset the shared Timer0/Timer1 prescaler; T1 is already running.
         */
        TCCR0B = (1 << CS01) | (1 << CS00);
        TCCR2B = (1 << CS22);
        initialized_ = true; // Software is ready; this does not mean ESCs are armed.
    }
}

/*
 * Begin initial ESC arming after init().
 * Set ALL channels to minimum throttle, matching the original config_motors().
 * Returns immediately: the timers keep repeating pulses while the ESCs start up.
 * This driver has no feedback to confirm arming and does not calibrate endpoints.
 * Calling config() again returns all motor commands to minimum.
 */
void escPwm::config() {
    // Same OCR value of 125 used by the original config_motors(): 1000 us.
    // Hardware continues these minimum pulses while the ESCs perform startup.
    write(MIN_PULSE_US, MIN_PULSE_US, MIN_PULSE_US, MIN_PULSE_US);
}

/*
 * Individual channel writes
 * -------------------------
 * pulseUs is the requested high time in microseconds. Each function changes
 * only its own motor; the other three retain their previous commands.
 * pulseToCompare() handles limits and rounding. An 8-bit OCR write is a single
 * register access, so an interrupt cannot split it into a partially written value.
 * The timer buffers the value and applies it at TOP (count 255). The current
 * high pulse completes normally; the new command repeats until changed/closed.
 * Calls are ignored before init() or after close(). A write after init() can
 * start that channel even if config() has not been called.
 */

// Motor 1: Arduino D3 / PD3 / Timer2 output B.
void escPwm::write_motor_1(uint16_t pulseUs) {
    if (!initialized_) return;
    OCR2B = pulseToCompare(pulseUs);
}

// Motor 2: Arduino D5 / PD5 / Timer0 output B.
void escPwm::write_motor_2(uint16_t pulseUs) {
    if (!initialized_) return;
    OCR0B = pulseToCompare(pulseUs);
}

// Motor 3: Arduino D6 / PD6 / Timer0 output A.
void escPwm::write_motor_3(uint16_t pulseUs) {
    if (!initialized_) return;
    OCR0A = pulseToCompare(pulseUs);
}

// Motor 4: Arduino D11 / PB3 / Timer2 output A.
void escPwm::write_motor_4(uint16_t pulseUs) {
    if (!initialized_) return;
    OCR2A = pulseToCompare(pulseUs);
}

/*
 * Optional group write: accept four pulse widths, always in motor order 1..4.
 * Used by config() and available to a future mixer that produces four commands.
 * Convert before disabling interrupts, keeping the interrupt pause short.
 */
void escPwm::write(uint16_t motor1Us, uint16_t motor2Us,
                   uint16_t motor3Us, uint16_t motor4Us) {
    if (!initialized_) return;

    uint8_t motor1 = pulseToCompare(motor1Us);
    uint8_t motor2 = pulseToCompare(motor2Us);
    uint8_t motor3 = pulseToCompare(motor3Us);
    uint8_t motor4 = pulseToCompare(motor4Us);

    // Hardware applies each buffered width at that timer's next TOP.
    // No waiting, software pulse generation or timer interrupts are needed.
    // This block prevents a CPU interrupt between these register writes. It does
    // not stop the counters or guarantee that all four values latch in one frame.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        OCR2B = motor1;
        OCR0B = motor2;
        OCR0A = motor3;
        OCR2A = motor4;
    }
}

/*
 * Stop signal generation immediately; this may shorten the pulse in progress.
 * A LOW pin here means no throttle signal, rather than minimum-throttle pulses.
 * Keep pins as outputs driving low, and make subsequent config/write calls no-ops.
 * init() can prepare the driver again. Timer1 and unrelated GPIO pins are preserved.
 * The original Arduino Timer0 configuration is not restored on close().
 */
void escPwm::close() {
    if (!initialized_) return;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // Stop counter clocks and keep timer interrupts disabled.
        TCCR0B = 0;
        TCCR2B = 0;
        TIMSK0 = 0;
        TIMSK2 = 0;
        // Prepare GPIO low levels before handing the pins back from PWM to GPIO.
        PORTD &= ~PORT_D_OUTPUTS;
        PORTB &= ~PORT_B_OUTPUTS;
        TCCR0A = 0; // Disconnect PWM, leaving the port latches driving low.
        TCCR2A = 0;
        // Clear pending flags using the write-one-to-clear convention.
        TIFR0 = (1 << OCF0A) | (1 << OCF0B) | (1 << TOV0);
        TIFR2 = (1 << OCF2A) | (1 << OCF2B) | (1 << TOV2);
        initialized_ = false;
    }
}
