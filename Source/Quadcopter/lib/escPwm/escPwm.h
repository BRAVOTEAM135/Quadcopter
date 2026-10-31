#ifndef ESC_PWM_H
#define ESC_PWM_H

#include <stdint.h>

/*
 * ESC command outputs - application interface
 * -------------------------------------------
 * Create ONE escPwm object for the four hardware channels on a 16 MHz ATmega328P.
 * Construction does not touch hardware. Call init() during setup().
 *
 * Normal call order:
 *   init() -> config() -> allow ESC startup -> individual motor writes -> close()
 *
 * Motor 1 = D3, motor 2 = D5, motor 3 = D6, motor 4 = D11.
 * All write arguments are pulse widths in microseconds, not percentages or RPM.
 * The command range is 1000..2000 us, rounded to 8 us steps. The ESC's calibrated
 * endpoints determine the throttle it assigns to those pulse widths.
 *
 * The timers repeat the last command at approximately 490 Hz without service
 * calls or blocking delays. An ESC must support that input rate; the rate follows
 * the original project's timer settings and has not been verified on the ESCs.
 *
 * This driver owns Timer0/Timer2 and the four output pins. Use T1 for timekeeping;
 * Arduino millis(), micros() and delay() cannot provide normal timing after init().
 * No ESC readiness feedback, command timeout or throttle calibration is provided.
 * See escPwm.cpp for the register layouts and timing calculations.
 */
class escPwm {
public:
    /// Configure Timer0/Timer2 and output pins, initially held low without pulses.
    /// Repeated calls while open leave existing commands and timer counts alone.
    void init();

    /// Begin initial ESC arming by sending 1000 us pulses on all four outputs.
    /// Pulses repeat in hardware; this does not wait for or confirm ESC readiness.
    /// Call after init(). Repeating config() sets every channel back to minimum.
    /// It has no effect before init() or after close().
    void config();

    /// Stop both timers and hold all four output pins low. Safe to call repeatedly.
    /// This removes the throttle signal immediately; it does not send 1000 us pulses.
    /// Later config/write calls are ignored until init() is called again.
    void close();

    // Individual writes change only the named motor. Values are unsigned pulse
    // widths, clamped to 1000..2000 us and rounded to the nearest 8 us.
    // Writes return immediately; hardware applies the value at its next TOP.
    // Ignored before init()/after close(); a write after init() can start pulses
    // on its channel without config(). No function here waits for ESC startup.

    /// Set motor 1: Arduino D3 / Timer2 output B.
    void write_motor_1(uint16_t pulseUs);
    /// Set motor 2: Arduino D5 / Timer0 output B.
    void write_motor_2(uint16_t pulseUs);
    /// Set motor 3: Arduino D6 / Timer0 output A.
    void write_motor_3(uint16_t pulseUs);
    /// Set motor 4: Arduino D11 / Timer2 output A.
    void write_motor_4(uint16_t pulseUs);

    /// Set four pulse widths in microseconds, each clamped to 1000..2000 us.
    /// All writes round to the nearest 8 us and are ignored before init()/after close().
    /// Optional alternative to individual writes; updates need not latch simultaneously.
    void write(uint16_t motor1Us, uint16_t motor2Us,
               uint16_t motor3Us, uint16_t motor4Us);

private:
    bool initialized_ = false; // Tracks local hardware setup, not the ESCs' armed state.
};

#endif
