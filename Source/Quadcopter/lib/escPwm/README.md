# ESC PWM driver

`escPwm` owns Timer0 and Timer2 on the 16 MHz ATmega328P. The interface is
`init()`, `config()`, `write_motor_1()` through `write_motor_4()`, and `close()`.
`write(motor1Us, motor2Us, motor3Us, motor4Us)` is also available for a group update.

This driver sends throttle command pulses to the ESCs. The ESCs drive the motor
windings. A command such as `write_motor_1(1200)` asks for a 1200 us high pulse;
it does not request 1200 RPM or set the motor supply voltage directly.

## Reading the source

Start with `escPwm.h` for the application calls. `escPwm.cpp` follows the same
register-based approach as the original `Motors.cpp`, with the pin mapping and
timing calculation at the top and register bit layouts beside initialization.

| Source section | What it explains |
| --- | --- |
| Constants | Pulse limits and masks selecting only the four output pins |
| `pulseToCompare()` | Clamp the requested microseconds, round and convert to OCR |
| `init()` steps 1..6 | Stop timers, prepare pins, clear latches, load widths, select mode, start clocks |
| `config()` | Send the minimum pulse on every channel for initial ESC arming |
| Individual writes | Select one compare register and leave other channels alone |
| Group `write()` | Convert and write all four motor commands |
| `close()` | Stop the clocks, disconnect PWM and hold the GPIO outputs low |

The unnamed `namespace` makes constants and the conversion helper private to
the source file. `constexpr` defines a constant value known at compile time.
`initialized_` records whether this software has prepared the hardware; it does
not report whether an ESC has armed.

## Wiring and application calls

| Motor | Arduino pin | Hardware output | Compare register |
| --- | --- | --- | --- |
| 1 | D3 | OC2B | OCR2B |
| 2 | D5 | OC0B | OCR0B |
| 3 | D6 | OC0A | OCR0A |
| 4 | D11 | OC2A | OCR2A |

```cpp
#include "escPwm.h"

escPwm esc; // One instance owns all four outputs.

// In setup(): prepare hardware, then explicitly begin initial ESC arming.
esc.init();   // Outputs stay low.
esc.config(); // All four channels send 1000 us pulses.

// In the control task, after mixing the four motor commands:
esc.write_motor_1(1000); // D3, pulse width in microseconds.
esc.write_motor_2(1200); // D5.
esc.write_motor_3(1400); // D6.
esc.write_motor_4(1600); // D11.

// When stopping PWM output:
esc.close();
```

`init()` configures the timers with zero compare values, holding all outputs low.
`config()` sends minimum pulses on all four channels to begin ESC arming, matching
the original `config_motors()` value of OCR 125. Hardware keeps sending these
pulses without further calls. Like the original, config does not wait for or
confirm ESC readiness, and does not perform throttle endpoint calibration.
Allow the ESC startup procedure to finish before increasing commands. Calling
config again resets all four commands to minimum.

Repeated init calls leave the current commands alone. Every write clamps its
unsigned input to 1000..2000 us and rounds to the nearest 8 us (half steps round
up). Individual writes change only the selected channel. A write after init can
also start pulses without config; config is the explicit all-channel startup step.
Config and writes before init or after close do nothing. `close()` stops the timers,
disconnects PWM and keeps the pins configured as outputs driving low; it does
not send minimum pulses or restore Arduino's timer configuration. A later init
holds outputs low again until config or a write.

The intended application sequence is:

1. Construct one `escPwm` object; this performs no hardware writes.
2. Call `init()` in setup to prepare the timers and hold all channels low.
3. Call `config()` to begin repeating minimum-throttle pulses on all channels.
4. Allow ESC startup to finish before issuing higher commands. This driver
   neither measures startup time nor receives an armed confirmation.
5. In the control task, write each motor's command after mixing throttle and PID
   corrections. Between calls, hardware keeps repeating the previous command.
6. Call `close()` when removing the command signal. This stops a pulse immediately
   if one is in progress; it is different from commanding minimum throttle.

`src/main.cpp` currently performs steps 1..3. The higher-throttle writes remain
commented because receiver input, arming decisions, PID and mixing are unfinished.

## Timing

Like the original project, both timers use non-inverting phase-correct PWM with
TOP 255 and prescaler 64. At 16 MHz each counter tick is 4 us. Counting up and
down gives a 2040 us period (about 490 Hz), and the high pulse lasts `OCR * 8 us`.
Thus 1000 us is OCR 125, 1500 us rounds to 1504 us (OCR 188), and 2000 us is
OCR 250. The old empirical mapping of 135..238 is replaced by this direct
microsecond conversion.

For example, `esc.write_motor_1(1200)` follows this path:

1. Confirm that init has run; otherwise return without touching hardware.
2. Keep 1200 within the permitted 1000..2000 us range.
3. Calculate `(1200 + 4) / 8`, giving the integer compare value 150.
4. Write 150 to OCR2B, the compare register for motor 1 on D3.
5. Timer2 loads the buffered value at TOP. Its output then produces a
   `150 * 8 = 1200 us` high pulse every 2040 us until another command is written.

The frequency is how often a pulse repeats. Pulse width carries the command.
This is why a 1000 us command is not a 0% electrical duty cycle: it is still a
high pulse during part of each frame, which a calibrated ESC can interpret as
minimum throttle. OCR=0, used by init, instead produces no high pulse at all.

The hardware repeats the latest command without CPU intervention. It applies
new compare values at TOP without changing a pulse halfway through. The timers
run independently: one write does not promise simultaneous updates on all
four pins. Writes do not reset either counter. No automatic timeout, ESC readiness
detection, receiver handling or motor mixing is provided by this driver.

## Register names and bit operations

Here `n` means timer number 0 or 2, and `x` means channel A or B.

| Register or field | Purpose in this driver |
| --- | --- |
| `DDRB`, `DDRD` | Data direction: set the four output pin bits to 1 |
| `PORTB`, `PORTD` | Store low GPIO levels used when PWM is disconnected |
| `TCCRnA` | Choose compare output behaviour and the low waveform mode bits |
| `TCCRnB` | Choose the clock prescaler and the remaining waveform mode bit |
| `COMnA`, `COMnB` | Connect hardware compare outputs; value `10` selects non-inverting PWM |
| `WGMn2:n0` | Waveform Generation Mode; `001` selects phase-correct PWM, TOP=255 |
| `CSn2:n0` | Clock Select; `011` for Timer0 and `100` for Timer2 both select /64 |
| `FOCnA`, `FOCnB` | Force compare actions in non-PWM mode to clear old internal output levels |
| `TCNTn` | Current counter value, reset only by init rather than on every write |
| `OCRnx` | Output Compare Register holding one channel's pulse width setting |
| `TIMSKn` | Interrupt enables, all disabled because PWM generation needs no ISR |
| `TIFRn` | Pending interrupt flags, cleared by writing ones to the selected flags |

`(1 << PD3)` selects bit 3. `|` joins selected bits into a mask.
`DDRD |= mask` sets only those direction bits, preserving other pins.
`PORTD &= ~mask` clears only those output levels. Full assignments to timer
control registers are deliberate because this driver owns both timers.

`ATOMIC_BLOCK(ATOMIC_RESTORESTATE)` temporarily prevents interrupt handlers from
running during a group of register changes, then restores the previous interrupt
state. Counters still run during a group write, so this does not synchronize the
two timers or guarantee one shared update boundary.

## Relationship to the original motor driver

The motor-to-pin mapping and phase-correct PWM timer settings are retained.
The old `PWM1` through `PWM4` macros named the same four compare registers now
shown directly in each individual write function. `config()` uses the original
arming assignment of OCR 125, corresponding to 1000 us with these timer settings.

The pulse conversion is different: the original `radio2motor()` mapped command
1000..2000 into OCR 135..238, corresponding to approximately 1080..1904 us after
integer conversion. The current API instead accepts the desired pulse width
directly. Existing ESC calibration must be considered when using the new endpoints.

## Hardware ownership and verification

Timer1 and its prescaler are left alone, so `T1::get_micros()` still provides
the project clock. Arduino `millis()`, `micros()` and `delay()` no longer work
normally once this driver takes Timer0. Do not combine it with `analogWrite()`
on these pins, `tone()`, SPI on D11, or other Timer0/Timer2 owners. Timer2 must
be using its normal synchronous CPU clock, as it does after Arduino startup.

The approximately 490 Hz rate comes from the original timer configuration. The
stored Turnigy Plush manual does not establish that this input rate is supported,
so it remains unverified for the fitted ESCs. Their input rate and calibration
are hardware requirements. Check pulse
widths and startup/stop behaviour on a scope or logic analyser before flight.

Register settings and timing follow the ATmega328P datasheet, sections 14.7.4
and 17.7.4:
[Microchip ATmega328P datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf).
