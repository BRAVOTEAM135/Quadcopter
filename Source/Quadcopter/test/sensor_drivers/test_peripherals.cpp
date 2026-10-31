#include "test_support.h"
#include "T1.h"
#include "ledBlink.h"
#include "serialPort.h"
#include "escPwm.h"

volatile uint8_t DDRB, PORTB, PINB;
volatile uint8_t DDRD, PORTD;
volatile uint8_t TCCR0A, TCCR0B, TIMSK0, TIFR0, TCNT0, OCR0A, OCR0B;
volatile uint8_t TCCR2A, TCCR2B, TIMSK2, TIFR2, TCNT2, OCR2A, OCR2B;
volatile uint8_t SREG, TCCR1A, TCCR1B, TIMSK1, TIFR1;
volatile uint16_t TCNT1, OCR1A;
TestSerial Serial;

/// Check timer initialization, elapsed time, closure and restart from zero.
void testTimer() {
    T1 timer;
    CHECK(!timer.getStatus().ready);
    CHECK(timer.get_micros() == 0);
    timer.close();
    timer.close();
    timer.delay(1); // Closing a clock must not leave delay() stuck forever.
    CHECK(timer.get_count() == 0 && timer.get_micros() == 0);

    SREG = 0x80;
    timer.init();
    CHECK(timer.getStatus().ready && !timer.getStatus().busy);
    CHECK(SREG == 0x80);
    CHECK(TIMSK1 & (1 << OCIE1A));
    CHECK(TCCR1B != 0);
    TIFR1 = 0; // Model the hardware clearing write-one-to-clear flags.
    test_timer1_interrupt();
    TCNT1 = 600;
    CHECK(timer.get_count() == 2600 && timer.get_micros() == 1300);
    timer.close();
    CHECK(TCCR1B == 0 && TIMSK1 == 0);
    CHECK(!timer.getStatus().ready);
    CHECK(timer.get_count() == 0 && timer.get_micros() == 0);
    timer.delay(1);

    timer.init();
    TIFR1 = 0;
    TCNT1 = 200;
    CHECK(timer.get_micros() == 100); // The new clock starts from zero.
    timer.close();
}

/// Check LED lifecycle and preservation of unrelated port pins.
void testLed() {
    ledBlink led;
    CHECK(!led.getStatus().ready);
    DDRB = 1; // Another pin must survive this driver's lifecycle.
    PORTB = 1;
    led.close();
    led.ledOn();
    CHECK(PORTB == 1 && DDRB == 1);
    led.init();
    CHECK(led.getStatus().ready && !led.getStatus().busy);
    led.ledOn();
    CHECK(PORTB == (1 | (1 << PORTB5)));
    led.close();
    CHECK(PORTB == 1 && DDRB == 1);
    CHECK(!led.getStatus().ready);
    PINB = 0;
    led.ledOn();
    led.ledToggle();
    CHECK(PORTB == 1 && PINB == 0);
    led.close();
    led.init();
    led.ledOn();
    CHECK(DDRB & (1 << DDB5));
    CHECK(PORTB & (1 << PORTB5));
    led.close();
}

/// Check UART delegation, inactive read/write behaviour after close and reinitialization.
void testSerial() {
    SerialPort port;
    CHECK(!port.getStatus().ready && !port.getStatus().busy);
    Serial.freeBytes = 64;
    port.close();
    CHECK(Serial.closeCalls == 0);
    CHECK(!port.print("closed") && Serial.writtenBytes == 0);
    CHECK(port.available() == 0 && port.read() == -1);
    port.init(115200);
    CHECK(port.getStatus().ready && !port.getStatus().busy);
    Serial.freeBytes = 0;
    CHECK(port.getStatus().busy);
    Serial.freeBytes = 64;
    CHECK(Serial.enabled && Serial.baud == 115200);
    CHECK(port.print("abc") && Serial.writtenBytes == 3);
    Serial.receivedBytes = 2;
    CHECK(port.available() == 2 && port.read() == 'A');
    port.close();
    CHECK(!Serial.enabled && Serial.closeCalls == 1);
    CHECK(!port.getStatus().ready && !port.getStatus().busy);
    CHECK(!port.println("closed") && Serial.writtenBytes == 3);
    CHECK(port.available() == 0 && port.read() == -1);
    port.close();
    CHECK(Serial.closeCalls == 1);
    port.init(9600);
    CHECK(Serial.enabled && Serial.baud == 9600);
    CHECK(port.println("x") && Serial.writtenBytes == 6);
    port.close();
}

/// Check motor pulse units, pin order, lifecycle and coexistence with the Timer1 clock.
void testEscPwm() {
    escPwm esc;
    T1 timer;
    timer.init();
    TIFR1 = 0;
    TCNT1 = 600;
    DDRD = PORTD = 1; // D0 belongs to another peripheral.
    DDRB = PORTB = (1 << PB3) | 1; // D8 belongs to another peripheral.
    OCR2B = 7;
    OCR0B = 8;
    OCR0A = 9;
    OCR2A = 10;
    esc.write(2000, 2000, 2000, 2000);
    esc.write_motor_1(2000);
    esc.write_motor_2(2000);
    esc.write_motor_3(2000);
    esc.write_motor_4(2000);
    esc.config();
    esc.close();
    CHECK(OCR2B == 7 && OCR0B == 8 && OCR0A == 9 && OCR2A == 10);

    SREG = 0x80;
    esc.init();
    CHECK(SREG == 0x80);
    CHECK(DDRD == (1 | (1 << PD3) | (1 << PD5) | (1 << PD6)));
    CHECK(DDRB == (1 | (1 << PB3)));
    CHECK(PORTD == 1 && PORTB == 1);
    CHECK(OCR2B == 0 && OCR0B == 0 && OCR0A == 0 && OCR2A == 0);
    CHECK(TCCR0A == ((1 << COM0A1) | (1 << COM0B1) | (1 << WGM00)));
    CHECK(TCCR2A == ((1 << COM2A1) | (1 << COM2B1) | (1 << WGM20)));
    CHECK(TCCR0B == ((1 << CS01) | (1 << CS00)));
    CHECK(TCCR2B == (1 << CS22));
    CHECK(TIMSK0 == 0 && TIMSK2 == 0);
    CHECK(TCNT0 == 0 && TCNT2 == 0);
    CHECK(TCCR1A == 0 && TCCR1B == ((1 << WGM12) | (1 << CS11)));
    CHECK(TIMSK1 == (1 << OCIE1A) && OCR1A == 1999);
    CHECK(timer.get_micros() == 300);

    esc.config();
    CHECK(OCR2B == 125 && OCR0B == 125 && OCR0A == 125 && OCR2A == 125);
    esc.write_motor_1(1200);
    CHECK(OCR2B == 150 && OCR0B == 125 && OCR0A == 125 && OCR2A == 125);
    esc.write_motor_2(1400);
    CHECK(OCR2B == 150 && OCR0B == 175 && OCR0A == 125 && OCR2A == 125);
    esc.write_motor_3(1600);
    CHECK(OCR2B == 150 && OCR0B == 175 && OCR0A == 200 && OCR2A == 125);
    esc.write_motor_4(2000);
    CHECK(OCR2B == 150 && OCR0B == 175 && OCR0A == 200 && OCR2A == 250);
    esc.write_motor_1(0);
    esc.write_motor_2(65535);
    esc.write_motor_3(1004);
    esc.write_motor_4(1500);
    CHECK(OCR2B == 125 && OCR0B == 250 && OCR0A == 126 && OCR2A == 188);

    TCNT0 = 40;
    TCNT2 = 90;
    esc.config(); // Repeating config resets every command without resetting time.
    CHECK(OCR2B == 125 && OCR0B == 125 && OCR0A == 125 && OCR2A == 125);
    CHECK(TCNT0 == 40 && TCNT2 == 90);
    esc.write(1000, 1200, 1600, 2000);
    CHECK(OCR2B == 125 && OCR0B == 150 && OCR0A == 200 && OCR2A == 250);
    CHECK(TCNT0 == 40 && TCNT2 == 90); // Writing must not restart a pulse.
    esc.init();
    CHECK(OCR0B == 150 && OCR0A == 200 && OCR2A == 250); // Init is idempotent.
    CHECK(TCNT0 == 40 && TCNT2 == 90);

    esc.write(0, 999, 2001, 65535);
    CHECK(OCR2B == 125 && OCR0B == 125 && OCR0A == 250 && OCR2A == 250);
    esc.write(1003, 1004, 1500, 1999);
    CHECK(OCR2B == 125 && OCR0B == 126 && OCR0A == 188 && OCR2A == 250);

    // Every individual channel must produce a bounded width within 4 us.
    // Other channels keep their previous commands until explicitly written.
    for (uint16_t pulse = 1000; pulse <= 2000; ++pulse) {
        esc.write_motor_1(pulse);
        int16_t errorUs = static_cast<int16_t>(OCR2B * 8) - pulse;
        CHECK(errorUs >= -4 && errorUs <= 4);
        CHECK(OCR2B >= 125 && OCR2B <= 250);
        uint8_t motor1 = OCR2B;
        esc.write_motor_2(pulse);
        CHECK(OCR0B == motor1);
        esc.write_motor_3(pulse);
        CHECK(OCR0A == motor1);
        esc.write_motor_4(pulse);
        CHECK(OCR2A == motor1 && OCR2B == motor1);
    }

    SREG = 0;
    esc.write(1200, 1200, 1200, 1200);
    CHECK(SREG == 0);
    esc.close();
    CHECK(SREG == 0);
    CHECK(TCCR0A == 0 && TCCR0B == 0 && TIMSK0 == 0);
    CHECK(TCCR2A == 0 && TCCR2B == 0 && TIMSK2 == 0);
    CHECK(PORTD == 1 && PORTB == 1);
    CHECK(DDRD == (1 | (1 << PD3) | (1 << PD5) | (1 << PD6)));
    CHECK(DDRB == (1 | (1 << PB3)));
    esc.write(2000, 2000, 2000, 2000);
    esc.write_motor_1(2000);
    esc.write_motor_2(2000);
    esc.write_motor_3(2000);
    esc.write_motor_4(2000);
    esc.config();
    esc.close();
    CHECK(OCR2B == 150 && OCR0B == 150 && OCR0A == 150 && OCR2A == 150);
    CHECK(timer.getStatus().ready);
    test_timer1_interrupt();
    CHECK(timer.get_micros() == 1300); // Closing PWM leaves Timer1 running.

    esc.init();
    CHECK(OCR2B == 0 && OCR0B == 0 && OCR0A == 0 && OCR2A == 0);
    CHECK(TCCR0B != 0 && TCCR2B != 0);
    esc.config();
    CHECK(OCR2B == 125 && OCR0B == 125 && OCR0A == 125 && OCR2A == 125);
    esc.close();
    timer.close();
}

/// Run this file's simulator checks, then stop at the debugger completion breakpoint.
int main() {
    testTimer();
    testLed();
    testSerial();
    testEscPwm();
    test_done();
    for (;;) {}
}
