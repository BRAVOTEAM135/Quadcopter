#include "test_support.h"
#include "T1.h"
#include "ledBlink.h"
#include "serialPort.h"

volatile uint8_t DDRB, PORTB, PINB;
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

/// Run this file's simulator checks, then stop at the debugger completion breakpoint.
int main() {
    testTimer();
    testLed();
    testSerial();
    test_done();
    for (;;) {}
}
