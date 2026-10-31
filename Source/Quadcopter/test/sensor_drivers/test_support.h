#pragma once

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "i2cBus.h"
#include "GY80.h"
#include "MPU6050.h"
#include "BMP085.h"
#include <math.h>

volatile uint8_t TWCR, TWSR, TWBR, TWDR;
volatile uint16_t tests_run = 0;
volatile uint16_t failed_line = 0;

/// Provide the debugger breakpoint used to report completion or the first failed check.
__attribute__((noinline)) void test_done() {
    asm volatile("nop");
}

/// Count an assertion and stop at test_done() with its source line when it fails.
void check(bool ok, uint16_t line) {
    ++tests_run;
    if (!ok) {
        failed_line = line;
        test_done();
        for (;;) {}
    }
}
#define CHECK(condition) check((condition), __LINE__)

i2cBus bus;
uint32_t nowUs = 0;

/// Inject a simulated TWI status and data byte, then run the actual driver interrupt handler.
void interrupt(uint8_t status, uint8_t value = 0) {
    TWSR = status;
    TWDR = value;
    test_twi_interrupt();
}

bool serviceBusInHelper = true;

/// Simulate hardware completing STOP and optionally service the bus.
void finishStop() {
    CHECK(TWCR & (1 << TWSTO));
    TWCR &= ~(1 << TWSTO); // Model hardware completing STOP.
    if (serviceBusInHelper) bus.service(nowUs);
}

/// Simulate START, write address and register selection, checking transmitted bytes.
void registerPrefix(uint8_t address, uint8_t reg) {
    CHECK(TWCR & (1 << TWSTA));
    interrupt(0x08);
    CHECK(TWDR == static_cast<uint8_t>(address << 1));
    interrupt(0x18);
    CHECK(TWDR == reg);
    interrupt(0x28);
}

/// Simulate a complete register read, checking address, repeated START and ACK/NACK sequencing.
void completeRead(uint8_t address, uint8_t reg, const uint8_t* data, uint8_t length) {
    registerPrefix(address, reg);
    CHECK(TWCR & (1 << TWSTA));
    interrupt(0x10);
    CHECK(TWDR == static_cast<uint8_t>((address << 1) | 1));
    interrupt(0x40);
    for (uint8_t i = 0; i < length; ++i) {
        CHECK(bool(TWCR & (1 << TWEA)) == (i + 1 < length));
        interrupt(i + 1 == length ? 0x58 : 0x50, data[i]);
    }
    finishStop();
}

/// Simulate a register write and STOP, checking its address, register and value.
void completeWrite(uint8_t address, uint8_t reg, uint8_t value) {
    registerPrefix(address, reg);
    CHECK(TWDR == value);
    interrupt(0x28);
    finishStop();
}

/// Return true when two floating-point values differ by less than the chosen tolerance.
bool near(float actual, float expected, float tolerance = 0.001f) {
    float difference = actual - expected;
    return difference > -tolerance && difference < tolerance;
}
