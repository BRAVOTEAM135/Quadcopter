#pragma once

#include <stddef.h>
#include <stdint.h>

#define SERIAL_TX_BUFFER_SIZE 64

// Minimal UART fixture for checking the SerialPort lifecycle and delegation.
struct TestSerial {
    bool enabled;
    uint32_t baud;
    uint16_t closeCalls;
    uint16_t writtenBytes;
    int freeBytes;
    int receivedBytes;

    /// Simulate enabling UART at the requested baud rate.
    void begin(uint32_t rate) { enabled = true; baud = rate; }
    /// Simulate UART shutdown, discard received bytes and count close calls.
    void end() { enabled = false; receivedBytes = 0; ++closeCalls; }
    /// Return the simulated free transmit-buffer capacity.
    int availableForWrite() { return freeBytes; }
    /// Count queued bytes and report the entire sequence as written.
    size_t write(const uint8_t*, size_t length) {
        writtenBytes += length;
        return length;
    }
    /// Return the simulated number of received bytes waiting to be read.
    int available() { return receivedBytes; }
    /// Consume a simulated received byte, or return -1 when none remains.
    int read() { return receivedBytes ? (--receivedBytes, 'A') : -1; }
};

extern TestSerial Serial;
