#include "serialPort.h"

#include <stdlib.h>
#include <string.h>


/*----- Initialise Serial -----*/

void SerialPort::init(uint32_t baudRate) {

    Serial.begin(baudRate);

}


/*----- Raw Data Transmission -----*/

// Queue raw bytes only when the entire message fits.

bool SerialPort::send(const uint8_t* data, size_t length) {

    // Nothing to transmit
    if (length == 0) {
        return true;
    }

    // Validate pointer
    if (data == nullptr) {
        return false;
    }

    // Check available transmission buffer space
    int freeBytes = Serial.availableForWrite();

    // Return immediately if insufficient space
    if (freeBytes < 0 ||
        length > static_cast<size_t>(freeBytes)) {

        return false;
    }

    // Write data without waiting for buffer space
    // Assumes only the main program writes to Serial
    return Serial.write(data, length) == length;

}


/*----- Text Transmission -----*/

// Transmit text without a newline

bool SerialPort::print(const char* text) {

    // Validate pointer
    if (text == nullptr) {
        return false;
    }

    // Transmit text
    return send(
        reinterpret_cast<const uint8_t*>(text),
        strlen(text)
    );

}


/*----- Text Transmission With Newline -----*/

// Transmit a complete line or nothing if insufficient space

bool SerialPort::println(const char* text) {

    // Validate pointer
    if (text == nullptr) {
        return false;
    }

    // Calculate message length
    size_t length = strlen(text);

    // Check available transmission buffer space
    int freeBytes = Serial.availableForWrite();

    // Reserve two additional bytes for CR + LF
    if (freeBytes < 2 ||
        length > static_cast<size_t>(freeBytes - 2)) {

        return false;
    }

    // Transmit message
    if (!send(
        reinterpret_cast<const uint8_t*>(text),
        length
    )) {

        return false;
    }

    // Transmit newline
    const uint8_t newline[] = {'\r', '\n'};

    return send(newline, sizeof(newline));

}


/*----- Integer Transmission -----*/

bool SerialPort::print(int value) {

    char buffer[12];

    // Convert integer to text
    itoa(value, buffer, 10);

    return print(buffer);

}


/*----- Integer Transmission With Newline -----*/

bool SerialPort::println(int value) {

    char buffer[12];

    // Convert integer to text
    itoa(value, buffer, 10);

    return println(buffer);

}


/*----- Unsigned 32-bit Integer Transmission -----*/

bool SerialPort::print(uint32_t value) {

    char buffer[11];

    // Convert unsigned integer to text
    ultoa(value, buffer, 10);

    return print(buffer);

}


/*----- Unsigned 32-bit Integer With Newline -----*/

bool SerialPort::println(uint32_t value) {

    char buffer[11];

    // Convert unsigned integer to text
    ultoa(value, buffer, 10);

    return println(buffer);

}


/*----- Floating Point Transmission -----*/

bool SerialPort::print(float value, uint8_t decimals) {

    char buffer[52];

    // Limit decimal places
    if (decimals > 6) {
        decimals = 6;
    }

    // Convert floating-point number to text
    dtostrf(value, 0, decimals, buffer);

    return print(buffer);

}


/*----- Floating Point With Newline -----*/

bool SerialPort::println(float value, uint8_t decimals) {

    char buffer[52];

    // Limit decimal places
    if (decimals > 6) {
        decimals = 6;
    }

    // Convert floating-point number to text
    dtostrf(value, 0, decimals, buffer);

    return println(buffer);

}


/*----- Receive Buffer -----*/

// Return number of bytes available to read

int SerialPort::available() {

    return Serial.available();

}


/*----- Read Received Data -----*/

// Read one byte without waiting
// Returns -1 if no byte is available

int SerialPort::read() {

    return Serial.read();

}

/* END OF FILE */