#include "SerialPort.h"

#include <string.h>
#include <stdlib.h>


void SerialPort::init(uint32_t baudRate) {

    Serial.begin(baudRate);

}


// Raw data transmission
bool SerialPort::send(const uint8_t* data, size_t length) {

    // Nothing to transmit
    if (length == 0) {
        return true;
    }

    // Validate pointer
    if (data == nullptr) {
        return false;
    }

    // Check sufficient buffer space exists
    if (length > static_cast<size_t>(Serial.availableForWrite())) {
        return false;
    }

    // Write data and verify number of bytes accepted
    return Serial.write(data, length) == length;

}


// Transmit text
bool SerialPort::print(const char* text) {

    if (text == nullptr) {
        return false;
    }

    size_t length = strlen(text);

    return send(
        reinterpret_cast<const uint8_t*>(text),
        length
    );

}


// Transmit text followed by newline
bool SerialPort::println(const char* text) {

    if (text == nullptr) {
        return false;
    }

    size_t length = strlen(text);

    // Ensure the entire message and newline fit
    if (length + 2 > static_cast<size_t>(Serial.availableForWrite())) {
        return false;
    }

    if (!send(
        reinterpret_cast<const uint8_t*>(text),
        length
    )) {
        return false;
    }

    const uint8_t newline[] = {'\r', '\n'};

    return send(newline, sizeof(newline));

}


// Transmit integer
bool SerialPort::print(int value) {

    char buffer[12];

    itoa(value, buffer, 10);

    return print(buffer);

}


// Transmit integer followed by newline
bool SerialPort::println(int value) {

    char buffer[12];

    itoa(value, buffer, 10);

    return println(buffer);

}


// Transmit an unsigned 32-bit integer followed by a newline
bool SerialPort::println(uint32_t value) {

    char buffer[11];

    ultoa(value, buffer, 10);

    return println(buffer);

}


// Transmit floating-point number
bool SerialPort::print(float value, uint8_t decimals) {

    char buffer[52];

    // Limit decimal places to protect buffer
    if (decimals > 6) {
        decimals = 6;
    }

    dtostrf(value, 0, decimals, buffer);

    return print(buffer);

}


// Transmit floating-point number followed by newline
bool SerialPort::println(float value, uint8_t decimals) {

    char buffer[52];

    if (decimals > 6) {
        decimals = 6;
    }

    dtostrf(value, 0, decimals, buffer);

    return println(buffer);

}


// Number of received bytes available
int SerialPort::available() {

    return Serial.available();

}


// Read one received byte
int SerialPort::read() {

    return Serial.read();

}