#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <Arduino.h>

class SerialPort {

public:

    void init(uint32_t baudRate);

    // Text transmission
    bool print(const char* text);
    bool println(const char* text);

    // Integer transmission
    bool print(int value);
    bool println(int value);
    bool println(uint32_t value);

    // Floating-point transmission
    bool print(float value, uint8_t decimals = 2);
    bool println(float value, uint8_t decimals = 2);

    // Receive data
    int available();
    int read();

private: 

    // Raw data transmission
    bool send(const uint8_t* data, size_t length);

};

#endif