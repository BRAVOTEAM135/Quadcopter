#ifndef SERIALPORT_H_
#define SERIALPORT_H_

#include <Arduino.h>

class SerialPort {

public:

    // Initialise the hardware UART
    void init(uint32_t baudRate);

    // Text transmission
    // Returns false if the TX buffer is too full
    bool print(const char* text);
    bool println(const char* text);

    // Integer transmission
    bool print(int value);
    bool println(int value);
    bool print(uint32_t value);
    bool println(uint32_t value);

    // Floating-point transmission
    bool print(float value, uint8_t decimals = 2);
    bool println(float value, uint8_t decimals = 2);

    // Receive data without waiting
    // read() returns -1 when no byte is available
    int available();
    int read();

private:

    // Queue all bytes or return false without waiting
    bool send(const uint8_t* data, size_t length);

};

#endif

/* END OF FILE */