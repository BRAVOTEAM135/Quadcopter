#ifndef SERIALPORT_H_
#define SERIALPORT_H_

#include <Arduino.h>
#include "driverStatus.h"

class SerialPort {

public:

    /// Initialize the Arduino UART at the requested baud rate and enable this driver's
    /// read/write calls.
    void init(uint32_t baudRate);

    /// Wait for queued serial output to finish, then disable UART; repeated calls are safe.
    void close();

    /// Inspect whether UART is open and has queued transmit bytes; never waits.
    DriverStatus getStatus() const;

    /// Queue text without a newline without waiting; false means closed, invalid text or
    /// insufficient TX space.
    bool print(const char* text);
    /// Queue text followed by CR/LF without waiting; false means closed, invalid text or
    /// insufficient TX space.
    bool println(const char* text);

    /// Queue an integer without a newline without waiting; false means closed or insufficient
    /// TX space.
    bool print(int value);
    /// Queue an integer followed by CR/LF without waiting; false means closed or insufficient
    /// TX space.
    bool println(int value);
    /// Queue an unsigned 32-bit integer without a newline without waiting; false means closed
    /// or insufficient TX space.
    bool print(uint32_t value);
    /// Queue an unsigned 32-bit integer followed by CR/LF without waiting; false means closed
    /// or insufficient TX space.
    bool println(uint32_t value);

    /// Queue a floating-point value without a newline without waiting; false means closed or
    /// insufficient TX space.
    bool print(float value, uint8_t decimals = 2);
    /// Queue a floating-point value followed by CR/LF without waiting; false means closed or
    /// insufficient TX space.
    bool println(float value, uint8_t decimals = 2);

    /// Return the number of received bytes waiting to be read; return zero if closed.
    int available();
    /// Read one received byte without waiting; return -1 if no byte is available or the driver
    /// is closed.
    int read();

private:

    bool initialized_ = false;

    /// Queue the entire byte sequence without waiting; false means closed, invalid data or
    /// insufficient TX space.
    bool send(const uint8_t* data, size_t length);

};

#endif

/* END OF FILE */
