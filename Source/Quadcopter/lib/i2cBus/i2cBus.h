#ifndef I2CBUS_H
#define I2CBUS_H

#include <stdint.h>

class i2cBus {

public:

    // The outcome of a finished transaction. Ack means the entire requested
    // transfer succeeded. Nack means the sensor rejected an address or byte.
    enum class ProbeResult : uint8_t {
        Ack,
        Nack,
        BusError,
        Timeout
    };

    // Register transfers use the same outcomes as probes. This is another name
    // for the same enum, keeping existing ProbeResult callers compatible.
    using Result = ProbeResult;

    // How to use this driver:
    //   1. Call init() once during setup().
    //   2. Request a probe, register read, or register write.
    //   3. Keep calling service(nowUs) from loop() while other work runs.
    //   4. Collect the result with the matching take...() function.
    //
    // Request functions return immediately. true means "request accepted",
    // not "sensor responded". false means busy or invalid arguments.
    // All addresses are 7-bit addresses, e.g. 0x53, without a read/write bit.
    // Use the same microsecond clock for requests and service(), e.g. micros().
    // Call these functions from normal program code, not another ISR.

    // Configure the ATmega328P's hardware I2C peripheral, called TWI, for 100 kHz.
    // Initialize only when no transaction is outstanding.
    void init();

    // Ask "does anything acknowledge this address?" No register data is sent.
    bool startProbe(uint8_t address, uint32_t nowUs);

    // Check STOP completion and the 5 ms timeout. Byte transfers themselves
    // progress through hardware interrupts; service() never waits for a byte.
    void service(uint32_t nowUs);

    // false: no finished probe yet. true: result was copied out and the bus
    // is available for another request. Each result can be collected once.
    bool takeProbeResult(ProbeResult& result);

    // Only ONE transaction may be outstanding on this hardware bus. A finished
    // transaction still reserves the driver until its result is collected.
    //
    // Read 1..32 bytes beginning at a sensor's register address. data points to
    // the caller's receive array: keep it alive and do not use/change its bytes
    // until takeResult() returns true. The interrupt fills it over time. After
    // an error it may contain only part of the requested data; discard that data.
    //
    // owner is an optional identity label, not a data buffer. Sensor drivers
    // pass this so each driver can collect only its own result. The pointer is
    // compared, never dereferenced. Simple callers can omit it (nullptr).
    bool readRegisters(uint8_t address, uint8_t reg, uint8_t* data,
                       uint8_t length, uint32_t nowUs,
                       const void* owner = nullptr);

    // Write one byte to one sensor register. value is copied when accepted,
    // so the caller does not need to keep a transmit buffer alive.
    bool writeRegister(uint8_t address, uint8_t reg, uint8_t value,
                       uint32_t nowUs, const void* owner = nullptr);

    // Collect a register read/write result using the SAME owner as the request.
    // false means not finished or owner mismatch. true releases the driver for
    // the next request. Probe results use takeProbeResult() instead.
    bool takeResult(Result& result, const void* owner = nullptr);

};

#endif
