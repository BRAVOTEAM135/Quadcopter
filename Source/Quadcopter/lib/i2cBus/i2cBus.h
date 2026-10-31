#ifndef I2CBUS_H
#define I2CBUS_H

#include <stdint.h>
#include "driverStatus.h"

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

    /// Configure and enable TWI at 100 kHz; call only when no transfer or unread result is
    /// outstanding.
    void init();

    /// Disable TWI and its interrupts, abort all transfers and discard results; close sensor
    /// drivers first.
    void close();

    /// Inspect whether the bus is open, reserved by a transfer/result, or reporting an error.
    DriverStatus getStatus() const;

    /// Cancel the matching owner's transfer and stop writes to its receive buffer; true means
    /// cancelled. Continue service() to finish STOP or its timeout.
    bool cancel(const void* owner);

    /// Request an address-acknowledgement probe; true means accepted, not acknowledged yet.
    bool startProbe(uint8_t address, uint32_t nowUs);

    /// Check STOP completion, cancellation and the 5 ms timeout without waiting for hardware.
    void service(uint32_t nowUs);

    /// Collect a completed probe once and release the bus; false means no probe result is
    /// ready.
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
    /// Start a 1..32-byte register read; true means accepted. Keep the receive buffer valid
    /// until collection or cancellation.
    bool readRegisters(uint8_t address, uint8_t reg, uint8_t* data,
                       uint8_t length, uint32_t nowUs,
                       const void* owner = nullptr);

    /// Start a single-byte register write; true means accepted. The value is copied, so no
    /// transmit buffer is needed.
    bool writeRegister(uint8_t address, uint8_t reg, uint8_t value,
                       uint32_t nowUs, const void* owner = nullptr);

    /// Collect a finished register transfer for the same owner and release the bus; false means
    /// unfinished or wrong owner.
    bool takeResult(Result& result, const void* owner = nullptr);

};

#endif
