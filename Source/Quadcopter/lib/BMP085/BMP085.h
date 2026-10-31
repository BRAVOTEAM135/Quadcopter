#ifndef BMP085_H
#define BMP085_H

#include "i2cBus.h"
#include "imuTypes.h"

// Compensated pressure in Pa and temperature in degrees C.
class BMP085 {
public:
    /// Store the shared bus and software state; hardware starts in init().
    explicit BMP085(i2cBus& bus) : bus_(bus) {}

    /// Start hardware initialization; true means accepted, not yet ready.
    bool init(uint32_t nowUs);
    /// Advance initialization/reading without waiting. True copies one fresh result;
    /// false leaves sample unchanged. Call on every loop pass using the same clock.
    bool read(PressureSample& sample, uint32_t nowUs);
    /// Return readiness, current activity and the last error without changing state.
    DriverStatus getStatus() const;
    /// Stop acquisition and discard readings; leave the chip powered and shared bus open.
    void close();

private:
    // A driver must keep its own state and transfer buffer.
    BMP085(const BMP085&) = delete;
    BMP085& operator=(const BMP085&) = delete;
    /// Test initialization completion internally, including active reads.
    bool ready() const { return initialized_; }
    /// Test whether initialization or a read is in progress.
    bool busy() const;
    /// Advance the hardware state machine by one step without waiting.
    void service(uint32_t nowUs);

    enum class State : uint8_t {
        Stopped,
        PowerWait,
        Identity,
        Calibration,
        Ready,
        TemperatureCommand,
        TemperatureWait,
        TemperatureRead,
        PressureCommand,
        PressureWait,
        PressureRead,
        Failed
    };

    i2cBus& bus_;
    State state_ = State::Stopped;
    DriverError error_ = DriverError::None;
    bool initialized_ = false;
    bool pending_ = false;
    bool sampleAvailable_ = false;
    uint32_t waitStartUs_ = 0;
    uint8_t buffer_[22] = {};
    uint16_t calibration_[11] = {};
    uint16_t rawTemperature_ = 0;
    PressureSample pressure_ = {};

    /// Record a failure and discard the reading; reads can retry, but failed initialization
    /// needs init().
    void fail(DriverError error);
    /// Convert raw measurements with the stored calibration; return false for invalid
    /// calibration or out-of-range results.
    bool compensatePressure(uint32_t rawPressure);
};

#endif
