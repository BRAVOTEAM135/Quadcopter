#ifndef HMC5883L_H
#define HMC5883L_H

#include "i2cBus.h"
#include "imuTypes.h"

// Magnetic field in microtesla, in X/Y/Z order.
class HMC5883L {
public:
    /// Store the shared bus and software state; hardware starts in init().
    explicit HMC5883L(i2cBus& bus) : bus_(bus) {}

    /// Start hardware initialization; true means accepted, not yet ready.
    bool init(uint32_t nowUs);
    /// Advance initialization/reading without waiting. True copies one fresh result;
    /// false leaves sample unchanged. Call on every loop pass using the same clock.
    bool read(MagneticSample& sample, uint32_t nowUs);
    /// Return readiness, current activity and the last error without changing state.
    DriverStatus getStatus() const;
    /// Stop acquisition and discard readings; leave the chip powered and shared bus open.
    void close();

private:
    // A driver must keep its own state and transfer buffer.
    HMC5883L(const HMC5883L&) = delete;
    HMC5883L& operator=(const HMC5883L&) = delete;
    /// Test initialization completion internally, including active reads.
    bool ready() const;
    /// Test whether initialization or a read is in progress.
    bool busy() const;
    /// Advance the hardware state machine by one step without waiting.
    void service(uint32_t nowUs);

    enum class State : uint8_t {
        Stopped,
        PowerWait,
        Identity,
        Configure,
        Ready,
        Status,
        Reading,
        Failed
    };
    i2cBus& bus_;
    State state_ = State::Stopped;
    DriverError error_ = DriverError::None;
    bool pending_ = false;
    bool sampleAvailable_ = false;
    bool fresh_ = false;
    uint8_t configStep_ = 0;
    uint32_t waitStartUs_ = 0;
    uint8_t buffer_[6] = {};
    MagneticSample sample_ = {};

    /// Record a failure and discard the reading; reads can retry, but failed initialization
    /// needs init().
    void fail(DriverError error);
};

#endif
