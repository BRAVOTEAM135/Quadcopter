#ifndef L3G4200D_H
#define L3G4200D_H

#include "i2cBus.h"
#include "imuTypes.h"

// Angular velocity in rad/s.
class L3G4200D {
public:
    /// Store the shared bus and software state; hardware starts in init().
    explicit L3G4200D(i2cBus& bus, uint8_t address = 0x69) : bus_(bus), address_(address) {}

    /// Start hardware initialization; true means accepted, not yet ready.
    bool init(uint32_t nowUs);
    /// Advance initialization/reading without waiting. True copies one fresh result;
    /// false leaves sample unchanged. Call on every loop pass using the same clock.
    bool read(GyroscopeSample& sample, uint32_t nowUs);
    /// Return readiness, current activity and the last error without changing state.
    DriverStatus getStatus() const;
    /// Stop acquisition and discard readings; leave the chip powered and shared bus open.
    void close();

private:
    // A driver must keep its own state and transfer buffer.
    L3G4200D(const L3G4200D&) = delete;
    L3G4200D& operator=(const L3G4200D&) = delete;
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
        Settling,
        Ready,
        Reading,
        Failed
    };
    i2cBus& bus_;
    const uint8_t address_;
    State state_ = State::Stopped;
    DriverError error_ = DriverError::None;
    bool pending_ = false;
    bool sampleAvailable_ = false;
    uint8_t configStep_ = 0;
    uint32_t waitStartUs_ = 0;
    uint8_t buffer_[7] = {};
    GyroscopeSample sample_ = {};

    /// Record a failure and discard the reading; reads can retry, but failed initialization
    /// needs init().
    void fail(DriverError error);
};

#endif
