#ifndef MPU6050_H
#define MPU6050_H

#include "i2cBus.h"
#include "imuTypes.h"

class MPU6050 {
public:
    explicit MPU6050(i2cBus& bus, uint8_t address = 0x68);
    MPU6050(const MPU6050&) = delete;
    MPU6050& operator=(const MPU6050&) = delete;

    // Asynchronous identity check, reset and configuration. Returns false if
    // already busy or the address is invalid. Service until ready() or error().
    bool begin(uint32_t nowUs);
    void service(uint32_t nowUs);
    bool ready() const;
    bool busy() const;
    SensorError error() const { return error_; }

    // +/-2 g, +/-250 degrees/s, 200 Hz, DLPF enabled. A request with no fresh
    // hardware data finishes without publishing a sample. Retry next period.
    bool requestSample(uint32_t nowUs);
    bool takeSample(ImuSample& sample);
    float temperatureC() const { return temperatureC_; }

private:
    enum class State : uint8_t {
        Stopped, PowerWait, Identity, Reset, ResetWait, Wake, WakeWait,
        Configure, Ready, Reading, Failed
    };
    i2cBus& bus_;
    const uint8_t address_;
    State state_ = State::Stopped;
    SensorError error_ = SensorError::None;
    bool pending_ = false;
    bool sampleAvailable_ = false;
    uint8_t configStep_ = 0;
    uint32_t waitStartUs_ = 0;
    uint8_t buffer_[15] = {};
    ImuSample sample_ = {};
    float temperatureC_ = 0;

    void fail(SensorError error);
};

#endif
