#ifndef GY80_H
#define GY80_H

#include "i2cBus.h"
#include "imuTypes.h"

// Original GY-80: ADXL345 + L3G4200D + HMC5883L + BMP085.
class GY80 {
public:
    explicit GY80(i2cBus& bus, uint8_t accelerometerAddress = 0x53,
                  uint8_t gyroscopeAddress = 0x69);
    GY80(const GY80&) = delete;
    GY80& operator=(const GY80&) = delete;

    // Checks identities and configures the enabled devices asynchronously.
    // Disable optional devices to use just the accelerometer/gyroscope.
    bool begin(uint32_t nowUs, bool compass = true, bool barometer = true);
    void service(uint32_t nowUs);
    bool ready() const { return state_ == State::Ready; }
    bool busy() const;
    SensorError error() const { return error_; }

    // Motion: 200 Hz, +/-2 g, +/-250 degrees/s. Only fresh data is published.
    bool requestSample(uint32_t nowUs);
    bool takeSample(ImuSample& sample);
    // Compass: 75 Hz, +/-1.3 gauss, axes returned in X/Y/Z order.
    bool requestMagneticField(uint32_t nowUs);
    bool takeMagneticField(MagneticSample& sample);
    // BMP085: OSS=0. Conversion waits release the bus and allow motion reads.
    bool requestPressure(uint32_t nowUs);
    bool takePressure(PressureSample& sample);

private:
    enum class State : uint8_t { Stopped, PowerWait, Initializing, Settling, Ready, Failed };
    enum class Job : uint8_t {
        None, AccelStatus, AccelData, GyroData, MagStatus, MagData,
        TempCommand, TempData, PressureCommand, PressureData
    };
    enum class PressureState : uint8_t {
        Idle, StartTemperature, WaitTemperature, ReadTemperature,
        StartPressure, WaitPressure, ReadPressure
    };

    i2cBus& bus_;
    const uint8_t accelAddress_;
    const uint8_t gyroAddress_;
    State state_ = State::Stopped;
    Job job_ = Job::None;
    PressureState pressureState_ = PressureState::Idle;
    SensorError error_ = SensorError::None;
    bool compass_ = true;
    bool barometer_ = true;
    bool pending_ = false;
    bool sampleAvailable_ = false;
    bool magneticAvailable_ = false;
    bool pressureAvailable_ = false;
    bool magneticFresh_ = false;
    uint8_t initStep_ = 0;
    uint8_t buffer_[22] = {};
    uint16_t calibration_[11] = {};
    uint16_t rawTemperature_ = 0;
    uint32_t waitStartUs_ = 0;
    uint32_t pressureWaitUs_ = 0;
    ImuSample sample_ = {};
    MagneticSample magnetic_ = {};
    PressureSample pressure_ = {};

    void serviceInit(uint32_t nowUs);
    void startJob(uint32_t nowUs);
    void finishJob(uint32_t nowUs);
    void fail(SensorError error);
    bool compensatePressure(uint32_t rawPressure);
};

#endif
