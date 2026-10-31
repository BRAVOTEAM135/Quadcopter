#ifndef GY80_H
#define GY80_H

#include "ADXL345.h"
#include "L3G4200D.h"
#include "HMC5883L.h"
#include "BMP085.h"

// The whole board: 200 Hz motion, 25 Hz compass and 10 Hz pressure.
class GY80 {
public:
    /// Store the bus and chip addresses; hardware starts only in init().
    explicit GY80(i2cBus& bus, uint8_t accelerometerAddress = 0x53,
                  uint8_t gyroscopeAddress = 0x69);
    /// Start enabled chips without waiting; true means initialization was accepted.
    bool init(uint32_t nowUs, bool compass = true, bool barometer = true);
    /// Advance all board activity without waiting. True returns updated measurements;
    /// fresh flags identify which changed. False leaves sample unchanged.
    bool read(Gy80Sample& sample, uint32_t nowUs);
    /// Inspect initialization, activity and the latest error without changing state.
    DriverStatus getStatus() const;
    /// Close every chip and clear board readings; leave the shared bus open.
    void close();

private:
    GY80(const GY80&) = delete;
    GY80& operator=(const GY80&) = delete;
    enum class State : uint8_t {
        Stopped,
        InitAcceleration,
        InitGyroscope,
        InitCompass,
        InitPressure,
        Ready
    };
    enum class Motion : uint8_t { Idle, Acceleration, Gyroscope };
    i2cBus& bus_;
    ADXL345 accelerometer_;
    L3G4200D gyroscope_;
    HMC5883L compass_;
    BMP085 barometer_;
    State state_ = State::Stopped;
    Motion motion_ = Motion::Idle;
    DriverError error_ = DriverError::None;
    bool compassEnabled_ = false;
    bool barometerEnabled_ = false;
    bool compassReading_ = false;
    bool pressureReading_ = false;
    uint32_t motionTimeUs_ = 0;
    uint32_t compassTimeUs_ = 0;
    uint32_t pressureTimeUs_ = 0;
    Gy80Sample latest_ = {};
    AccelerationSample pendingAcceleration_ = {};

    /// Progress one chip's initialization; move on only after it is ready.
    void serviceInit(uint32_t nowUs);
    /// Stop all chips and retain an initialization failure for getStatus().
    void failInitialization(DriverError error);
    /// Acquire a fresh acceleration/gyro pair when due, one small step per pass.
    void readMotion(uint32_t nowUs);
    /// Advance or start a compass read between motion acquisitions.
    void readCompass(uint32_t nowUs);
    /// Advance or start a pressure cycle while leaving conversion waits nonblocking.
    void readPressure(uint32_t nowUs);
};

#endif
