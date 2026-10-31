#ifndef IMU_H
#define IMU_H

#include "i2cBus.h"
#include "imuFilter.h"
#ifdef IMU_USE_MPU6050
#include "MPU6050.h"
#else
#include "GY80.h"
#endif

struct ImuReading {
    ImuSample motion;
    ImuOrientation orientation;
};

// Call init() once, then read() on every loop pass. No hardware wait occurs.
class IMU {
public:
    /// Store the shared bus; hardware starts only in init().
    explicit IMU(i2cBus& bus) : sensor_(bus) {}
    /// Start the selected sensor and stationary startup zero; initialize the bus first.
    bool init(uint32_t nowUs);
    /// Advance acquisition and filtering; true returns a new motion/angle snapshot.
    /// Angles remain invalid during startup zeroing. False leaves reading unchanged.
    bool read(ImuReading& reading, uint32_t nowUs);
    /// Ready means hardware and startup averaging are complete; check angle validity/age too.
    DriverStatus getStatus() const;
    /// Stop acquisition and filtering; leave the shared bus open and chips powered.
    void close();

    /// Restart stationary zeroing if open, retaining supplied compass calibration.
    void reset() { filter_.reset(); }
    /// Validate/store external compass correction and restart zeroing if open.
    bool setMagneticCalibration(const MagneticCalibration& calibration) {
        return filter_.setMagneticCalibration(calibration);
    }

private:
    IMU(const IMU&) = delete;
    IMU& operator=(const IMU&) = delete;
#ifdef IMU_USE_MPU6050
    MPU6050 sensor_;
    bool motionReading_ = false;
    uint32_t motionTimeUs_ = 0;
#else
    GY80 sensor_;
#endif
    ImuFilter filter_;
};

#endif
