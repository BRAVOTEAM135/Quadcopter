#ifndef IMU_H
#define IMU_H

#include "imuTypes.h"

// The application supplies one of these drivers. Their full definitions are
// needed in imu.cpp; this header only needs their names for references/pointers.
class GY80;
class MPU6050;

// Common accelerometer/gyroscope interface. The sensor driver still owns its
// transfer state; this wrapper adds no waits or additional acquisition stages.
// Measurements keep the driver's units and axes. imuFilter handles stationary
// gyro bias and orientation; this wrapper does not correct the raw samples.
class IMU {
public:
    // The constructor matching the supplied driver's type is chosen by C++.
    // Keep that driver alive for as long as this IMU object is used.
    explicit IMU(GY80& sensor);
    explicit IMU(MPU6050& sensor);
    IMU(const IMU&) = delete;
    IMU& operator=(const IMU&) = delete;

    // Start initialization once during setup, using the Timer1 timestamp.
    // true means accepted; ready() becomes true after initialization finishes.
    bool begin(uint32_t nowUs);

    // Advance the sensor driver every scheduler pass. The shared I2C bus must
    // also be serviced by the application/scheduler.
    void service(uint32_t nowUs);
    bool ready() const;
    SensorError error() const;

    // Request from the periodic task. false means not ready, busy, or a previous
    // sample is unread. This never waits for a measurement.
    bool requestSample(uint32_t nowUs);

    // Copy a new sample once: acceleration in m/s^2, angular velocity in rad/s,
    // timestamp in microseconds. false means no completed sample is available.
    bool takeSample(ImuSample& sample);

    // GY80 supplies a compass; MPU6050 alone does not. Compass acquisition is
    // serviced automatically at 25 Hz, separately from 200 Hz motion requests.
    bool hasMagnetometer() const { return gy80_ != nullptr; }
    bool takeMagneticField(MagneticSample& sample);

private:
    // Exactly one pointer is set by the constructor; the other is nullptr.
    // These are references to existing objects, not two allocated drivers.
    // const prevents changing the selected driver during an acquisition.
    GY80* const gy80_;
    MPU6050* const mpu6050_;
    uint32_t magneticRequestUs_ = 0;
};

#endif
