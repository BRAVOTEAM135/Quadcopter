#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include <stdint.h>
#include "i2cBus.h"

struct Vector3 {
    float x;
    float y;
    float z;
};

struct AccelerationSample {
    Vector3 accelerationMps2;
    uint32_t timestampUs;
};

struct GyroscopeSample {
    Vector3 angularVelocityRadS;
    uint32_t timestampUs;
};

// Axes follow each sensor's datasheet, before mounting alignment/calibration.
// timestampUs is the host read start time, not an interrupt capture timestamp.
struct ImuSample {
    Vector3 accelerationMps2;
    Vector3 angularVelocityRadS;
    uint32_t timestampUs;
    float temperatureC; // MPU6050 chip temperature; zero for GY80 motion samples.
};

struct MagneticSample {
    Vector3 fieldMicrotesla;
    uint32_t timestampUs;
};

struct PressureSample {
    int32_t pressurePa;
    float temperatureC;
    uint32_t timestampUs;
};

// Whole-board read result. Fresh flags describe only the current read() call.
struct Gy80Sample {
    ImuSample motion;
    MagneticSample magnetic;
    PressureSample pressure;
    bool motionFresh;
    bool magneticFresh;
    bool pressureFresh;
};

/// Convert an I2C transaction result to the corresponding sensor error.
inline DriverError sensorError(i2cBus::Result result) {
    switch (result) {
        case i2cBus::Result::Ack: return DriverError::None;
        case i2cBus::Result::Nack: return DriverError::Nack;
        case i2cBus::Result::Timeout: return DriverError::Timeout;
        default: return DriverError::BusError;
    }
}

/// Decode two bytes, most significant first, as a signed 16-bit value.
inline int16_t readBigEndian16(const uint8_t* bytes) {
    return static_cast<int16_t>((static_cast<uint16_t>(bytes[0]) << 8) | bytes[1]);
}

/// Decode two bytes, least significant first, as a signed 16-bit value.
inline int16_t readLittleEndian16(const uint8_t* bytes) {
    return static_cast<int16_t>((static_cast<uint16_t>(bytes[1]) << 8) | bytes[0]);
}

#endif
