#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include <stdint.h>
#include "i2cBus.h"

struct Vector3 {
    float x;
    float y;
    float z;
};

// Axes follow each sensor's datasheet, before mounting alignment/calibration.
// timestampUs is the host read start time, not an interrupt capture timestamp.
struct ImuSample {
    Vector3 accelerationMps2;
    Vector3 angularVelocityRadS;
    uint32_t timestampUs;
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

enum class SensorError : uint8_t {
    None, Nack, BusError, Timeout, WrongDevice, InvalidData
};

inline SensorError sensorError(i2cBus::Result result) {
    switch (result) {
        case i2cBus::Result::Ack: return SensorError::None;
        case i2cBus::Result::Nack: return SensorError::Nack;
        case i2cBus::Result::Timeout: return SensorError::Timeout;
        default: return SensorError::BusError;
    }
}

inline int16_t readBigEndian16(const uint8_t* bytes) {
    return static_cast<int16_t>((static_cast<uint16_t>(bytes[0]) << 8) | bytes[1]);
}

inline int16_t readLittleEndian16(const uint8_t* bytes) {
    return static_cast<int16_t>((static_cast<uint16_t>(bytes[1]) << 8) | bytes[0]);
}

#endif
