#include "imu.h"
#include "GY80.h"
#include "MPU6050.h"

IMU::IMU(GY80& sensor) : gy80_(&sensor), mpu6050_(nullptr) {}

IMU::IMU(MPU6050& sensor) : gy80_(nullptr), mpu6050_(&sensor) {}

bool IMU::begin(uint32_t nowUs) {
    magneticRequestUs_ = nowUs;
    if (gy80_ != nullptr) {
        // Enable the compass for heading; pressure remains a separate task.
        return gy80_->begin(nowUs, true, false);
    }
    return mpu6050_->begin(nowUs);
}

void IMU::service(uint32_t nowUs) {
    if (gy80_ != nullptr) {
        gy80_->service(nowUs);
        // A rejected request simply retries on a later pass; no waiting.
        if (gy80_->ready() &&
            static_cast<uint32_t>(nowUs - magneticRequestUs_) >= 40000UL &&
            gy80_->requestMagneticField(nowUs)) {
            magneticRequestUs_ = nowUs;
        }
    } else {
        mpu6050_->service(nowUs);
    }
}

bool IMU::ready() const {
    if (gy80_ != nullptr) return gy80_->ready();
    return mpu6050_->ready();
}

SensorError IMU::error() const {
    if (gy80_ != nullptr) return gy80_->error();
    return mpu6050_->error();
}

bool IMU::requestSample(uint32_t nowUs) {
    if (gy80_ != nullptr) return gy80_->requestSample(nowUs);
    return mpu6050_->requestSample(nowUs);
}

bool IMU::takeSample(ImuSample& sample) {
    if (gy80_ != nullptr) return gy80_->takeSample(sample);
    return mpu6050_->takeSample(sample);
}

bool IMU::takeMagneticField(MagneticSample& sample) {
    return gy80_ != nullptr && gy80_->takeMagneticField(sample);
}
