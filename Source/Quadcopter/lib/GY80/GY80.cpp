#include "GY80.h"

#define MOTION_INTERVAL_US 5000UL
#define COMPASS_INTERVAL_US 40000UL
#define PRESSURE_INTERVAL_US 100000UL

/// Connect the four component drivers to the same hardware bus.
GY80::GY80(i2cBus& bus, uint8_t accelerometerAddress, uint8_t gyroscopeAddress)
    : bus_(bus), accelerometer_(bus, accelerometerAddress), gyroscope_(bus, gyroscopeAddress),
      compass_(bus), barometer_(bus) {
}

/// Start all enabled chips; read() progresses them in board order.
bool GY80::init(uint32_t nowUs, bool compass, bool barometer) {
    if (getStatus().busy) {
        return false;
    }
    close();
    compassEnabled_ = compass;
    barometerEnabled_ = barometer;
    if (!accelerometer_.init(nowUs)) {
        failInitialization(accelerometer_.getStatus().error);
        return false;
    }
    if (!gyroscope_.init(nowUs)) {
        failInitialization(gyroscope_.getStatus().error);
        return false;
    }
    if (compassEnabled_ && !compass_.init(nowUs)) {
        failInitialization(compass_.getStatus().error);
        return false;
    }
    if (barometerEnabled_ && !barometer_.init(nowUs)) {
        failInitialization(barometer_.getStatus().error);
        return false;
    }
    motionTimeUs_ = compassTimeUs_ = pressureTimeUs_ = nowUs;
    state_ = State::InitAcceleration;
    return true;
}

/// Stop chip activity and invalidate stored board data without closing the bus.
void GY80::close() {
    accelerometer_.close();
    gyroscope_.close();
    compass_.close();
    barometer_.close();
    state_ = State::Stopped;
    motion_ = Motion::Idle;
    compassEnabled_ = barometerEnabled_ = false;
    compassReading_ = pressureReading_ = false;
    latest_ = {};
    pendingAcceleration_ = {};
    error_ = DriverError::None;
}

/// Report readiness, pending activity and errors without advancing hardware.
DriverStatus GY80::getStatus() const {
    bool initializing = state_ != State::Stopped && state_ != State::Ready;
    return {state_ == State::Ready,
            initializing || motion_ != Motion::Idle || compassReading_ || pressureReading_,
            error_};
}

/// Close the owned chips while preserving the reason initialization failed.
void GY80::failInitialization(DriverError error) {
    close();
    error_ = error;
}

/// Poll initialization without letting a newly initialized chip start reading early.
void GY80::serviceInit(uint32_t nowUs) {
    DriverStatus status = {};
    switch (state_) {
        case State::InitAcceleration: {
            AccelerationSample unused;
            accelerometer_.read(unused, nowUs);
            status = accelerometer_.getStatus();
            if (status.ready) {
                state_ = State::InitGyroscope;
            }
            break;
        }
        case State::InitGyroscope: {
            GyroscopeSample unused;
            gyroscope_.read(unused, nowUs);
            status = gyroscope_.getStatus();
            if (status.ready) {
                if (compassEnabled_) {
                    state_ = State::InitCompass;
                } else if (barometerEnabled_) {
                    state_ = State::InitPressure;
                } else {
                    state_ = State::Ready;
                }
            }
            break;
        }
        case State::InitCompass: {
            MagneticSample unused;
            compass_.read(unused, nowUs);
            status = compass_.getStatus();
            if (status.ready) {
                state_ = barometerEnabled_ ? State::InitPressure : State::Ready;
            }
            break;
        }
        case State::InitPressure: {
            PressureSample unused;
            barometer_.read(unused, nowUs);
            status = barometer_.getStatus();
            if (status.ready) {
                state_ = State::Ready;
            }
            break;
        }
        default:
            return;
    }
    if (status.error != DriverError::None) {
        failInitialization(status.error);
    }
}

/// Read acceleration first, then gyro; publish only a complete fresh pair.
void GY80::readMotion(uint32_t nowUs) {
    if (motion_ == Motion::Idle) {
        if (compassReading_ ||
            static_cast<uint32_t>(nowUs - motionTimeUs_) < MOTION_INTERVAL_US) {
            return;
        }
        motionTimeUs_ = nowUs;
        motion_ = Motion::Acceleration;
    }
    if (motion_ == Motion::Acceleration) {
        AccelerationSample acceleration;
        if (accelerometer_.read(acceleration, nowUs)) {
            pendingAcceleration_ = acceleration;
            motion_ = Motion::Gyroscope;
        } else {
            DriverStatus status = accelerometer_.getStatus();
            if (!status.busy) {
                if (status.error != DriverError::None) {
                    error_ = status.error;
                }
                motion_ = Motion::Idle;
                return;
            }
        }
    }
    if (motion_ == Motion::Gyroscope) {
        GyroscopeSample gyro;
        if (gyroscope_.read(gyro, nowUs)) {
            latest_.motion.accelerationMps2 = pendingAcceleration_.accelerationMps2;
            latest_.motion.timestampUs = pendingAcceleration_.timestampUs;
            latest_.motion.angularVelocityRadS = gyro.angularVelocityRadS;
            latest_.motionFresh = true;
            error_ = DriverError::None;
            motion_ = Motion::Idle;
        } else {
            DriverStatus status = gyroscope_.getStatus();
            if (!status.busy) {
                if (status.error != DriverError::None) {
                    error_ = status.error;
                }
                motion_ = Motion::Idle;
            }
        }
    }
}

/// Service the compass at its slower cadence, between motion acquisitions.
void GY80::readCompass(uint32_t nowUs) {
    if (!compassEnabled_) {
        return;
    }
    if (!compassReading_) {
        if (motion_ != Motion::Idle ||
            static_cast<uint32_t>(nowUs - compassTimeUs_) < COMPASS_INTERVAL_US) {
            return;
        }
        compassTimeUs_ = nowUs;
        compassReading_ = true;
    }
    if (compass_.read(latest_.magnetic, nowUs)) {
        latest_.magneticFresh = true;
        compassReading_ = false;
    } else {
        DriverStatus status = compass_.getStatus();
        if (!status.busy) {
            if (status.error != DriverError::None) {
                error_ = status.error;
            }
            compassReading_ = false;
        }
    }
}

/// Service pressure independently; other chips can run during conversion waits.
void GY80::readPressure(uint32_t nowUs) {
    if (!barometerEnabled_) {
        return;
    }
    if (!pressureReading_) {
        if (static_cast<uint32_t>(nowUs - pressureTimeUs_) < PRESSURE_INTERVAL_US) {
            return;
        }
        pressureTimeUs_ = nowUs;
        pressureReading_ = true;
    }
    if (barometer_.read(latest_.pressure, nowUs)) {
        latest_.pressureFresh = true;
        pressureReading_ = false;
    } else {
        DriverStatus status = barometer_.getStatus();
        if (!status.busy) {
            if (status.error != DriverError::None) {
                error_ = status.error;
            }
            pressureReading_ = false;
        }
    }
}

/// Progress background work and return a board snapshot only when data changes.
bool GY80::read(Gy80Sample& sample, uint32_t nowUs) {
    bus_.service(nowUs);
    latest_.motionFresh = latest_.magneticFresh = latest_.pressureFresh = false;
    if (state_ != State::Ready) {
        serviceInit(nowUs);
        return false;
    }
    readCompass(nowUs);
    readMotion(nowUs);
    readPressure(nowUs);
    if (!latest_.motionFresh && !latest_.magneticFresh && !latest_.pressureFresh) {
        return false;
    }
    sample = latest_;
    return true;
}
