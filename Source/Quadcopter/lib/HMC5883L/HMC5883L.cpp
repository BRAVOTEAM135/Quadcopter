#include "HMC5883L.h"

#define HMC5883L_ADDRESS 0x1E
#define HMC5883L_STARTUP_US 100000UL
#define HMC5883L_MAG_SCALE (100.0f / 1090.0f)

/// Return true once hardware initialization is complete, including during an active read.
bool HMC5883L::ready() const {
    return state_ == State::Ready || state_ == State::Status || state_ == State::Reading;
}

/// Return true while initialization or a measurement request is in progress.
bool HMC5883L::busy() const {
    return state_ != State::Stopped && state_ != State::Ready && state_ != State::Failed;
}

/// Start compass identity checks and configuration; return false if busy. Keep calling
/// read() until getStatus().ready.
bool HMC5883L::init(uint32_t nowUs) {
    if (busy()) {
        return false;
    }
    pending_ = false;
    sampleAvailable_ = false;
    fresh_ = false;
    error_ = DriverError::None;
    configStep_ = 0;
    waitStartUs_ = nowUs;
    state_ = State::PowerWait;
    return true;
}

/// Stop acquisition, cancel this driver's transfer and discard readings; leave the chip powered
/// and bus open.
void HMC5883L::close() {
    bus_.cancel(this);
    state_ = State::Stopped;
    pending_ = false;
    sampleAvailable_ = false;
    fresh_ = false;
    error_ = DriverError::None;
    sample_ = {};
}

/// Record a failure and discard the reading; reads can retry, but failed initialization needs
/// init().
void HMC5883L::fail(DriverError error) {
    bool acquiring = ready();
    error_ = error;
    pending_ = false;
    sampleAvailable_ = false;
    state_ = acquiring ? State::Ready : State::Failed;
}

/// Advance the private state machine; read() services the bus and calls this helper.
void HMC5883L::service(uint32_t nowUs) {
    if (state_ == State::PowerWait) {
        if (static_cast<uint32_t>(nowUs - waitStartUs_) < HMC5883L_STARTUP_US) {
            return;
        }
        state_ = State::Identity;
    }

    if (pending_) {
        i2cBus::Result result;
        if (!bus_.takeResult(result, this)) {
            return;
        }
        pending_ = false;
        if (result != i2cBus::Result::Ack) {
            fail(sensorError(result));
            return;
        }
        switch (state_) {
            case State::Identity:
                if (buffer_[0] != 'H' || buffer_[1] != '4' || buffer_[2] != '3') {
                    fail(DriverError::WrongDevice);
                    return;
                }
                state_ = State::Configure;
                break;
            case State::Configure:
                if (++configStep_ == 3) {
                    state_ = State::Ready;
                }
                break;
            case State::Status:
                // Drain locked partial data, but publish only a fresh reading.
                fresh_ = (buffer_[0] & 0x03) == 0x01;
                state_ = (buffer_[0] & 0x03) ? State::Reading : State::Ready;
                break;
            case State::Reading: {
                if (fresh_) {
                    int16_t x = readBigEndian16(buffer_);
                    int16_t z = readBigEndian16(buffer_ + 2);
                    int16_t y = readBigEndian16(buffer_ + 4);
                    if (x == -4096 || y == -4096 || z == -4096) {
                        fail(DriverError::InvalidData);
                        return;
                    }
                    sample_.fieldMicrotesla = {x * HMC5883L_MAG_SCALE, y * HMC5883L_MAG_SCALE,
                                               z * HMC5883L_MAG_SCALE};
                    sampleAvailable_ = true;
                }
                state_ = State::Ready;
                return;
            }
            default:
                fail(DriverError::BusError);
                return;
        }
    }

    switch (state_) {
        case State::Identity:
            pending_ = bus_.readRegisters(HMC5883L_ADDRESS, 0x0A, buffer_, 3, nowUs, this);
            break;
        case State::Configure: {
            uint8_t reg = configStep_, value = 0;
            switch (configStep_) {
                case 0:
                    value = 0x78;
                    break; // 8 averages, 75 Hz.
                case 1:
                    value = 0x20;
                    break; // +/-1.3 gauss.
                case 2:
                    value = 0x00;
                    break; // Continuous conversion.
            }
            pending_ = bus_.writeRegister(HMC5883L_ADDRESS, reg, value, nowUs, this);
            break;
        }
        case State::Status:
            pending_ = bus_.readRegisters(HMC5883L_ADDRESS, 0x09, buffer_, 1, nowUs, this);
            break;
        case State::Reading:
            pending_ = bus_.readRegisters(HMC5883L_ADDRESS, 0x03, buffer_, 6, nowUs, this);
            if (pending_) {
                sample_.timestampUs = nowUs;
            }
            break;
        default:
            break;
    }
}

/// Combine background progress, one-time result collection and read startup.
bool HMC5883L::read(MagneticSample& sample, uint32_t nowUs) {
    bool wasReady = ready();
    bool wasBusy = busy();
    bus_.service(nowUs);
    service(nowUs);
    if (sampleAvailable_) {
        sample = sample_;
        sampleAvailable_ = false;
        return true;
    }
    // Finish initialization or a failed/stale read on its own pass. This keeps
    // status/error observable and lets a parent initialize chips in order.
    if (wasReady && !wasBusy && state_ == State::Ready) {
        error_ = DriverError::None;
        state_ = State::Status;
        service(nowUs);
    }
    return false;
}

/// Inspect state without starting a transfer or consuming a measurement.
DriverStatus HMC5883L::getStatus() const {
    return {ready(), busy(), error_};
}
