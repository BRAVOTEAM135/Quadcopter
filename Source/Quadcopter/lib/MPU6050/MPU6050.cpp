#include "MPU6050.h"

namespace {
constexpr float ACCEL_SCALE = 9.80665f / 16384.0f;
constexpr float GYRO_SCALE = 0.01745329252f / 131.0f;
constexpr uint32_t STARTUP_WAIT_US = 100000UL;
} // namespace

/// Store the shared bus and address and prepare software state; hardware starts only when
/// init() is called.
MPU6050::MPU6050(i2cBus& bus, uint8_t address) : bus_(bus), address_(address) {
}

/// Return true once hardware initialization is complete, including during an active read.
bool MPU6050::ready() const {
    return state_ == State::Ready || state_ == State::Reading;
}

/// Return true while initialization or a measurement request is in progress.
bool MPU6050::busy() const {
    return state_ != State::Stopped && state_ != State::Ready && state_ != State::Failed;
}

/// Start identity checks and configuration; return false if busy or the address is invalid.
/// Initialization completes through read().
bool MPU6050::init(uint32_t nowUs) {
    if (busy()) {
        return false;
    }
    if (address_ != 0x68 && address_ != 0x69) {
        error_ = DriverError::WrongDevice;
        state_ = State::Failed;
        return false;
    }
    pending_ = false;
    sampleAvailable_ = false;
    error_ = DriverError::None;
    configStep_ = 0;
    waitStartUs_ = nowUs;
    state_ = State::PowerWait;
    return true;
}

/// Stop acquisition, cancel this driver's transfer and discard readings; leave the chip powered
/// and bus open.
void MPU6050::close() {
    bus_.cancel(this);
    state_ = State::Stopped;
    pending_ = false;
    sampleAvailable_ = false;
    error_ = DriverError::None;
    sample_ = {};
}

/// Record a failure and discard the reading; reads can retry, but failed initialization needs
/// init().
void MPU6050::fail(DriverError error) {
    error_ = error;
    pending_ = false;
    sampleAvailable_ = false;
    state_ = state_ == State::Reading ? State::Ready : State::Failed;
}

/// Advance the private state machine; read() services the bus and calls this helper.
void MPU6050::service(uint32_t nowUs) {
    if (state_ == State::PowerWait || state_ == State::ResetWait || state_ == State::WakeWait) {
        if (static_cast<uint32_t>(nowUs - waitStartUs_) < STARTUP_WAIT_US) {
            return;
        }
        if (state_ == State::PowerWait) {
            state_ = State::Identity;
        } else if (state_ == State::ResetWait) {
            state_ = State::Wake;
        } else {
            state_ = State::Configure;
        }
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
                // WHO_AM_I is 0x68 for either AD0 address selection.
                if (buffer_[0] != 0x68) {
                    fail(DriverError::WrongDevice);
                    return;
                }
                state_ = State::Reset;
                break;
            case State::Reset:
                waitStartUs_ = nowUs;
                state_ = State::ResetWait;
                return;
            case State::Wake:
                waitStartUs_ = nowUs;
                state_ = State::WakeWait;
                return;
            case State::Configure:
                if (++configStep_ == 6) {
                    state_ = State::Ready;
                }
                break;
            case State::Reading:
                // INT_STATUS followed by the contiguous accel/temp/gyro block.
                if ((buffer_[0] & 0x01) != 0) {
                    sample_.accelerationMps2 = {readBigEndian16(buffer_ + 1) * ACCEL_SCALE,
                                                readBigEndian16(buffer_ + 3) * ACCEL_SCALE,
                                                readBigEndian16(buffer_ + 5) * ACCEL_SCALE};
                    sample_.temperatureC = readBigEndian16(buffer_ + 7) / 340.0f + 36.53f;
                    sample_.angularVelocityRadS = {readBigEndian16(buffer_ + 9) * GYRO_SCALE,
                                                   readBigEndian16(buffer_ + 11) * GYRO_SCALE,
                                                   readBigEndian16(buffer_ + 13) * GYRO_SCALE};
                    sampleAvailable_ = true;
                }
                state_ = State::Ready;
                return;
            default:
                fail(DriverError::BusError);
                return;
        }
    }

    switch (state_) {
        case State::Identity:
            pending_ = bus_.readRegisters(address_, 0x75, buffer_, 1, nowUs, this);
            break;
        case State::Reset:
            pending_ = bus_.writeRegister(address_, 0x6B, 0x80, nowUs, this);
            break;
        case State::Wake:
            pending_ = bus_.writeRegister(address_, 0x6B, 0x01, nowUs, this);
            break;
        case State::Configure: {
            uint8_t reg = 0;
            uint8_t value = 0;
            switch (configStep_) {
                case 0:
                    reg = 0x6C;
                    value = 0x00;
                    break; // All axes enabled.
                case 1:
                    reg = 0x1A;
                    value = 0x03;
                    break; // DLPF: ~44 Hz gyro.
                case 2:
                    reg = 0x19;
                    value = 0x04;
                    break; // 1 kHz / 5 = 200 Hz.
                case 3:
                    reg = 0x1B;
                    value = 0x00;
                    break; // +/-250 degrees/s.
                case 4:
                    reg = 0x1C;
                    value = 0x00;
                    break; // +/-2 g.
                case 5:
                    reg = 0x38;
                    value = 0x01;
                    break; // DATA_RDY status enabled.
            }
            pending_ = bus_.writeRegister(address_, reg, value, nowUs, this);
            break;
        }
        case State::Reading:
            pending_ =
                bus_.readRegisters(address_, 0x3A, buffer_, sizeof(buffer_), nowUs, this);
            if (pending_) {
                sample_.timestampUs = nowUs;
            }
            break;
        default:
            break;
    }
}

/// Combine background progress, one-time result collection and read startup.
bool MPU6050::read(ImuSample& sample, uint32_t nowUs) {
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
        state_ = State::Reading;
        service(nowUs);
    }
    return false;
}

/// Inspect state without starting a transfer or consuming a measurement.
DriverStatus MPU6050::getStatus() const {
    return {ready(), busy(), error_};
}
