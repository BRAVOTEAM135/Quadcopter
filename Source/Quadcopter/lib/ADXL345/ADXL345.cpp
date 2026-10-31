#include "ADXL345.h"

#define ADXL345_STARTUP_US 100000UL
#define ADXL345_ACCEL_SCALE (9.80665f / 256.0f)

/// Return true once hardware initialization is complete, including during an active read.
bool ADXL345::ready() const {
    return state_ == State::Ready || state_ == State::Status || state_ == State::Reading;
}

/// Return true while initialization or a measurement request is in progress.
bool ADXL345::busy() const {
    return state_ != State::Stopped && state_ != State::Ready && state_ != State::Failed;
}

/// Start identity checks and configuration; return false if busy or the address is invalid.
/// Initialization completes through read().
bool ADXL345::init(uint32_t nowUs) {
    if (busy()) {
        return false;
    }
    if (address_ != 0x53 && address_ != 0x1D) {
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
void ADXL345::close() {
    bus_.cancel(this);
    state_ = State::Stopped;
    pending_ = false;
    sampleAvailable_ = false;
    error_ = DriverError::None;
    sample_ = {};
}

/// Record a failure and discard the reading; reads can retry, but failed initialization needs
/// init().
void ADXL345::fail(DriverError error) {
    bool acquiring = ready();
    error_ = error;
    pending_ = false;
    sampleAvailable_ = false;
    state_ = acquiring ? State::Ready : State::Failed;
}

/// Advance the private state machine; read() services the bus and calls this helper.
void ADXL345::service(uint32_t nowUs) {
    if (state_ == State::PowerWait) {
        if (static_cast<uint32_t>(nowUs - waitStartUs_) < ADXL345_STARTUP_US) {
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
                if (buffer_[0] != 0xE5) {
                    fail(DriverError::WrongDevice);
                    return;
                }
                state_ = State::Configure;
                break;
            case State::Configure:
                if (++configStep_ == 6) {
                    state_ = State::Ready;
                }
                break;
            case State::Status:
                state_ = (buffer_[0] & 0x80) ? State::Reading : State::Ready;
                break;
            case State::Reading:
                sample_.accelerationMps2 = {
                    readLittleEndian16(buffer_) * ADXL345_ACCEL_SCALE,
                    readLittleEndian16(buffer_ + 2) * ADXL345_ACCEL_SCALE,
                    readLittleEndian16(buffer_ + 4) * ADXL345_ACCEL_SCALE};
                sampleAvailable_ = true;
                state_ = State::Ready;
                return;
            default:
                fail(DriverError::BusError);
                return;
        }
    }

    switch (state_) {
        case State::Identity:
            pending_ = bus_.readRegisters(address_, 0x00, buffer_, 1, nowUs, this);
            break;
        case State::Configure: {
            uint8_t reg = 0, value = 0;
            switch (configStep_) {
                case 0:
                    reg = 0x2D;
                    value = 0x00;
                    break; // Standby while configuring.
                case 1:
                    reg = 0x31;
                    value = 0x08;
                    break; // Full resolution, +/-2 g.
                case 2:
                    reg = 0x2C;
                    value = 0x0B;
                    break; // 200 Hz.
                case 3:
                    reg = 0x2D;
                    value = 0x08;
                    break; // Measurement mode.
                case 4:
                    reg = 0x38;
                    value = 0x00;
                    break; // FIFO bypass.
                case 5:
                    reg = 0x2E;
                    value = 0x00;
                    break; // Poll status, no interrupt pin.
            }
            pending_ = bus_.writeRegister(address_, reg, value, nowUs, this);
            break;
        }
        case State::Status:
            pending_ = bus_.readRegisters(address_, 0x30, buffer_, 1, nowUs, this);
            break;
        case State::Reading:
            pending_ = bus_.readRegisters(address_, 0x32, buffer_, 6, nowUs, this);
            if (pending_) {
                sample_.timestampUs = nowUs;
            }
            break;
        default:
            break;
    }
}

/// Combine background progress, one-time result collection and read startup.
bool ADXL345::read(AccelerationSample& sample, uint32_t nowUs) {
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
DriverStatus ADXL345::getStatus() const {
    return {ready(), busy(), error_};
}
