#include "L3G4200D.h"

#define L3G4200D_STARTUP_US 100000UL
#define L3G4200D_SETTLE_US 300000UL
#define L3G4200D_GYRO_SCALE (0.00875f * 0.01745329252f)

/// Return true after configuration and gyro settling, including during an active read.
bool L3G4200D::ready() const {
    return state_ == State::Ready || state_ == State::Reading;
}

/// Return true while initialization or a measurement request is in progress.
bool L3G4200D::busy() const {
    return state_ != State::Stopped && state_ != State::Ready && state_ != State::Failed;
}

/// Start identity checks and configuration; return false if busy or the address is invalid.
/// Initialization completes through read().
bool L3G4200D::init(uint32_t nowUs) {
    if (busy()) {
        return false;
    }
    if (address_ != 0x69 && address_ != 0x68) {
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
void L3G4200D::close() {
    bus_.cancel(this);
    state_ = State::Stopped;
    pending_ = false;
    sampleAvailable_ = false;
    error_ = DriverError::None;
    sample_ = {};
}

/// Record a failure and discard the reading; reads can retry, but failed initialization needs
/// init().
void L3G4200D::fail(DriverError error) {
    bool acquiring = ready();
    error_ = error;
    pending_ = false;
    sampleAvailable_ = false;
    state_ = acquiring ? State::Ready : State::Failed;
}

/// Advance the private state machine; read() services the bus and calls this helper.
void L3G4200D::service(uint32_t nowUs) {
    if (state_ == State::PowerWait || state_ == State::Settling) {
        uint32_t waitUs = state_ == State::PowerWait ? L3G4200D_STARTUP_US : L3G4200D_SETTLE_US;
        if (static_cast<uint32_t>(nowUs - waitStartUs_) < waitUs) {
            return;
        }
        state_ = state_ == State::PowerWait ? State::Identity : State::Ready;
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
                if (buffer_[0] != 0xD3) {
                    fail(DriverError::WrongDevice);
                    return;
                }
                state_ = State::Configure;
                break;
            case State::Configure:
                if (++configStep_ == 7) {
                    waitStartUs_ = nowUs;
                    state_ = State::Settling;
                    return;
                }
                break;
            case State::Reading:
                // STATUS and XYZ are one burst, with little-endian axis data.
                if (buffer_[0] & 0x08) {
                    sample_.angularVelocityRadS = {
                        readLittleEndian16(buffer_ + 1) * L3G4200D_GYRO_SCALE,
                        readLittleEndian16(buffer_ + 3) * L3G4200D_GYRO_SCALE,
                        readLittleEndian16(buffer_ + 5) * L3G4200D_GYRO_SCALE};
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
            pending_ = bus_.readRegisters(address_, 0x0F, buffer_, 1, nowUs, this);
            break;
        case State::Configure: {
            uint8_t reg = 0, value = 0;
            switch (configStep_) {
                case 0:
                    reg = 0x20;
                    value = 0x00;
                    break; // Power down while configuring.
                case 1:
                    reg = 0x23;
                    value = 0x80;
                    break; // BDU, little endian, +/-250 dps.
                case 2:
                    reg = 0x20;
                    value = 0x6F;
                    break; // 200 Hz, 50 Hz LPF, XYZ enabled.
                case 3:
                    reg = 0x21;
                    value = 0x00;
                    break;
                case 4:
                    reg = 0x22;
                    value = 0x00;
                    break; // No interrupt pin.
                case 5:
                    reg = 0x24;
                    value = 0x00;
                    break; // HPF and FIFO disabled.
                case 6:
                    reg = 0x2E;
                    value = 0x00;
                    break;
            }
            pending_ = bus_.writeRegister(address_, reg, value, nowUs, this);
            break;
        }
        case State::Reading:
            // Bit 7 enables address auto-increment from STATUS through XYZ.
            pending_ = bus_.readRegisters(address_, 0xA7, buffer_, 7, nowUs, this);
            if (pending_) {
                sample_.timestampUs = nowUs;
            }
            break;
        default:
            break;
    }
}

/// Combine background progress, one-time result collection and read startup.
bool L3G4200D::read(GyroscopeSample& sample, uint32_t nowUs) {
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
DriverStatus L3G4200D::getStatus() const {
    return {ready(), busy(), error_};
}
