#include "BMP085.h"

namespace {
constexpr uint8_t ADDRESS = 0x77;
constexpr uint32_t STARTUP_WAIT_US = 100000UL;
constexpr uint32_t CONVERSION_US = 4500UL;
} // namespace

/// Return true during initialization or a measurement cycle, including conversion waits.
bool BMP085::busy() const {
    return state_ != State::Stopped && state_ != State::Ready && state_ != State::Failed;
}

/// Start the identity check and calibration-register read; return false if busy. Keep calling
/// read() until getStatus().ready.
bool BMP085::init(uint32_t nowUs) {
    if (busy()) {
        return false;
    }
    initialized_ = false;
    pending_ = false;
    sampleAvailable_ = false;
    error_ = DriverError::None;
    waitStartUs_ = nowUs;
    state_ = State::PowerWait;
    return true;
}

/// Stop acquisition, cancel this driver's transfer and discard readings; leave the chip powered
/// and bus open.
void BMP085::close() {
    bus_.cancel(this);
    state_ = State::Stopped;
    initialized_ = false;
    pending_ = false;
    sampleAvailable_ = false;
    error_ = DriverError::None;
    pressure_ = {};
}

/// Record a failure and discard the reading; reads can retry, but failed initialization needs
/// init().
void BMP085::fail(DriverError error) {
    error_ = error;
    pending_ = false;
    sampleAvailable_ = false;
    state_ = initialized_ ? State::Ready : State::Failed;
}

/// Advance the private state machine; read() services the bus and calls this helper.
void BMP085::service(uint32_t nowUs) {
    // Wait for power-up or a conversion while allowing other drivers to run.
    if (state_ == State::PowerWait) {
        if (static_cast<uint32_t>(nowUs - waitStartUs_) < STARTUP_WAIT_US) {
            return;
        }
        state_ = State::Identity;
    } else if (state_ == State::TemperatureWait || state_ == State::PressureWait) {
        if (static_cast<uint32_t>(nowUs - waitStartUs_) < CONVERSION_US) {
            return;
        }
        state_ =
            state_ == State::TemperatureWait ? State::TemperatureRead : State::PressureRead;
    }

    // Collect the previous transaction before starting the next step.
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
                if (buffer_[0] != 0x55) {
                    fail(DriverError::WrongDevice);
                    return;
                }
                state_ = State::Calibration;
                break;
            case State::Calibration:
                for (uint8_t i = 0; i < 11; ++i) {
                    calibration_[i] = static_cast<uint16_t>(readBigEndian16(buffer_ + 2 * i));
                    if (calibration_[i] == 0 || calibration_[i] == 0xFFFF) {
                        fail(DriverError::InvalidData);
                        return;
                    }
                }
                initialized_ = true;
                state_ = State::Ready;
                break;
            case State::TemperatureCommand:
                waitStartUs_ = nowUs;
                state_ = State::TemperatureWait;
                return;
            case State::TemperatureRead:
                rawTemperature_ = static_cast<uint16_t>(readBigEndian16(buffer_));
                state_ = State::PressureCommand;
                break;
            case State::PressureCommand:
                waitStartUs_ = nowUs;
                state_ = State::PressureWait;
                return;
            case State::PressureRead: {
                uint32_t raw = (static_cast<uint32_t>(buffer_[0]) << 16) |
                               (static_cast<uint32_t>(buffer_[1]) << 8) | buffer_[2];
                if (!compensatePressure(raw >> 8)) {
                    fail(DriverError::InvalidData);
                    return;
                }
                sampleAvailable_ = true;
                state_ = State::Ready;
                break;
            }
            default:
                fail(DriverError::BusError);
                return;
        }
    }

    switch (state_) {
        case State::Identity:
            pending_ = bus_.readRegisters(ADDRESS, 0xD0, buffer_, 1, nowUs, this);
            break;
        case State::Calibration:
            pending_ = bus_.readRegisters(ADDRESS, 0xAA, buffer_, 22, nowUs, this);
            break;
        case State::TemperatureCommand:
            pending_ = bus_.writeRegister(ADDRESS, 0xF4, 0x2E, nowUs, this);
            break;
        case State::TemperatureRead:
            pending_ = bus_.readRegisters(ADDRESS, 0xF6, buffer_, 2, nowUs, this);
            break;
        case State::PressureCommand:
            pending_ = bus_.writeRegister(ADDRESS, 0xF4, 0x34, nowUs, this);
            break;
        case State::PressureRead:
            pending_ = bus_.readRegisters(ADDRESS, 0xF6, buffer_, 3, nowUs, this);
            break;
        default:
            break;
    }
}

/// Convert raw measurements with the stored calibration; return false for invalid calibration
/// or out-of-range results.
bool BMP085::compensatePressure(uint32_t rawPressure) {
    // Bosch's integer algorithm, OSS=0. Wide intermediates avoid overflow on
    // AVR and division guards reject corrupt/degenerate calibration data.
    const int32_t ac1 = static_cast<int16_t>(calibration_[0]);
    const int32_t ac2 = static_cast<int16_t>(calibration_[1]);
    const int32_t ac3 = static_cast<int16_t>(calibration_[2]);
    const uint32_t ac4 = calibration_[3];
    const uint32_t ac5 = calibration_[4];
    const uint32_t ac6 = calibration_[5];
    const int32_t b1 = static_cast<int16_t>(calibration_[6]);
    const int32_t b2 = static_cast<int16_t>(calibration_[7]);
    const int32_t mc = static_cast<int16_t>(calibration_[9]);
    const int32_t md = static_cast<int16_t>(calibration_[10]);

    int64_t x1 = ((static_cast<int64_t>(rawTemperature_) - ac6) * ac5) >> 15;
    if (x1 + md == 0) {
        return false;
    }
    int64_t x2 = (static_cast<int64_t>(mc) * 2048) / (x1 + md);
    const int64_t b5 = x1 + x2;
    const int64_t temperature = (b5 + 8) >> 4;
    if (temperature < -400 || temperature > 850) {
        return false;
    }
    const int64_t b6 = b5 - 4000;
    x1 = (b2 * ((b6 * b6) >> 12)) >> 11;
    x2 = (ac2 * b6) >> 11;
    const int64_t b3 = (ac1 * 4 + x1 + x2 + 2) >> 2;
    x1 = (ac3 * b6) >> 13;
    x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
    const int64_t x3 = (x1 + x2 + 2) >> 2;
    const int64_t b4 = (ac4 * (x3 + 32768)) >> 15;
    if (b4 <= 0 || static_cast<int64_t>(rawPressure) < b3) {
        return false;
    }
    const int64_t b7 = (static_cast<int64_t>(rawPressure) - b3) * 50000;
    int64_t pressure = (b7 * 2) / b4;
    if (pressure <= 0 || pressure > 200000) {
        return false;
    }
    x1 = (pressure >> 8) * (pressure >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * pressure) >> 16;
    pressure += (x1 + x2 + 3791) >> 4;
    if (pressure < 30000 || pressure > 110000 || temperature < -400 || temperature > 850) {
        return false;
    }
    pressure_.pressurePa = static_cast<int32_t>(pressure);
    pressure_.temperatureC = static_cast<float>(temperature) / 10.0f;
    return true;
}

/// Combine background progress, one-time result collection and read startup.
bool BMP085::read(PressureSample& sample, uint32_t nowUs) {
    bool wasReady = ready();
    bool wasBusy = busy();
    bus_.service(nowUs);
    service(nowUs);
    if (sampleAvailable_) {
        sample = pressure_;
        sampleAvailable_ = false;
        return true;
    }
    // Finish initialization or a failed/stale read on its own pass. This keeps
    // status/error observable and lets a parent initialize chips in order.
    if (wasReady && !wasBusy && state_ == State::Ready) {
        error_ = DriverError::None;
        pressure_.timestampUs = nowUs;
        state_ = State::TemperatureCommand;
        service(nowUs);
    }
    return false;
}

/// Inspect state without starting a transfer or consuming a measurement.
DriverStatus BMP085::getStatus() const {
    return {ready(), busy(), error_};
}
