#include "GY80.h"

namespace {
    constexpr uint8_t COMPASS_ADDRESS = 0x1E;
    constexpr uint8_t BAROMETER_ADDRESS = 0x77;
    constexpr uint32_t STARTUP_WAIT_US = 100000UL;
    constexpr uint32_t GYRO_SETTLE_US = 300000UL;
    constexpr uint32_t CONVERSION_US = 4500UL;
    constexpr float ACCEL_SCALE = 9.80665f / 256.0f;
    constexpr float GYRO_SCALE = 0.00875f * 0.01745329252f;
    constexpr float MAG_SCALE = 100.0f / 1090.0f;
}

GY80::GY80(i2cBus& bus, uint8_t accelerometerAddress, uint8_t gyroscopeAddress)
    : bus_(bus), accelAddress_(accelerometerAddress), gyroAddress_(gyroscopeAddress) {}

bool GY80::busy() const {
    return (state_ != State::Stopped && state_ != State::Ready && state_ != State::Failed) ||
           job_ != Job::None || pressureState_ != PressureState::Idle;
}

bool GY80::begin(uint32_t nowUs, bool compass, bool barometer) {
    if (busy()) return false;
    if ((accelAddress_ != 0x53 && accelAddress_ != 0x1D) ||
        (gyroAddress_ != 0x69 && gyroAddress_ != 0x68)) {
        error_ = SensorError::WrongDevice;
        state_ = State::Failed;
        return false;
    }
    compass_ = compass;
    barometer_ = barometer;
    pending_ = sampleAvailable_ = magneticAvailable_ = pressureAvailable_ = false;
    error_ = SensorError::None;
    initStep_ = 0;
    job_ = Job::None;
    pressureState_ = PressureState::Idle;
    waitStartUs_ = nowUs;
    state_ = State::PowerWait;
    return true;
}

void GY80::fail(SensorError error) {
    error_ = error;
    pending_ = false;
    if (state_ == State::Initializing) state_ = State::Failed;
    if (job_ == Job::TempCommand || job_ == Job::TempData ||
        job_ == Job::PressureCommand || job_ == Job::PressureData) {
        pressureState_ = PressureState::Idle;
    }
    job_ = Job::None;
}

bool GY80::requestSample(uint32_t nowUs) {
    if (!ready() || job_ != Job::None || sampleAvailable_) return false;
    error_ = SensorError::None;
    job_ = Job::AccelStatus;
    startJob(nowUs);
    return true;
}

bool GY80::takeSample(ImuSample& sample) {
    if (!sampleAvailable_) return false;
    sample = sample_;
    sampleAvailable_ = false;
    return true;
}

bool GY80::requestMagneticField(uint32_t nowUs) {
    if (!ready() || !compass_ || job_ != Job::None || magneticAvailable_) return false;
    error_ = SensorError::None;
    job_ = Job::MagStatus;
    startJob(nowUs);
    return true;
}

bool GY80::takeMagneticField(MagneticSample& sample) {
    if (!magneticAvailable_) return false;
    sample = magnetic_;
    magneticAvailable_ = false;
    return true;
}

bool GY80::requestPressure(uint32_t nowUs) {
    if (!ready() || !barometer_ || pressureState_ != PressureState::Idle ||
        pressureAvailable_) return false;
    error_ = SensorError::None;
    pressure_.timestampUs = nowUs;
    pressureState_ = PressureState::StartTemperature;
    return true;
}

bool GY80::takePressure(PressureSample& sample) {
    if (!pressureAvailable_) return false;
    sample = pressure_;
    pressureAvailable_ = false;
    return true;
}

void GY80::serviceInit(uint32_t nowUs) {
    if (pending_) {
        i2cBus::Result result;
        if (!bus_.takeResult(result, this)) return;
        pending_ = false;
        if (result != i2cBus::Result::Ack) {
            fail(sensorError(result));
            return;
        }
        if ((initStep_ == 0 && buffer_[0] != 0xE5) ||
            (initStep_ == 1 && buffer_[0] != 0xD3) ||
            (initStep_ == 2 && (buffer_[0] != 'H' || buffer_[1] != '4' || buffer_[2] != '3')) ||
            (initStep_ == 3 && buffer_[0] != 0x55)) {
            fail(SensorError::WrongDevice);
            return;
        }
        if (initStep_ == 4) {
            for (uint8_t i = 0; i < 11; ++i) {
                calibration_[i] = static_cast<uint16_t>(readBigEndian16(buffer_ + 2 * i));
                if (calibration_[i] == 0 || calibration_[i] == 0xFFFF) {
                    fail(SensorError::InvalidData);
                    return;
                }
            }
        }
        ++initStep_;
    }

    // Skipped optional devices never touch the bus or require their identity.
    if ((!compass_ && (initStep_ == 2 || (initStep_ >= 12 && initStep_ <= 14))) ||
        (!barometer_ && (initStep_ == 3 || initStep_ == 4))) {
        ++initStep_;
        return;
    }

    switch (initStep_) {
        case 0: pending_ = bus_.readRegisters(accelAddress_, 0x00, buffer_, 1, nowUs, this); break;
        case 1: pending_ = bus_.readRegisters(gyroAddress_, 0x0F, buffer_, 1, nowUs, this); break;
        case 2: pending_ = bus_.readRegisters(COMPASS_ADDRESS, 0x0A, buffer_, 3, nowUs, this); break;
        case 3: pending_ = bus_.readRegisters(BAROMETER_ADDRESS, 0xD0, buffer_, 1, nowUs, this); break;
        case 4: pending_ = bus_.readRegisters(BAROMETER_ADDRESS, 0xAA, buffer_, 22, nowUs, this); break;
        case 5: pending_ = bus_.writeRegister(accelAddress_, 0x2D, 0x00, nowUs, this); break;
        case 6: pending_ = bus_.writeRegister(accelAddress_, 0x31, 0x08, nowUs, this); break; // Full resolution, +/-2 g.
        case 7: pending_ = bus_.writeRegister(accelAddress_, 0x2C, 0x0B, nowUs, this); break; // 200 Hz.
        case 8: pending_ = bus_.writeRegister(accelAddress_, 0x2D, 0x08, nowUs, this); break; // Measurement mode.
        case 9: pending_ = bus_.writeRegister(gyroAddress_, 0x20, 0x00, nowUs, this); break;
        case 10: pending_ = bus_.writeRegister(gyroAddress_, 0x23, 0x80, nowUs, this); break; // BDU, little endian, +/-250 dps.
        case 11: pending_ = bus_.writeRegister(gyroAddress_, 0x20, 0x6F, nowUs, this); break; // 200 Hz, 50 Hz LPF, XYZ enabled.
        case 12: pending_ = bus_.writeRegister(COMPASS_ADDRESS, 0x00, 0x78, nowUs, this); break; // 8 averages, 75 Hz.
        case 13: pending_ = bus_.writeRegister(COMPASS_ADDRESS, 0x01, 0x20, nowUs, this); break; // +/-1.3 gauss.
        case 14: pending_ = bus_.writeRegister(COMPASS_ADDRESS, 0x02, 0x00, nowUs, this); break; // Continuous conversion.
        case 15: pending_ = bus_.writeRegister(accelAddress_, 0x38, 0x00, nowUs, this); break; // FIFO bypass.
        case 16: pending_ = bus_.writeRegister(accelAddress_, 0x2E, 0x00, nowUs, this); break; // Poll status; no interrupt pins.
        case 17: pending_ = bus_.writeRegister(gyroAddress_, 0x21, 0x00, nowUs, this); break;
        case 18: pending_ = bus_.writeRegister(gyroAddress_, 0x22, 0x00, nowUs, this); break;
        case 19: pending_ = bus_.writeRegister(gyroAddress_, 0x24, 0x00, nowUs, this); break; // HPF and FIFO disabled.
        case 20: pending_ = bus_.writeRegister(gyroAddress_, 0x2E, 0x00, nowUs, this); break;
        default:
            waitStartUs_ = nowUs;
            state_ = State::Settling;
            break;
    }
}

void GY80::startJob(uint32_t nowUs) {
    switch (job_) {
        case Job::AccelStatus:
            pending_ = bus_.readRegisters(accelAddress_, 0x30, buffer_, 1, nowUs, this);
            break;
        case Job::AccelData:
            pending_ = bus_.readRegisters(accelAddress_, 0x32, buffer_, 6, nowUs, this);
            if (pending_) sample_.timestampUs = nowUs;
            break;
        case Job::GyroData:
            // Bit 7 enables L3G4200D address auto-increment. Read STATUS + XYZ.
            pending_ = bus_.readRegisters(gyroAddress_, 0xA7, buffer_, 7, nowUs, this);
            break;
        case Job::MagStatus:
            pending_ = bus_.readRegisters(COMPASS_ADDRESS, 0x09, buffer_, 1, nowUs, this);
            break;
        case Job::MagData:
            pending_ = bus_.readRegisters(COMPASS_ADDRESS, 0x03, buffer_, 6, nowUs, this);
            if (pending_) magnetic_.timestampUs = nowUs;
            break;
        case Job::TempCommand:
            pending_ = bus_.writeRegister(BAROMETER_ADDRESS, 0xF4, 0x2E, nowUs, this);
            break;
        case Job::TempData:
            pending_ = bus_.readRegisters(BAROMETER_ADDRESS, 0xF6, buffer_, 2, nowUs, this);
            break;
        case Job::PressureCommand:
            pending_ = bus_.writeRegister(BAROMETER_ADDRESS, 0xF4, 0x34, nowUs, this);
            break;
        case Job::PressureData:
            pending_ = bus_.readRegisters(BAROMETER_ADDRESS, 0xF6, buffer_, 3, nowUs, this);
            break;
        default: break;
    }
}

void GY80::finishJob(uint32_t nowUs) {
    switch (job_) {
        case Job::AccelStatus:
            job_ = (buffer_[0] & 0x80) ? Job::AccelData : Job::None;
            break;
        case Job::AccelData:
            sample_.accelerationMps2 = {
                readLittleEndian16(buffer_) * ACCEL_SCALE,
                readLittleEndian16(buffer_ + 2) * ACCEL_SCALE,
                readLittleEndian16(buffer_ + 4) * ACCEL_SCALE
            };
            job_ = Job::GyroData;
            break;
        case Job::GyroData:
            if (buffer_[0] & 0x08) {
                sample_.angularVelocityRadS = {
                    readLittleEndian16(buffer_ + 1) * GYRO_SCALE,
                    readLittleEndian16(buffer_ + 3) * GYRO_SCALE,
                    readLittleEndian16(buffer_ + 5) * GYRO_SCALE
                };
                sampleAvailable_ = true;
            }
            job_ = Job::None;
            break;
        case Job::MagStatus:
            // If LOCK is set, reading all six bytes also releases the lock.
            // Drain a locked/partial old reading without publishing it.
            magneticFresh_ = (buffer_[0] & 0x03) == 0x01;
            job_ = (buffer_[0] & 0x03) ? Job::MagData : Job::None;
            break;
        case Job::MagData: {
            if (!magneticFresh_) {
                job_ = Job::None;
                break;
            }
            int16_t x = readBigEndian16(buffer_);
            int16_t z = readBigEndian16(buffer_ + 2);
            int16_t y = readBigEndian16(buffer_ + 4);
            if (x == -4096 || y == -4096 || z == -4096) {
                fail(SensorError::InvalidData);
                return;
            }
            magnetic_.fieldMicrotesla = {x * MAG_SCALE, y * MAG_SCALE, z * MAG_SCALE};
            magneticAvailable_ = true;
            job_ = Job::None;
            break;
        }
        case Job::TempCommand:
            pressureWaitUs_ = nowUs;
            pressureState_ = PressureState::WaitTemperature;
            job_ = Job::None;
            break;
        case Job::TempData:
            rawTemperature_ = static_cast<uint16_t>(readBigEndian16(buffer_));
            pressureState_ = PressureState::StartPressure;
            job_ = Job::None;
            break;
        case Job::PressureCommand:
            pressureWaitUs_ = nowUs;
            pressureState_ = PressureState::WaitPressure;
            job_ = Job::None;
            break;
        case Job::PressureData: {
            uint32_t raw = (static_cast<uint32_t>(buffer_[0]) << 16) |
                           (static_cast<uint32_t>(buffer_[1]) << 8) | buffer_[2];
            if (!compensatePressure(raw >> 8)) {
                fail(SensorError::InvalidData);
                return;
            }
            pressureAvailable_ = true;
            pressureState_ = PressureState::Idle;
            job_ = Job::None;
            break;
        }
        default: break;
    }
}

void GY80::service(uint32_t nowUs) {
    if (state_ == State::PowerWait || state_ == State::Settling) {
        const uint32_t waitUs = state_ == State::PowerWait ? STARTUP_WAIT_US : GYRO_SETTLE_US;
        if (static_cast<uint32_t>(nowUs - waitStartUs_) < waitUs) return;
        state_ = state_ == State::PowerWait ? State::Initializing : State::Ready;
    }
    if (state_ == State::Initializing) {
        serviceInit(nowUs);
        return;
    }
    if (!ready()) return;

    if (pending_) {
        i2cBus::Result result;
        if (!bus_.takeResult(result, this)) return;
        pending_ = false;
        if (result != i2cBus::Result::Ack) {
            fail(sensorError(result));
            return;
        }
        finishJob(nowUs);
    }

    if (job_ == Job::None) {
        if (pressureState_ == PressureState::WaitTemperature &&
            static_cast<uint32_t>(nowUs - pressureWaitUs_) >= CONVERSION_US) {
            pressureState_ = PressureState::ReadTemperature;
        } else if (pressureState_ == PressureState::WaitPressure &&
                   static_cast<uint32_t>(nowUs - pressureWaitUs_) >= CONVERSION_US) {
            pressureState_ = PressureState::ReadPressure;
        }
        switch (pressureState_) {
            case PressureState::StartTemperature: job_ = Job::TempCommand; break;
            case PressureState::ReadTemperature: job_ = Job::TempData; break;
            case PressureState::StartPressure: job_ = Job::PressureCommand; break;
            case PressureState::ReadPressure: job_ = Job::PressureData; break;
            default: break;
        }
    }
    startJob(nowUs);
}

bool GY80::compensatePressure(uint32_t rawPressure) {
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
    if (x1 + md == 0) return false;
    int64_t x2 = (static_cast<int64_t>(mc) * 2048) / (x1 + md);
    const int64_t b5 = x1 + x2;
    const int64_t temperature = (b5 + 8) >> 4;
    if (temperature < -400 || temperature > 850) return false;
    const int64_t b6 = b5 - 4000;
    x1 = (b2 * ((b6 * b6) >> 12)) >> 11;
    x2 = (ac2 * b6) >> 11;
    const int64_t b3 = (ac1 * 4 + x1 + x2 + 2) >> 2;
    x1 = (ac3 * b6) >> 13;
    x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
    const int64_t x3 = (x1 + x2 + 2) >> 2;
    const int64_t b4 = (ac4 * (x3 + 32768)) >> 15;
    if (b4 <= 0 || static_cast<int64_t>(rawPressure) < b3) return false;
    const int64_t b7 = (static_cast<int64_t>(rawPressure) - b3) * 50000;
    int64_t pressure = (b7 * 2) / b4;
    if (pressure <= 0 || pressure > 200000) return false;
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
