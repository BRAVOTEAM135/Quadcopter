#pragma once
#include "test_support.h"

// Hardware-side fixtures, independent of driver classes or their private states.
struct SensorFixture {
    uint8_t accelAddress, gyroAddress, mpuAddress, pressureCommand;
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
    int16_t mpuAx, mpuAy, mpuAz;
    bool accelFresh, gyroFresh, magneticFresh, mpuFresh;
    bool magneticLocked, magneticOverflow, wrongIdentity, invalidCalibration;
    bool hold, nackNext;
    uint16_t accelReads, gyroReads, magneticReads, pressureReads, mpuReads;
};
SensorFixture sensors;
const uint8_t calibration[] = {0x01, 0x98, 0xFF, 0xB8, 0xC7, 0xD1, 0x7F, 0xE5,
                               0x7F, 0xF5, 0x5A, 0x71, 0x18, 0x2E, 0,    4,
                               0x80, 0,    0xDD, 0xF9, 0x0B, 0x34};

/// Reset the simulated chips to a stationary, valid default configuration.
void resetSensors() {
    sensors = {};
    sensors.accelAddress = 0x53;
    sensors.gyroAddress = 0x69;
    sensors.mpuAddress = 0x68;
    sensors.az = 256;
    sensors.mpuAz = 16384;
    sensors.mx = 349;
    sensors.accelFresh = sensors.gyroFresh = sensors.magneticFresh = sensors.mpuFresh = true;
    serviceBusInHelper = false; // Production read() must service STOP and timeouts.
}

/// Extract one byte from an independent signed-axis fixture in the chip's byte order.
uint8_t axisByte(int16_t x, int16_t y, int16_t z, uint8_t index, bool bigEndian) {
    int16_t value = index < 2 ? x : index < 4 ? y : z;
    bool high = bool(index & 1) != bigEndian;
    return high ? static_cast<uint16_t>(value) >> 8 : static_cast<uint8_t>(value);
}

/// Return a hardware register byte, including identity, freshness and Bosch reference data.
uint8_t sensorByte(uint8_t address, uint8_t reg, uint8_t index) {
    if (address == sensors.accelAddress && reg == 0x00) {
        return sensors.wrongIdentity ? 0 : 0xE5;
    }
    if (address == sensors.accelAddress && reg == 0x30) {
        return sensors.accelFresh ? 0x80 : 0;
    }
    if (address == sensors.accelAddress && reg == 0x32) {
        if (index == 0) {
            ++sensors.accelReads;
        }
        return axisByte(sensors.ax, sensors.ay, sensors.az, index, false);
    }
    if (address == sensors.gyroAddress && reg == 0x0F) {
        return sensors.wrongIdentity ? 0 : 0xD3;
    }
    if (address == sensors.gyroAddress && reg == 0xA7) {
        if (index == 0) {
            ++sensors.gyroReads;
            return sensors.gyroFresh ? 8 : 0;
        }
        return axisByte(sensors.gx, sensors.gy, sensors.gz, index - 1, false);
    }
    if (address == 0x1E && reg == 0x0A) {
        const uint8_t id[] = {'H', '4', '3'};
        return sensors.wrongIdentity ? 0 : id[index];
    }
    if (address == 0x1E && reg == 0x09) {
        return sensors.magneticLocked ? 3 : sensors.magneticFresh ? 1 : 0;
    }
    if (address == 0x1E && reg == 0x03) {
        if (index == 0) {
            ++sensors.magneticReads;
        }
        return axisByte(sensors.magneticOverflow ? -4096 : sensors.mx, sensors.mz, sensors.my,
                        index, true);
    }
    if (address == 0x77 && reg == 0xD0) {
        return sensors.wrongIdentity ? 0 : 0x55;
    }
    if (address == 0x77 && reg == 0xAA) {
        return sensors.invalidCalibration ? 0 : calibration[index];
    }
    if (address == 0x77 && reg == 0xF6) {
        if (sensors.pressureCommand == 0x2E) {
            return index == 0 ? 0x6C : 0xFA;
        }
        if (index == 0) {
            ++sensors.pressureReads;
        }
        const uint8_t pressure[] = {0x5D, 0x23, 0};
        return pressure[index];
    }
    if (address == sensors.mpuAddress && reg == 0x75) {
        return sensors.wrongIdentity ? 0 : 0x68;
    }
    if (address == sensors.mpuAddress && reg == 0x3A) {
        if (index == 0) {
            ++sensors.mpuReads;
            return sensors.mpuFresh ? 1 : 0;
        }
        if (index <= 6) {
            return axisByte(sensors.mpuAx, sensors.mpuAy, sensors.mpuAz, index - 1, true);
        }
        if (index == 7) {
            return 0x01;
        }
        if (index == 8) {
            return 0x54; // Temperature raw 340 -> 37.53 degrees C.
        }
        return axisByte(sensors.gx, sensors.gy, sensors.gz, index - 9, true);
    }
    CHECK(false); // Unexpected device/register access.
    return 0;
}

/// Check configuration writes against the chip register settings used by the project.
void checkSensorWrite(uint8_t address, uint8_t reg, uint8_t value) {
    bool valid = false;
    if (address == sensors.accelAddress) {
        valid = (reg == 0x2D && (value == 0 || value == 8)) || (reg == 0x31 && value == 8) ||
                (reg == 0x2C && value == 0x0B) || ((reg == 0x38 || reg == 0x2E) && value == 0);
    }
    if (address == sensors.gyroAddress) {
        valid |= (reg == 0x20 && (value == 0 || value == 0x6F)) ||
                 (reg == 0x23 && value == 0x80) ||
                 ((reg == 0x21 || reg == 0x22 || reg == 0x24 || reg == 0x2E) && value == 0);
    }
    if (address == sensors.mpuAddress) {
        valid |= (reg == 0x6B && (value == 0x80 || value == 1)) ||
                 (reg == 0x6C && value == 0) || (reg == 0x1A && value == 3) ||
                 (reg == 0x19 && value == 4) || ((reg == 0x1B || reg == 0x1C) && value == 0) ||
                 (reg == 0x38 && value == 1);
    }
    if (address == 0x1E) {
        valid = (reg == 0 && value == 0x78) || (reg == 1 && value == 0x20) ||
                (reg == 2 && value == 0);
    }
    if (address == 0x77) {
        valid = reg == 0xF4 && (value == 0x2E || value == 0x34);
        sensors.pressureCommand = value;
    }
    CHECK(valid);
}

/// Simulate a single queued TWI transaction; production code still progresses only in read().
void respondToSensor() {
    if (sensors.hold) {
        return;
    }
    if (TWCR & (1 << TWSTO)) {
        TWCR &= ~(1 << TWSTO);
        return;
    }
    if (!(TWCR & (1 << TWSTA))) {
        return;
    }
    interrupt(0x08);
    uint8_t address = TWDR >> 1;
    CHECK((TWDR & 1) == 0);
    if (sensors.nackNext) {
        sensors.nackNext = false;
        interrupt(0x20);
        finishStop();
        return;
    }
    interrupt(0x18);
    uint8_t reg = TWDR;
    interrupt(0x28);
    if (TWCR & (1 << TWSTA)) {
        interrupt(0x10);
        CHECK(TWDR == static_cast<uint8_t>((address << 1) | 1));
        interrupt(0x40);
        for (uint8_t index = 0;; ++index) {
            CHECK(index < 32);
            bool ack = TWCR & (1 << TWEA);
            interrupt(ack ? 0x50 : 0x58, sensorByte(address, reg, index));
            if (!ack) {
                break;
            }
        }
    } else {
        checkSensorWrite(address, reg, TWDR);
        interrupt(0x28);
    }
    finishStop();
}

/// Poll only the public read/status API until initialization completes, with a finite limit.
template <typename Driver, typename Sample> void waitReady(Driver& driver, Sample& sample) {
    for (uint16_t i = 0; i < 10000; ++i) {
        driver.read(sample, nowUs);
        respondToSensor();
        nowUs += 100;
        if (driver.getStatus().ready) {
            return;
        }
        if (driver.getStatus().error != DriverError::None) {
            break;
        }
    }
    CHECK(false);
}

/// Poll only read() until one fresh result arrives; no private driver method is used.
template <typename Driver, typename Sample> void waitSample(Driver& driver, Sample& sample) {
    for (uint16_t i = 0; i < 1000; ++i) {
        bool fresh = driver.read(sample, nowUs);
        respondToSensor();
        nowUs += 100;
        if (fresh) {
            return;
        }
        if (driver.getStatus().error != DriverError::None) {
            break;
        }
    }
    CHECK(false);
}
