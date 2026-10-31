#include "sensor_fixture.h"

/// Check transport sequencing, ownership and timeout recovery.
void testBus() {
    bus.init();
    uint8_t data[3] = {};
    i2cBus::Result result;
    CHECK(TWBR == 72);
    CHECK(!bus.startProbe(0x07, nowUs));
    CHECK(!bus.readRegisters(0x68, 0x00, nullptr, 1, nowUs));
    CHECK(!bus.readRegisters(0x68, 0x00, data, 0, nowUs));
    CHECK(!bus.readRegisters(0x68, 0x00, data, 33, nowUs));
    CHECK(bus.startProbe(0x53, nowUs));
    CHECK(!bus.startProbe(0x68, nowUs));
    interrupt(0x08);
    CHECK(TWDR == 0xA6);
    interrupt(0x18);
    CHECK(!bus.takeProbeResult(result)); // Must wait for STOP.
    finishStop();
    CHECK(!bus.takeResult(result)); // Register API cannot steal a probe.
    CHECK(bus.takeProbeResult(result) && result == i2cBus::Result::Ack);

    const uint8_t values[] = {0x80, 0x01, 0xFF};
    CHECK(bus.readRegisters(0x68, 0x3B, data, 3, nowUs, data));
    completeRead(0x68, 0x3B, values, 3);
    CHECK(!bus.takeProbeResult(result));
    CHECK(!bus.takeResult(result)); // Wrong owner cannot consume the result.
    CHECK(!bus.writeRegister(0x68, 0x6B, 1, nowUs));
    CHECK(bus.takeResult(result, data) && result == i2cBus::Result::Ack);
    CHECK(data[0] == 0x80 && data[1] == 0x01 && data[2] == 0xFF);

    CHECK(bus.readRegisters(0x68, 0x75, data, 1, nowUs));
    completeRead(0x68, 0x75, values, 1);
    CHECK(bus.takeResult(result) && result == i2cBus::Result::Ack);
    CHECK(bus.writeRegister(0x68, 0x6B, 1, nowUs));
    completeWrite(0x68, 0x6B, 1);
    CHECK(bus.takeResult(result) && result == i2cBus::Result::Ack);

    const uint8_t nacks[] = {0x20, 0x30, 0x48};
    for (uint8_t status : nacks) {
        CHECK(bus.readRegisters(0x68, 0x75, data, 1, nowUs));
        interrupt(status);
        finishStop();
        CHECK(bus.takeResult(result) && result == i2cBus::Result::Nack);
    }
    CHECK(bus.writeRegister(0x68, 0x6B, 1, nowUs));
    interrupt(0x38);
    CHECK(!(TWCR & (1 << TWSTO)));
    CHECK(bus.takeResult(result) && result == i2cBus::Result::BusError);
    CHECK(bus.writeRegister(0x68, 0x6B, 1, nowUs));
    interrupt(0x00);
    finishStop();
    CHECK(bus.takeResult(result) && result == i2cBus::Result::BusError);

    nowUs = 0xFFFFFF00UL;
    CHECK(bus.startProbe(0x53, nowUs));
    nowUs += 4999;
    bus.service(nowUs);
    CHECK(!bus.takeProbeResult(result));
    ++nowUs;
    bus.service(nowUs);
    CHECK(bus.takeProbeResult(result) && result == i2cBus::Result::Timeout);
    CHECK(bus.startProbe(0x53, nowUs));
    interrupt(0x18); // Leave STOP stuck; timeout must recover it too.
    nowUs += 5000;
    bus.service(nowUs);
    CHECK(bus.takeProbeResult(result) && result == i2cBus::Result::Timeout);
    nowUs = 0;
}

/// Check owner-specific cancellation and safe bus closure.
void testBusCloseAndCancel() {
    bus.close();
    bus.close();
    uint8_t data[3] = {};
    i2cBus::Result result;
    CHECK(!bus.startProbe(0x53, nowUs));
    CHECK(!bus.readRegisters(0x68, 0x3B, data, 3, nowUs));
    CHECK(!bus.writeRegister(0x68, 0x6B, 1, nowUs));
    interrupt(0x50, 0xAA); // A late handler cannot re-enable closed hardware.
    bus.service(nowUs);
    CHECK(TWCR == 0 && !bus.takeResult(result));

    bus.init();
    CHECK(bus.readRegisters(0x68, 0x3B, data, 3, nowUs, data));
    registerPrefix(0x68, 0x3B);
    interrupt(0x10);
    interrupt(0x40);
    interrupt(0x50, 0x11);
    CHECK(data[0] == 0x11);
    CHECK(!bus.cancel(&result)); // Another owner cannot cancel this transfer.
    CHECK(bus.cancel(data));
    CHECK(!(TWCR & (1 << TWIE)));
    interrupt(0x50, 0xAA);
    CHECK(data[1] == 0); // No ISR writes to a cancelled buffer.
    CHECK(!bus.startProbe(0x53, nowUs));
    finishStop();
    CHECK(!bus.takeResult(result, data));
    CHECK(bus.startProbe(0x53, nowUs));
    CHECK(!bus.cancel(nullptr)); // Register cancellation cannot steal a probe.
    interrupt(0x18);
    finishStop();
    CHECK(bus.takeProbeResult(result));

    // Cancellation also releases an unread completed result.
    const uint8_t value[] = {0x68};
    CHECK(bus.readRegisters(0x68, 0x75, data, 1, nowUs, data));
    completeRead(0x68, 0x75, value, 1);
    CHECK(bus.cancel(data));
    CHECK(!bus.takeResult(result, data));

    // A stuck cancellation STOP times out across clock wrap, freeing the bus.
    nowUs = 0xFFFFFF00UL;
    CHECK(bus.readRegisters(0x68, 0x75, data, 1, nowUs, data));
    CHECK(bus.cancel(data));
    nowUs += 5000;
    bus.service(nowUs);
    CHECK(bus.readRegisters(0x68, 0x75, data, 1, nowUs, data));
    bus.close(); // Abort with no further buffer access or results.
    interrupt(0x58, 0xAA);
    CHECK(data[0] == 0x68 && TWCR == 0);
    CHECK(!bus.takeResult(result, data));
    nowUs = 0;
}

/// Check read/status, signed scaling, stale samples, timeout recovery and reopening.
void testAcceleration() {
    bus.init();
    resetSensors();
    sensors.accelAddress = 0x1D;
    ADXL345 sensor(bus, 0x1D);
    AccelerationSample sample = {{11, 22, 33}, 1234};
    CHECK(!sensor.getStatus().ready && !sensor.getStatus().busy);
    CHECK(sensor.init(nowUs));
    CHECK(!sensor.init(nowUs));
    CHECK(!sensor.read(sample, nowUs));
    CHECK(sample.timestampUs == 1234 && sample.accelerationMps2.x == 11);
    waitReady(sensor, sample);
    sensors.ax = 256;
    sensors.ay = -256;
    sensors.az = 0;
    waitSample(sensor, sample);
    CHECK(near(sample.accelerationMps2.x, 9.80665f));
    CHECK(near(sample.accelerationMps2.y, -9.80665f));
    sensors.accelFresh = false;
    uint32_t saved = sample.timestampUs;
    for (uint8_t i = 0; i < 8; ++i) {
        CHECK(!sensor.read(sample, nowUs));
        respondToSensor();
        nowUs += 100;
    }
    CHECK(sample.timestampUs == saved);
    for (uint8_t i = 0; i < 4 && sensor.getStatus().busy; ++i) {
        sensor.read(sample, nowUs);
        respondToSensor();
    }
    sensors.hold = true;
    sensor.read(sample, nowUs);
    CHECK(sensor.getStatus().busy);
    uint8_t control = TWCR;
    nowUs += 5000;
    CHECK(sensor.getStatus().busy && TWCR == control); // Status never services timeouts.
    CHECK(!sensor.read(sample, nowUs));
    CHECK(sensor.getStatus().error == DriverError::Timeout && !sensor.getStatus().busy);
    sensors.hold = false;
    sensors.accelFresh = true;
    waitSample(sensor, sample);
    CHECK(sensor.getStatus().error == DriverError::None);
    sensor.read(sample, nowUs);
    sensor.close();
    sensor.close();
    CHECK(!sensor.getStatus().ready && !sensor.getStatus().busy);
    CHECK(!sensor.read(sample, nowUs));
    respondToSensor();
    CHECK(sensor.init(nowUs));
    waitReady(sensor, sample);
    waitSample(sensor, sample);
    sensor.close();
    bus.close();
}

/// Check gyro scale, settling, stale rejection and an alternate I2C address.
void testGyroscope() {
    bus.init();
    resetSensors();
    sensors.gyroAddress = 0x68;
    L3G4200D sensor(bus, 0x68);
    GyroscopeSample sample = {};
    CHECK(sensor.init(nowUs));
    waitReady(sensor, sample);
    sensors.gx = 1000;
    sensors.gy = -1000;
    waitSample(sensor, sample);
    CHECK(near(sample.angularVelocityRadS.x, 0.1527163f));
    CHECK(near(sample.angularVelocityRadS.y, -0.1527163f));
    sensors.gyroFresh = false;
    for (uint8_t i = 0; i < 8; ++i) {
        CHECK(!sensor.read(sample, nowUs));
        respondToSensor();
        nowUs += 100;
    }
    sensor.close();
    bus.close();
}

/// Check compass axis order, locked/overflow data and device identity rejection.
void testCompass() {
    bus.init();
    resetSensors();
    HMC5883L sensor(bus);
    MagneticSample sample = {};
    CHECK(sensor.init(nowUs));
    waitReady(sensor, sample);
    sensors.mx = 1090;
    sensors.my = 0;
    sensors.mz = -1090;
    waitSample(sensor, sample);
    CHECK(near(sample.fieldMicrotesla.x, 100) && near(sample.fieldMicrotesla.z, -100));
    uint32_t saved = sample.timestampUs;
    sensors.magneticLocked = true;
    for (uint8_t i = 0; i < 10; ++i) {
        CHECK(!sensor.read(sample, nowUs));
        respondToSensor();
        nowUs += 100;
    }
    CHECK(sample.timestampUs == saved);
    sensors.magneticLocked = false;
    sensors.magneticOverflow = true;
    for (uint8_t i = 0; i < 10; ++i) {
        sensor.read(sample, nowUs);
        respondToSensor();
        nowUs += 100;
        if (sensor.getStatus().error == DriverError::InvalidData) {
            break;
        }
    }
    CHECK(sensor.getStatus().error == DriverError::InvalidData);
    sensor.close();
    respondToSensor();
    CHECK(sensor.init(nowUs));
    sensors.wrongIdentity = true;
    for (uint16_t i = 0; i < 2000; ++i) {
        sensor.read(sample, nowUs);
        respondToSensor();
        nowUs += 100;
        if (sensor.getStatus().error != DriverError::None) {
            break;
        }
    }
    CHECK(!sensor.getStatus().ready && sensor.getStatus().error == DriverError::WrongDevice);
    sensor.close();
    bus.close();
}

/// Check motion/temperature arrive together and a failed read can be retried.
void testMpu() {
    bus.init();
    resetSensors();
    sensors.mpuAddress = 0x69;
    MPU6050 sensor(bus, 0x69);
    ImuSample sample = {};
    CHECK(sensor.init(nowUs));
    waitReady(sensor, sample);
    sensors.mpuAx = 16384;
    sensors.mpuAy = -16384;
    sensors.mpuAz = 0;
    sensors.gx = 131;
    sensors.gy = -131;
    waitSample(sensor, sample);
    CHECK(near(sample.accelerationMps2.x, 9.80665f) &&
          near(sample.accelerationMps2.y, -9.80665f));
    CHECK(near(sample.angularVelocityRadS.x, 0.01745329252f));
    CHECK(near(sample.temperatureC, 37.53f));
    sensors.nackNext = true;
    for (uint8_t i = 0; i < 5; ++i) {
        sensor.read(sample, nowUs);
        respondToSensor();
        nowUs += 100;
        if (sensor.getStatus().error != DriverError::None) {
            break;
        }
    }
    CHECK(sensor.getStatus().error == DriverError::Nack);
    waitSample(sensor, sample);
    sensor.close();
    bus.close();
}

/// Check nonblocking conversion and Bosch's independent pressure reference result.
void testPressure() {
    bus.init();
    resetSensors();
    BMP085 sensor(bus);
    PressureSample sample = {};
    CHECK(sensor.init(nowUs));
    waitReady(sensor, sample);
    waitSample(sensor, sample);
    CHECK(sample.pressurePa == 69964 && near(sample.temperatureC, 15));
    sensor.close();
    respondToSensor();
    CHECK(sensor.init(nowUs));
    sensors.invalidCalibration = true;
    for (uint16_t i = 0; i < 2000; ++i) {
        sensor.read(sample, nowUs);
        respondToSensor();
        nowUs += 100;
        if (sensor.getStatus().error != DriverError::None) {
            break;
        }
    }
    CHECK(sensor.getStatus().error == DriverError::InvalidData && !sensor.getStatus().ready);
    sensor.close();
    bus.close();
}

/// Check automatic multi-rate board acquisition, complete motion pairs and disabled chips.
void testBoard() {
    bus.init();
    resetSensors();
    GY80 board(bus);
    Gy80Sample sample = {};
    CHECK(board.init(nowUs));
    waitReady(board, sample);
    uint16_t motion = 0, magnetic = 0, pressure = 0;
    uint32_t end = nowUs + 400000UL;
    while (static_cast<int32_t>(end - nowUs) > 0) {
        if (board.read(sample, nowUs)) {
            if (sample.motionFresh) {
                ++motion;
                CHECK(near(sample.motion.accelerationMps2.z, 9.80665f));
            }
            if (sample.magneticFresh) {
                ++magnetic;
            }
            if (sample.pressureFresh) {
                ++pressure;
                CHECK(sample.pressure.pressurePa == 69964);
            }
        }
        respondToSensor();
        nowUs += 100;
    }
    CHECK(motion >= 65 && motion <= 81);
    CHECK(magnetic >= 8 && magnetic <= 11);
    CHECK(pressure >= 3 && pressure <= 5);
    sensors.gyroFresh = false;
    sensors.ax = 512;
    uint32_t savedMotionTime = sample.motion.timestampUs;
    float savedAcceleration = sample.motion.accelerationMps2.x;
    for (uint16_t i = 0; i < 700; ++i) {
        if (board.read(sample, nowUs)) {
            if (sample.motionFresh) {
                CHECK(i < 100);
                savedMotionTime = sample.motion.timestampUs;
                savedAcceleration = sample.motion.accelerationMps2.x;
            } else {
                CHECK(sample.motion.timestampUs == savedMotionTime);
                CHECK(sample.motion.accelerationMps2.x == savedAcceleration);
            }
        }
        respondToSensor();
        nowUs += 100;
    }
    board.close();
    board.close();
    respondToSensor();
    CHECK(!board.getStatus().ready && !board.getStatus().busy);
    CHECK(!board.read(sample, nowUs));
    resetSensors();
    CHECK(board.init(nowUs, false, false));
    waitReady(board, sample);
    waitSample(board, sample);
    CHECK(sample.motionFresh && !sample.magneticFresh && !sample.pressureFresh);
    CHECK(sensors.magneticReads == 0 && sensors.pressureReads == 0);
    board.close();
    bus.close();
}

/// Check invalid addresses and initialization across the microsecond clock wrap.
void testInitAndWrap() {
    bus.init();
    resetSensors();
    ADXL345 bad(bus, 0x70);
    CHECK(!bad.init(nowUs) && bad.getStatus().error == DriverError::WrongDevice);
    MPU6050 badMpu(bus, 0x67);
    CHECK(!badMpu.init(nowUs));
    nowUs = 0xFFFFFF00UL;
    ADXL345 sensor(bus);
    AccelerationSample sample = {};
    CHECK(sensor.init(nowUs));
    waitReady(sensor, sample);
    waitSample(sensor, sample);
    CHECK(near(sample.accelerationMps2.z, 9.80665f));
    sensor.close();
    bus.close();
}

/// Run transport and public sensor-API checks in the AVR CPU simulator.
int main() {
    testBus();
    testBusCloseAndCancel();
    testAcceleration();
    testGyroscope();
    testCompass();
    testMpu();
    testPressure();
    testBoard();
    testInitAndWrap();
    test_done();
    for (;;) {
    }
}
