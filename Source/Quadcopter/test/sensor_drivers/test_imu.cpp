#include "sensor_fixture.h"
#include "imu.h"

/// Check startup zeroing and fresh snapshots through the public read/status interface.
int main() {
    bus.init();
    resetSensors();
    IMU imu(bus);
    ImuReading reading = {};
    CHECK(imu.init(nowUs));
    CHECK(!imu.init(nowUs));
    CHECK(!imu.getStatus().ready && imu.getStatus().busy);
    uint16_t samples = 0;
    uint32_t lastTimestamp = 0;
    for (uint16_t i = 0; i < 31000; ++i) {
        if (imu.read(reading, nowUs)) {
            ++samples;
            CHECK(reading.motion.timestampUs != lastTimestamp);
            lastTimestamp = reading.motion.timestampUs;
            CHECK(near(reading.motion.accelerationMps2.z, 9.80665f));
        }
        respondToSensor();
        nowUs += 100;
    }
    CHECK(samples >= 450);
    CHECK(imu.getStatus().ready && reading.orientation.valid);
    CHECK(near(reading.orientation.rollDeg(), 0));
    CHECK(near(reading.orientation.pitchDeg(), 0));
    CHECK(near(reading.orientation.yawDeg(), 0));
#ifdef IMU_USE_MPU6050
    CHECK(!reading.orientation.yawReferenced);
    CHECK(near(reading.motion.temperatureC, 37.53f));
#else
    CHECK(reading.orientation.yawReferenced);
    CHECK(sensors.pressureReads == 0);
#endif
    uint8_t control = TWCR;
    imu.getStatus();
    imu.getStatus();
    CHECK(TWCR == control);
    imu.reset();
    CHECK(!imu.getStatus().ready);
    imu.close();
    imu.close();
    respondToSensor();
    CHECK(!imu.getStatus().ready && !imu.getStatus().busy);
    CHECK(!imu.read(reading, nowUs));
    CHECK(imu.init(nowUs));
    sensors.wrongIdentity = true;
    for (uint16_t i = 0; i < 2000; ++i) {
        imu.read(reading, nowUs);
        respondToSensor();
        nowUs += 100;
        if (imu.getStatus().error != DriverError::None) {
            break;
        }
    }
    CHECK(imu.getStatus().error == DriverError::WrongDevice);
    CHECK(!imu.getStatus().ready && !imu.getStatus().busy);
    imu.close();
    bus.close();
    test_done();
    for (;;) {
    }
}
