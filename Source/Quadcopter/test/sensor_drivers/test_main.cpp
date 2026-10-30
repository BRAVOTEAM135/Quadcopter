#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "i2cBus.h"
#include "GY80.h"
#include "MPU6050.h"
#include "imu.h"
#include <math.h>

volatile uint8_t TWCR, TWSR, TWBR, TWDR;
volatile uint16_t tests_run = 0;
volatile uint16_t failed_line = 0;

__attribute__((noinline)) void test_done() {
    asm volatile("nop");
}

void check(bool ok, uint16_t line) {
    ++tests_run;
    if (!ok) {
        failed_line = line;
        test_done();
        for (;;) {}
    }
}
#define CHECK(condition) check((condition), __LINE__)

i2cBus bus;
uint32_t nowUs = 0;

void interrupt(uint8_t status, uint8_t value = 0) {
    TWSR = status;
    TWDR = value;
    test_twi_interrupt();
}

void finishStop() {
    CHECK(TWCR & (1 << TWSTO));
    TWCR &= ~(1 << TWSTO); // Model hardware completing STOP.
    bus.service(nowUs);
}

void registerPrefix(uint8_t address, uint8_t reg) {
    CHECK(TWCR & (1 << TWSTA));
    interrupt(0x08);
    CHECK(TWDR == static_cast<uint8_t>(address << 1));
    interrupt(0x18);
    CHECK(TWDR == reg);
    interrupt(0x28);
}

void completeRead(uint8_t address, uint8_t reg, const uint8_t* data, uint8_t length) {
    registerPrefix(address, reg);
    CHECK(TWCR & (1 << TWSTA));
    interrupt(0x10);
    CHECK(TWDR == static_cast<uint8_t>((address << 1) | 1));
    interrupt(0x40);
    for (uint8_t i = 0; i < length; ++i) {
        CHECK(bool(TWCR & (1 << TWEA)) == (i + 1 < length));
        interrupt(i + 1 == length ? 0x58 : 0x50, data[i]);
    }
    finishStop();
}

void completeWrite(uint8_t address, uint8_t reg, uint8_t value) {
    registerPrefix(address, reg);
    CHECK(TWDR == value);
    interrupt(0x28);
    finishStop();
}

bool near(float actual, float expected, float tolerance = 0.001f) {
    float difference = actual - expected;
    return difference > -tolerance && difference < tolerance;
}

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

void initMpu(MPU6050& imu, uint8_t address, bool startInitialization = true) {
    if (startInitialization) CHECK(imu.begin(nowUs));
    CHECK(!imu.requestSample(nowUs));
    imu.service(nowUs);
    CHECK(!(TWCR & (1 << TWSTA)));
    nowUs += 100000;
    imu.service(nowUs);
    const uint8_t identity[] = {0x68};
    completeRead(address, 0x75, identity, 1);
    imu.service(nowUs);
    completeWrite(address, 0x6B, 0x80);
    imu.service(nowUs);
    nowUs += 100000;
    imu.service(nowUs);
    completeWrite(address, 0x6B, 0x01);
    imu.service(nowUs);
    nowUs += 100000;
    imu.service(nowUs);
    const uint8_t regs[] = {0x6C, 0x1A, 0x19, 0x1B, 0x1C, 0x38};
    const uint8_t vals[] = {0x00, 0x03, 0x04, 0x00, 0x00, 0x01};
    for (uint8_t i = 0; i < sizeof(regs); ++i) {
        completeWrite(address, regs[i], vals[i]);
        imu.service(nowUs);
    }
    CHECK(imu.ready() && !imu.busy() && imu.error() == SensorError::None);
}

void testMpu() {
    bus.init();
    MPU6050 imu(bus, 0x69);
    initMpu(imu, 0x69);
    ImuSample sample;
    const uint8_t data[] = {
        1, 0x40, 0, 0xC0, 0, 0, 0, // +/-1 g.
        0x01, 0x54, // Temperature raw 340.
        0, 0x83, 0xFF, 0x7D, 0x80, 0 // +/-1 degree/s, -32768.
    };
    uint32_t timestamp = nowUs;
    CHECK(imu.requestSample(nowUs));
    CHECK(!imu.requestSample(nowUs));
    completeRead(0x69, 0x3A, data, sizeof(data));
    imu.service(nowUs);
    CHECK(!imu.requestSample(nowUs)); // Unread output is preserved.
    CHECK(imu.takeSample(sample));
    CHECK(!imu.takeSample(sample));
    CHECK(sample.timestampUs == timestamp);
    CHECK(near(sample.accelerationMps2.x, 9.80665f));
    CHECK(near(sample.accelerationMps2.y, -9.80665f));
    CHECK(near(sample.angularVelocityRadS.x, 0.01745329252f));
    CHECK(near(sample.angularVelocityRadS.y, -0.01745329252f));
    CHECK(near(sample.angularVelocityRadS.z, -4.3658068f));
    CHECK(near(imu.temperatureC(), 37.53f));
    uint8_t stale[15] = {};
    CHECK(imu.requestSample(nowUs));
    completeRead(0x69, 0x3A, stale, sizeof(stale));
    imu.service(nowUs);
    CHECK(!imu.takeSample(sample) && imu.ready());

    CHECK(imu.requestSample(nowUs));
    nowUs += 5000;
    bus.service(nowUs);
    imu.service(nowUs);
    CHECK(imu.error() == SensorError::Timeout && imu.ready());
    CHECK(imu.requestSample(nowUs)); // Recover by issuing another sample.
    completeRead(0x69, 0x3A, data, sizeof(data));
    imu.service(nowUs);
    CHECK(imu.takeSample(sample) && imu.error() == SensorError::None);

    CHECK(imu.begin(nowUs));
    nowUs += 100000;
    imu.service(nowUs);
    const uint8_t wrong[] = {0x70};
    completeRead(0x69, 0x75, wrong, 1);
    imu.service(nowUs);
    CHECK(!imu.ready() && !imu.busy() && imu.error() == SensorError::WrongDevice);
}

// Bosch datasheet's calibration and uncompensated measurement example.
const uint8_t calibration[] = {
    0x01, 0x98, 0xFF, 0xB8, 0xC7, 0xD1, 0x7F, 0xE5,
    0x7F, 0xF5, 0x5A, 0x71, 0x18, 0x2E, 0, 4,
    0x80, 0, 0xDD, 0xF9, 0x0B, 0x34
};

void initGy80(GY80& imu, bool auxiliaries = true, bool startInitialization = true,
              bool compassOnly = false) {
    if (startInitialization) CHECK(imu.begin(nowUs, auxiliaries, auxiliaries && !compassOnly));
    nowUs += 100000;
    imu.service(nowUs);
    const uint8_t accelId[] = {0xE5}, gyroId[] = {0xD3};
    const uint8_t magId[] = {'H', '4', '3'}, baroId[] = {0x55};
    completeRead(0x53, 0x00, accelId, 1);
    imu.service(nowUs);
    completeRead(0x69, 0x0F, gyroId, 1);
    imu.service(nowUs);
    if (auxiliaries) {
        completeRead(0x1E, 0x0A, magId, 3);
        imu.service(nowUs);
        if (compassOnly) {
            imu.service(nowUs);
            imu.service(nowUs);
        } else {
            completeRead(0x77, 0xD0, baroId, 1);
            imu.service(nowUs);
            completeRead(0x77, 0xAA, calibration, sizeof(calibration));
            imu.service(nowUs);
        }
    } else {
        // Skipping disabled devices should generate no bus transactions.
        imu.service(nowUs);
        imu.service(nowUs);
        imu.service(nowUs);
    }
    const uint8_t addresses[] = {0x53, 0x53, 0x53, 0x53, 0x69, 0x69, 0x69, 0x1E, 0x1E, 0x1E,
                                 0x53, 0x53, 0x69, 0x69, 0x69, 0x69};
    const uint8_t regs[] = {0x2D, 0x31, 0x2C, 0x2D, 0x20, 0x23, 0x20, 0x00, 0x01, 0x02,
                           0x38, 0x2E, 0x21, 0x22, 0x24, 0x2E};
    const uint8_t values[] = {0x00, 0x08, 0x0B, 0x08, 0x00, 0x80, 0x6F, 0x78, 0x20, 0x00,
                             0, 0, 0, 0, 0, 0};
    for (uint8_t i = 0; i < sizeof(regs); ++i) {
        if (!auxiliaries && i >= 7 && i <= 9) {
            imu.service(nowUs);
            continue;
        }
        completeWrite(addresses[i], regs[i], values[i]);
        imu.service(nowUs);
    }
    CHECK(!imu.ready());
    nowUs += 299999;
    imu.service(nowUs);
    CHECK(!imu.ready());
    ++nowUs;
    imu.service(nowUs);
    CHECK(imu.ready() && !imu.busy() && imu.error() == SensorError::None);
}

template <typename Sensor>
void sampleGy80(Sensor& imu) {
    const uint8_t ready[] = {0x80};
    const uint8_t accel[] = {0, 1, 0, 0xFF, 0, 0}; // +/-256 counts.
    const uint8_t gyro[] = {8, 0xE8, 3, 0x18, 0xFC, 0, 0}; // +/-1000 counts.
    CHECK(imu.requestSample(nowUs));
    completeRead(0x53, 0x30, ready, 1);
    imu.service(nowUs);
    completeRead(0x53, 0x32, accel, sizeof(accel));
    imu.service(nowUs);
    completeRead(0x69, 0xA7, gyro, sizeof(gyro));
    imu.service(nowUs);
    ImuSample sample;
    CHECK(imu.takeSample(sample));
    CHECK(near(sample.accelerationMps2.x, 9.80665f));
    CHECK(near(sample.accelerationMps2.y, -9.80665f));
    CHECK(near(sample.angularVelocityRadS.x, 0.1527163f));
    CHECK(near(sample.angularVelocityRadS.y, -0.1527163f));
}

void testGy80() {
    bus.init();
    GY80 imu(bus);
    initGy80(imu);
    sampleGy80(imu);
    const uint8_t ready[] = {1};
    // Wire order is X/Z/Y, returned order must be X/Y/Z.
    const uint8_t mag[] = {0x04, 0x42, 0xFB, 0xBE, 0, 0};
    CHECK(imu.requestMagneticField(nowUs));
    completeRead(0x1E, 0x09, ready, 1);
    imu.service(nowUs);
    completeRead(0x1E, 0x03, mag, sizeof(mag));
    imu.service(nowUs);
    MagneticSample magnetic;
    CHECK(imu.takeMagneticField(magnetic));
    CHECK(near(magnetic.fieldMicrotesla.x, 100));
    CHECK(near(magnetic.fieldMicrotesla.y, 0));
    CHECK(near(magnetic.fieldMicrotesla.z, -100));

    CHECK(imu.requestMagneticField(nowUs));
    const uint8_t locked[] = {3};
    completeRead(0x1E, 0x09, locked, 1);
    imu.service(nowUs);
    completeRead(0x1E, 0x03, mag, sizeof(mag));
    imu.service(nowUs);
    CHECK(!imu.takeMagneticField(magnetic)); // Drain old partial data only.

    CHECK(imu.requestPressure(nowUs));
    imu.service(nowUs);
    completeWrite(0x77, 0xF4, 0x2E);
    imu.service(nowUs);
    CHECK(imu.busy());
    sampleGy80(imu); // Motion remains available during temperature conversion.
    nowUs += 4500;
    imu.service(nowUs);
    const uint8_t temperature[] = {0x6C, 0xFA}; // 27898.
    completeRead(0x77, 0xF6, temperature, 2);
    imu.service(nowUs);
    completeWrite(0x77, 0xF4, 0x34);
    imu.service(nowUs);
    nowUs += 4500;
    imu.service(nowUs);
    const uint8_t pressure[] = {0x5D, 0x23, 0}; // 23843 at OSS=0.
    completeRead(0x77, 0xF6, pressure, 3);
    imu.service(nowUs);
    PressureSample measured;
    CHECK(imu.takePressure(measured));
    CHECK(measured.pressurePa == 69964);
    CHECK(near(measured.temperatureC, 15));
    CHECK(!imu.busy());

    CHECK(imu.requestMagneticField(nowUs));
    completeRead(0x1E, 0x09, ready, 1);
    imu.service(nowUs);
    const uint8_t overflow[] = {0xF0, 0, 0, 0, 0, 0};
    completeRead(0x1E, 0x03, overflow, sizeof(overflow));
    imu.service(nowUs);
    CHECK(!imu.takeMagneticField(magnetic) && imu.error() == SensorError::InvalidData);

    const uint8_t stale[] = {0};
    CHECK(imu.requestSample(nowUs));
    completeRead(0x53, 0x30, stale, 1);
    imu.service(nowUs);
    ImuSample sample;
    CHECK(!imu.takeSample(sample) && !imu.busy());

    CHECK(imu.requestSample(nowUs));
    interrupt(0x20);
    finishStop();
    imu.service(nowUs);
    CHECK(imu.error() == SensorError::Nack && !imu.busy());
    sampleGy80(imu); // Sample requests recover after transfer errors.
}

void testDisabledAuxiliaries() {
    bus.init();
    GY80 imu(bus);
    initGy80(imu, false);
    CHECK(!imu.requestPressure(nowUs));
    CHECK(!imu.requestMagneticField(nowUs));
    sampleGy80(imu);
}

void testInitFailures() {
    bus.init();
    GY80 imu(bus);
    CHECK(imu.begin(nowUs));
    nowUs += 100000;
    imu.service(nowUs);
    const uint8_t wrongId[] = {0};
    completeRead(0x53, 0x00, wrongId, 1);
    imu.service(nowUs);
    CHECK(!imu.ready() && !imu.busy() && imu.error() == SensorError::WrongDevice);

    CHECK(imu.begin(nowUs));
    nowUs += 100000;
    imu.service(nowUs);
    const uint8_t accelId[] = {0xE5}, gyroId[] = {0xD3};
    const uint8_t magId[] = {'H', '4', '3'}, baroId[] = {0x55};
    completeRead(0x53, 0x00, accelId, 1);
    imu.service(nowUs);
    completeRead(0x69, 0x0F, gyroId, 1);
    imu.service(nowUs);
    completeRead(0x1E, 0x0A, magId, 3);
    imu.service(nowUs);
    completeRead(0x77, 0xD0, baroId, 1);
    imu.service(nowUs);
    const uint8_t invalidCalibration[22] = {};
    completeRead(0x77, 0xAA, invalidCalibration, sizeof(invalidCalibration));
    imu.service(nowUs);
    CHECK(!imu.ready() && !imu.busy() && imu.error() == SensorError::InvalidData);

    CHECK(imu.begin(nowUs));
    nowUs += 100000;
    imu.service(nowUs);
    interrupt(0x20);
    finishStop();
    imu.service(nowUs);
    CHECK(!imu.ready() && !imu.busy() && imu.error() == SensorError::Nack);

    GY80 invalidGyro(bus, 0x53, 0x70);
    CHECK(!invalidGyro.begin(nowUs) && invalidGyro.error() == SensorError::WrongDevice);
    MPU6050 invalidMpu(bus, 0x67);
    CHECK(!invalidMpu.begin(nowUs) && invalidMpu.error() == SensorError::WrongDevice);
}

void testGy80Interface() {
    bus.init();
    GY80 sensor(bus);
    IMU imu(sensor);
    CHECK(!imu.ready() && !imu.requestSample(nowUs));
    CHECK(imu.begin(nowUs));
    CHECK(!imu.begin(nowUs)); // Initialization already active.
    // The common interface enables motion + compass, leaving pressure separate.
    initGy80(sensor, true, false, true);
    CHECK(imu.ready() && imu.error() == SensorError::None);
    sampleGy80(imu); // Request/service/take through the common interface.

    // service() has queued the slow compass acquisition after motion finished.
    const uint8_t magReady[] = {1};
    completeRead(0x1E, 0x09, magReady, 1);
    imu.service(nowUs);
    const uint8_t magData[] = {0x04, 0x42, 0, 0, 0, 0};
    completeRead(0x1E, 0x03, magData, sizeof(magData));
    imu.service(nowUs);
    MagneticSample magnetic;
    CHECK(imu.hasMagnetometer() && imu.takeMagneticField(magnetic));
    CHECK(near(magnetic.fieldMicrotesla.x, 100));
    CHECK(!imu.takeMagneticField(magnetic));

    CHECK(imu.begin(nowUs));
    nowUs += 100000;
    imu.service(nowUs);
    interrupt(0x20);
    finishStop();
    imu.service(nowUs);
    CHECK(!imu.ready() && imu.error() == SensorError::Nack);
}

void testMpuInterface() {
    bus.init();
    MPU6050 sensor(bus);
    IMU imu(sensor);
    MagneticSample magnetic;
    CHECK(!imu.hasMagnetometer() && !imu.takeMagneticField(magnetic));
    CHECK(!imu.ready() && !imu.requestSample(nowUs));
    CHECK(imu.begin(nowUs));
    initMpu(sensor, 0x68, false);
    CHECK(imu.ready() && imu.error() == SensorError::None);
    CHECK(imu.requestSample(nowUs));
    const uint8_t data[] = {1, 0x40, 0, 0, 0, 0, 0, 0, 0, 0, 0x83, 0, 0, 0, 0};
    completeRead(0x68, 0x3A, data, sizeof(data));
    imu.service(nowUs);
    ImuSample sample;
    CHECK(imu.takeSample(sample));
    CHECK(near(sample.accelerationMps2.x, 9.80665f));
    CHECK(near(sample.angularVelocityRadS.x, 0.01745329252f));
    CHECK(!imu.takeSample(sample));

    CHECK(imu.requestSample(nowUs));
    nowUs += 5000;
    bus.service(nowUs);
    imu.service(nowUs);
    CHECK(imu.ready() && imu.error() == SensorError::Timeout);
    CHECK(!imu.takeSample(sample));
}

int main() {
    testBus();
    testMpu();
    testGy80();
    testDisabledAuxiliaries();
    testInitFailures();
    testGy80Interface();
    testMpuInterface();
    test_done();
    for (;;) {}
}
