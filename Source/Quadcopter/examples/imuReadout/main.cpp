#include <Arduino.h>
#include <stdio.h>
#include "i2cBus.h"
#include "serialPort.h"

#ifdef IMU_USE_MPU6050
#include "MPU6050.h"
using SelectedImu = MPU6050;
using SelectedReading = ImuSample;
static const char* const READY_MESSAGE = "MPU6050 ready\r\n";
#else
#include "GY80.h"
using SelectedImu = GY80;
using SelectedReading = Gy80Sample;
static const char* const READY_MESSAGE = "GY80 ready\r\n";
#endif

i2cBus bus;
SerialPort serialPort;
SelectedImu imu(bus);

namespace {
    /// Return a printable message for a sensor error, or nullptr when no error message is
    /// needed.
    const char* errorMessage(DriverError error) {
        switch (error) {
            case DriverError::Nack: return "IMU: NACK\r\n";
            case DriverError::BusError: return "IMU: BUS ERROR\r\n";
            case DriverError::Timeout: return "IMU: TIMEOUT\r\n";
            case DriverError::WrongDevice: return "IMU: WRONG DEVICE\r\n";
            case DriverError::InvalidData: return "IMU: INVALID DATA\r\n";
            default: return nullptr;
        }
    }
}

/// Initialize UART, the shared I2C bus and selected sensor for the readout example.
void setup() {
    serialPort.init(115200);
    bus.init();
    imu.init(micros());
}

/// Poll read() on every pass and report motion/errors without waiting for serial output.
void loop() {
    const uint32_t nowUs = micros();
    SelectedReading measurement;
    bool newReading = imu.read(measurement, nowUs);

    // Retry complete serial messages without blocking bus service.
    static DriverError reportedError = DriverError::None;
    if (imu.getStatus().error != reportedError) {
        const char* message = errorMessage(imu.getStatus().error);
        if (message == nullptr || serialPort.print(message)) {
            reportedError = imu.getStatus().error;
        }
    }

    static uint8_t bannerLine = 0;
    if (imu.getStatus().ready && bannerLine < 2) {
        const char* message = bannerLine == 0 ? READY_MESSAGE :
            "ax_mms2,ay_mms2,az_mms2,gx_mrads,gy_mrads,gz_mrads\r\n";
        if (serialPort.print(message)) ++bannerLine;
    }

    static ImuSample latest = {};
    static bool fresh = false;
#ifdef IMU_USE_MPU6050
    if (newReading) {
        latest = measurement;
        fresh = true;
    }
#else
    if (newReading && measurement.motionFresh) {
        latest = measurement.motion;
        fresh = true;
    }
#endif

    // Print at 10 Hz while read() keeps hardware acquisition progressing. Integer CSV
    // avoids AVR printf's disabled float formatting. Units: mm/s^2, mrad/s.
    static uint32_t reportTimeUs = 0;
    static char line[64];
    static bool linePending = false;
    if (fresh && bannerLine == 2 && !linePending &&
        static_cast<uint32_t>(nowUs - reportTimeUs) >= 100000UL) {
        int length = snprintf(line, sizeof(line), "%ld,%ld,%ld,%ld,%ld,%ld\r\n",
            static_cast<long>(latest.accelerationMps2.x * 1000),
            static_cast<long>(latest.accelerationMps2.y * 1000),
            static_cast<long>(latest.accelerationMps2.z * 1000),
            static_cast<long>(latest.angularVelocityRadS.x * 1000),
            static_cast<long>(latest.angularVelocityRadS.y * 1000),
            static_cast<long>(latest.angularVelocityRadS.z * 1000));
        linePending = length > 0 && static_cast<size_t>(length) < sizeof(line);
        fresh = false;
        reportTimeUs = nowUs;
    }
    if (linePending && serialPort.print(line)) linePending = false;
}
