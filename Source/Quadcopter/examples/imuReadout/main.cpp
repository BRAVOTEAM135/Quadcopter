#include <Arduino.h>
#include <stdio.h>
#include "i2cBus.h"
#include "serialPort.h"

#ifdef IMU_USE_MPU6050
#include "MPU6050.h"
using SelectedImu = MPU6050;
static const char* const READY_MESSAGE = "MPU6050 ready\r\n";
#else
#include "GY80.h"
using SelectedImu = GY80;
static const char* const READY_MESSAGE = "GY80 ready\r\n";
#endif

i2cBus bus;
SerialPort serialPort;
SelectedImu imu(bus);

namespace {
    const char* errorMessage(SensorError error) {
        switch (error) {
            case SensorError::Nack: return "IMU: NACK\r\n";
            case SensorError::BusError: return "IMU: BUS ERROR\r\n";
            case SensorError::Timeout: return "IMU: TIMEOUT\r\n";
            case SensorError::WrongDevice: return "IMU: WRONG DEVICE\r\n";
            case SensorError::InvalidData: return "IMU: INVALID DATA\r\n";
            default: return nullptr;
        }
    }
}

void setup() {
    serialPort.init(115200);
    bus.init();
    imu.begin(micros());
}

void loop() {
    const uint32_t nowUs = micros();
    bus.service(nowUs);
    imu.service(nowUs);

    // Retry complete serial messages without blocking bus service.
    static SensorError reportedError = SensorError::None;
    if (imu.error() != reportedError) {
        const char* message = errorMessage(imu.error());
        if (message == nullptr || serialPort.print(message)) {
            reportedError = imu.error();
        }
    }

    static uint8_t bannerLine = 0;
    if (imu.ready() && bannerLine < 2) {
        const char* message = bannerLine == 0 ? READY_MESSAGE :
            "ax_mms2,ay_mms2,az_mms2,gx_mrads,gy_mrads,gz_mrads\r\n";
        if (serialPort.print(message)) ++bannerLine;
    }

    static uint32_t requestTimeUs = 0;
    if (imu.ready() && static_cast<uint32_t>(nowUs - requestTimeUs) >= 5000UL) {
        if (imu.requestSample(nowUs)) requestTimeUs = nowUs;
    }

    static ImuSample latest = {};
    static bool fresh = false;
    ImuSample sample;
    if (imu.takeSample(sample)) {
        latest = sample;
        fresh = true;
    }

    // Print at 10 Hz while requesting measurements at 200 Hz. Integer CSV
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
