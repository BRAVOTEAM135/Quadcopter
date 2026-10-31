#include <Arduino.h>
#include "ledBlink.h"
#include "serialPort.h"
#include "T1.h"
#include "i2cBus.h"
#include "imu.h"

// Task periods in microseconds.
#define CONTROL_INTERVAL_US   5000UL    // 200 Hz
#define HEARTBEAT_INTERVAL_US 100000UL  // 10 Hz
#define SERIAL_INTERVAL_US    500000UL  // 2 Hz

// Hardware. GY80 is the default; select MPU6050 in platformio.ini.
ledBlink led;
SerialPort serialPort;
T1 t1;
i2cBus bus;
IMU imu(bus);
ImuReading imuReading = {};

// Previous task run times in microseconds.
static uint32_t controlTime = 0;
static uint32_t heartbeatTime = 0;
static uint32_t serialTime = 0;

/// Initialize UART, timer, LED and IMU once at startup.
void setup() {
    serialPort.init(115200);
    t1.init();
    led.init();
    led.ledOff();

    bus.init();
    imu.init(t1.get_micros());
    // Keep the board still while the IMU averages its startup zero.
}

/// Advance the IMU and run due control, heartbeat and serial tasks repeatedly.
void loop() {
    uint32_t currentTime = t1.get_micros();

    // Read new measurements and calculate angles without waiting.
    // Calling read() every pass also advances initialization and transfers.
    imu.read(imuReading, currentTime);

    // Unsigned subtraction handles clock wrap. Advance by the interval to
    // keep each task on schedule, catching up one release per loop if late.

    // ----- Control loop -----
    if (currentTime - controlTime >= CONTROL_INTERVAL_US) {
        controlTime += CONTROL_INTERVAL_US;
        // 1. Read sensor inputs from imuReading.motion and imuReading.orientation.
        //    Check orientation.valid and timestampUs before control use.
        const ImuOrientation& angles = imuReading.orientation;

        if (angles.valid) {
            float roll  = angles.rollDeg();
            float pitch = angles.pitchDeg();
            float yaw   = angles.yawDeg();

            // Use roll, pitch and yaw in your PID calculations.
        }
        // 2. Read receiver demands.       (Not implemented yet.)
        // 3. Calculate PID corrections.   (Not implemented yet.)
        // 4. Mix and write motor outputs. (Not implemented yet.)
    }

    // ----- Heartbeat: two short flashes each second -----
    if (currentTime - heartbeatTime >= HEARTBEAT_INTERVAL_US) {
        heartbeatTime += HEARTBEAT_INTERVAL_US;
        static uint8_t heartbeatCount = 0;
        heartbeatCount++;

        if (heartbeatCount == 1 || heartbeatCount == 3) {
            led.ledOn();
        } else {
            led.ledOff();
        }

        if (heartbeatCount == 10) {
            heartbeatCount = 0;
        }
    }

    // ----- Serial: roll, pitch, yaw in degrees -----
    if (currentTime - serialTime >= SERIAL_INTERVAL_US) {
        serialTime += SERIAL_INTERVAL_US;
        const ImuOrientation& angles = imuReading.orientation;
        if (angles.valid) {
            serialPort.print(angles.timestampUs);
            serialPort.print(",");
            serialPort.print(angles.rollDeg(), 2);
            serialPort.print(",");
            serialPort.print(angles.pitchDeg(), 2);
            serialPort.print(",");
            serialPort.println(angles.yawDeg(), 2);
        }
    }
}
