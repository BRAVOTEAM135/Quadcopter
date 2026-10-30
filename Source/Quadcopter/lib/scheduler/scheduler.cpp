#include "Scheduler.h"

#include "T1.h"
#include "ledBlink.h"
#include "serialPort.h"
#include "i2cBus.h"
#include "imu.h"
#include "imuFilter.h"

#include <stdint.h>

/*----- Constants -----*/
#define HEARTBEAT_INTERVAL 100000UL     // 100ms - 10Hz heartbeat LED blink
#define SERIAL_INTERVAL    500000UL     // 500ms - 2Hz serial output
#define CONTROL_INTERVAL   5000UL       // 5ms - 200Hz control loop


/*----- External Objects -----*/
extern T1 t1;
extern ledBlink led;
extern SerialPort serialPort;
extern IMU imu;
extern ImuFilter imuFilter;
extern i2cBus bus;


/*----- Module Local Variables -----*/
static uint32_t heartbeatTime = 0;
static uint8_t heartbeatCount = 0;

static uint32_t serialTime = 0;

static uint32_t controlTime = 0;
static uint32_t controlCount = 0;

static ImuSample latestSample = {};
static bool newSample = false;

/*----- Scheduler -----*/
void scheduler_run() {
    uint32_t currentTime = t1.get_micros();

    bus.service(currentTime);
    imu.service(currentTime);

    // Compass data arrives more slowly than the motion samples.
    MagneticSample magnetic;
    if (imu.takeMagneticField(magnetic)) {
        imuFilter.updateMagneticField(magnetic);
    }

    // Collect a completed measurement without waiting.
    if (imu.takeSample(latestSample)) {
        newSample = true;
        // Fuse each completed sample once, using its own timestamp.
        imuFilter.update(latestSample);
    }

    // -------------------------
    // ----- Control Loop ------
    // -------------------------
    if ((uint32_t)(currentTime - controlTime) >= CONTROL_INTERVAL) {
        controlTime += CONTROL_INTERVAL;
        controlCount++;

        // Read IMU
        if (imu.ready()) {
            imu.requestSample(currentTime);
        }
        // Read receiver
        // Calculate PID
        // Update motor commands
    }


    // ---------------------
    // ----- Heartbeat -----
    // ---------------------
    if ((uint32_t)(currentTime - heartbeatTime) >= HEARTBEAT_INTERVAL) {

        heartbeatTime += HEARTBEAT_INTERVAL;
        heartbeatCount++;

        if (heartbeatCount == 1) {
            led.ledOn();
        }
        else if (heartbeatCount == 2) {
            led.ledOff();
        }
        else if (heartbeatCount == 3) {
            led.ledOn();
        }
        else if (heartbeatCount == 4) {
            led.ledOff();
        }
        else if (heartbeatCount >= 10) {
            heartbeatCount = 0;
        }
    }

    // ---------------------
    // ----- Serial -----
    // ---------------------
    if ((uint32_t)(currentTime - serialTime) >= SERIAL_INTERVAL) {
        serialTime += SERIAL_INTERVAL;

        const ImuOrientation& angles = imuFilter.orientation();

        if (angles.valid) {
            serialPort.print(angles.rollDeg(), 2);
            serialPort.print(",");
            serialPort.print(angles.pitchDeg(), 2);
            serialPort.print(",");
            serialPort.println(angles.yawDeg(), 2);
        }
    }
}

/* END OF FILE */
