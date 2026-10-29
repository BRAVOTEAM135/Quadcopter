#include "Scheduler.h"

#include "T1.h"
#include "ledBlink.h"
#include "serialPort.h"

#include <stdint.h>

/*----- Constants -----*/
#define HEARTBEAT_INTERVAL 10000UL  // 10ms
#define SERIAL_INTERVAL    500000UL  // 500ms


/*----- External Objects -----*/
extern T1 t1;
extern ledBlink led;
extern SerialPort serialPort;


/*----- Module Local Variables -----*/
static uint32_t heartbeatTime = 0;
static uint8_t heartbeatCount = 0;

static uint32_t serialTime = 0;

/*----- Scheduler -----*/
void scheduler_run() {
    uint32_t currentTime = t1.get_micros();

    // ---------------------
    // ----- Heartbeat -----
    // ---------------------
    if ((uint32_t)(currentTime - heartbeatTime) >= HEARTBEAT_INTERVAL) {

        heartbeatTime = currentTime;
        heartbeatCount++;

        if (heartbeatCount == 1) {
            led.ledOn();
        }
        else if (heartbeatCount == 5) {
            led.ledOff();
        }
        else if (heartbeatCount == 10) {
            led.ledOn();
        }
        else if (heartbeatCount == 15) {
            led.ledOff();
        }
        else if (heartbeatCount >= 100) {
            heartbeatCount = 0;
        }
    }

    // ---------------------
    // ----- Serial -----
    // ---------------------
    if ((uint32_t)(currentTime - serialTime) >= SERIAL_INTERVAL) {
        // serialTime = currentTime;
        serialTime += SERIAL_INTERVAL;
        serialPort.print("Count: ");
        serialPort.println(t1.get_count());
    }
}

/* END OF FILE */